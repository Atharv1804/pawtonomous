/*
  ESP32 MOTOR NODE - joins your WiFi, takes UDP commands from the Pi.

  Commands (UDP, port 4210, plain text), each prefixed with the secret key:
    <key> VEL <left> <right>   each -255..255 (+ = forward)   <- used by ROS motor_bridge
    <key> STOP
    <key> PING                 replies PONG (handy connectivity test)
    <key> FWD / REV / LEFT / RIGHT   (old commands, still work)
  Packets without the right key are ignored, so other devices on the WiFi cannot drive it.
  The same commands also work over USB serial (115200 baud, one per line, no key: the cable is
  physical access) - tools/keyboard_drive.py --serial uses this.

  Safety: if no command arrives for 500 ms while moving, it stops by itself.

  1) Copy secrets.h.example to secrets.h and fill in WiFi (2.4 GHz) + ROBOT_KEY.
     The Pi reads the same key from ~/robot_ws/.robot_udp_key.
  2) Pick your motor driver in DRIVER_MODE and check the pins.
  3) Board: "DOIT ESP32 DEVKIT V1", Serial Monitor 115200.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

// ---------------- WiFi + key: in secrets.h (never commit it) ----------------
#include "secrets.h"   // defines WIFI_SSID, WIFI_PASSWORD, ROBOT_KEY
const char* HOSTNAME      = "robot-motors";   // Pi can reach it as robot-motors.local

const unsigned int UDP_PORT = 4210;

// ---------------- Which motor driver? ----------------
#define DRIVER_BTS7960 1   // 2 PWM pins (RPWM/LPWM) + enable pins per side
#define DRIVER_DIR_PWM 2   // 1 PWM pin + direction pin(s) per side (typical dual H-bridge)
#define DRIVER_MODE DRIVER_BTS7960   // 2x BTS7960 (since 2026-10-22)

#if DRIVER_MODE == DRIVER_BTS7960
// Left bridge
const int L_REN = 17, L_LEN = 16, L_RPWM = 22, L_LPWM = 23;
// Right bridge
const int R_REN = 32, R_LEN = 33, R_RPWM = 26, R_LPWM = 25;   // RPWM moved 14 -> 26 (2026-10-23: no signal via D14)
#else
// New dual-channel driver. Channel A = LEFT motors, channel B = RIGHT motors.
// PWM goes to the EN pin; IN1/IN2 set the direction. (Same ESP32 pins as before.)
//   A: EN=22  IN1=16  IN2=17        B: EN=25  IN1=32  IN2=33
const int L_PWM = 22, L_IN1 = 16, L_IN2 = 17;
const int R_PWM = 25, R_IN1 = 32, R_IN2 = 33;
#endif

// ---------------- Wiring fixes (no re-wiring needed) ----------------
// Run `keyboard_drive.py --test` and set these so "left forward" really is the left wheels forward.
#define SWAP_SIDES   0   // 1 = the "left" pins actually drive the right wheels
#define INVERT_LEFT  0   // 1 = left wheels turn backwards when told forward
#define INVERT_RIGHT 1   // 1 = right wheels turn backwards when told forward
// The right driver cannot run its wheels backwards (RPWM input dead, also on GPIO26). 1 = never ask it to:
// a right-wheel "reverse" becomes "stop", so a right turn pivots on the left wheels instead of spinning.
#define RIGHT_FORWARD_ONLY 1

// ---------------- Smooth starts/stops ----------------
// The robot is heavy: jumping straight to full power jerks it. VEL commands set a TARGET and the
// output ramps toward it at RAMP_PER_SEC (0 -> 255 in ~0.35 s). STOP and the safety timeout are
// still immediate.
const float RAMP_PER_SEC = 700.0;
float curL = 0, curR = 0;
int targetL = 0, targetR = 0;
unsigned long lastRamp = 0;

// ---------------- Behaviour ----------------
const int DRIVE_SPEED = 200;            // for the old FWD/REV/LEFT/RIGHT commands
const unsigned long CMD_TIMEOUT = 500;  // ms

WiFiUDP udp;
bool udpStarted = false;
unsigned long lastCmdTime = 0;
unsigned long lastWifiTry = 0;
bool isMoving = false;
char packetBuffer[96];   // key (up to ~40 chars) + command

// side: 0 = left, 1 = right. speed: -255..255
void drive(int side, int speed) {
  speed = constrain(speed, -255, 255);
#if DRIVER_MODE == DRIVER_BTS7960
  int rp = side == 0 ? L_RPWM : R_RPWM;
  int lp = side == 0 ? L_LPWM : R_LPWM;
  if (speed >= 0) { analogWrite(rp, speed); analogWrite(lp, 0); }
  else            { analogWrite(rp, 0);     analogWrite(lp, -speed); }
#else
  int pwm = side == 0 ? L_PWM : R_PWM;
  int in1 = side == 0 ? L_IN1 : R_IN1;
  int in2 = side == 0 ? L_IN2 : R_IN2;
  bool fwd = speed >= 0;
  digitalWrite(in1, fwd ? HIGH : LOW);
  if (in2 >= 0) digitalWrite(in2, fwd ? LOW : HIGH);
  analogWrite(pwm, abs(speed));
#endif
}

void applyDrive(int l, int r) {
  if (RIGHT_FORWARD_ONLY && r < 0) r = 0;
  bool moving = (l != 0 || r != 0);
  if (SWAP_SIDES) { int t = l; l = r; r = t; }
  if (INVERT_LEFT) l = -l;
  if (INVERT_RIGHT) r = -r;
  drive(0, l);
  drive(1, r);
  isMoving = moving;
}

// Commanded motion: ramps toward (l, r) in rampTick().
void setDrive(int l, int r) {
  targetL = l;
  targetR = r;
  isMoving = (l != 0 || r != 0) || curL != 0 || curR != 0;
}

void rampTick() {
  unsigned long now = millis();
  float step = RAMP_PER_SEC * (now - lastRamp) / 1000.0;
  lastRamp = now;
  if (step <= 0) return;
  float dl = targetL - curL, dr = targetR - curR;
  curL += (fabs(dl) <= step) ? dl : (dl > 0 ? step : -step);
  curR += (fabs(dr) <= step) ? dr : (dr > 0 ? step : -step);
  applyDrive((int)curL, (int)curR);
}

// Immediate stop (STOP command, safety timeout): no ramp.
void stopAll() {
  targetL = targetR = 0;
  curL = curR = 0;
  applyDrive(0, 0);
  isMoving = false;
}

void setupPins() {
#if DRIVER_MODE == DRIVER_BTS7960
  int outs[] = {L_REN, L_LEN, R_REN, R_LEN, L_RPWM, L_LPWM, R_RPWM, R_LPWM};
  for (int p : outs) pinMode(p, OUTPUT);
  digitalWrite(L_REN, HIGH); digitalWrite(L_LEN, HIGH);
  digitalWrite(R_REN, HIGH); digitalWrite(R_LEN, HIGH);
#else
  int outs[] = {L_PWM, L_IN1, R_PWM, R_IN1};
  for (int p : outs) pinMode(p, OUTPUT);
  if (L_IN2 >= 0) pinMode(L_IN2, OUTPUT);
  if (R_IN2 >= 0) pinMode(R_IN2, OUTPUT);
#endif
}

// Runs one command. Returns true if it was PING (the caller replies PONG on its own channel).
bool runCommand(const char* buf) {
  int l, r;
  if (strncmp(buf, "VEL ", 4) == 0) {
    if (sscanf(buf + 4, "%d %d", &l, &r) == 2) {
      setDrive(l, r);
      lastCmdTime = millis();
    }
  } else if (strcmp(buf, "STOP") == 0) {
    stopAll();
    lastCmdTime = millis();
  } else if (strcmp(buf, "FWD") == 0)   { setDrive( DRIVE_SPEED,  DRIVE_SPEED); lastCmdTime = millis(); }
  else if (strcmp(buf, "REV") == 0)     { setDrive(-DRIVE_SPEED, -DRIVE_SPEED); lastCmdTime = millis(); }
  else if (strcmp(buf, "LEFT") == 0)    { setDrive(-DRIVE_SPEED,  DRIVE_SPEED); lastCmdTime = millis(); }
  else if (strcmp(buf, "RIGHT") == 0)   { setDrive( DRIVE_SPEED, -DRIVE_SPEED); lastCmdTime = millis(); }
  else if (strcmp(buf, "PING") == 0) return true;
  return false;
}

void handlePacket(const char* buf) {
  // Require "<ROBOT_KEY> " in front; anything else is ignored (no reply).
  size_t klen = strlen(ROBOT_KEY);
  if (klen < 8 || strncmp(buf, ROBOT_KEY, klen) != 0 || buf[klen] != ' ') return;
  if (runCommand(buf + klen + 1)) {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("PONG");
    udp.endPacket();
  }
}

// USB serial commands: one per line, e.g. "VEL 120 120\n".
char serialLine[48];
size_t serialLen = 0;

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLen > 0) {
        serialLine[serialLen] = '\0';
        if (strcmp(serialLine, "SCAN") == 0) {          // list the WiFi networks this ESP32 can join
          WiFi.disconnect();                          // a scan fails while a connect attempt runs
          delay(200);
          int n = WiFi.scanNetworks();
          Serial.printf("SCAN FOUND %d\n", n);
          for (int i = 0; i < n; i++) {
            Serial.printf("NET %d dBm ch%d %s\n", WiFi.RSSI(i), WiFi.channel(i), WiFi.SSID(i).c_str());
          }
          Serial.println("SCAN DONE");
        } else if (runCommand(serialLine)) Serial.println("PONG");
        serialLen = 0;
      }
    } else if (serialLen < sizeof(serialLine) - 1) {
      serialLine[serialLen++] = c;
    } else {
      serialLen = 0;                              // overlong garbage: drop it
    }
  }
}

void setup() {
  Serial.begin(115200);
  setupPins();
  stopAll();

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 3000) {   // short: USB commands wait meanwhile
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  // If it did not connect, loop() keeps retrying; motors stay stopped meanwhile.
}

void loop() {
  // ---- keep WiFi up, start UDP + mDNS once connected ----
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udp.begin(UDP_PORT);
      if (MDNS.begin(HOSTNAME)) MDNS.addService("robot", "udp", UDP_PORT);
      udpStarted = true;
      Serial.print("WiFi connected. IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("Name: ");
      Serial.print(HOSTNAME);
      Serial.print(".local  UDP port ");
      Serial.println(UDP_PORT);
    }
  } else if (millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println("WiFi not connected, retrying...");
    WiFi.reconnect();
  }

  // ---- commands ----
  pollSerial();
  if (udpStarted) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (len > 0) {
        packetBuffer[len] = '\0';
        // trim trailing whitespace/newlines
        while (len > 0 && (packetBuffer[len - 1] == '\n' || packetBuffer[len - 1] == '\r' || packetBuffer[len - 1] == ' ')) {
          packetBuffer[--len] = '\0';
        }
        handlePacket(packetBuffer);
      }
    }
  }

  rampTick();

  // ---- safety: stop if commands stop arriving ----
  if (isMoving && (millis() - lastCmdTime > CMD_TIMEOUT)) {
    stopAll();
    Serial.println("No command - auto-stopped.");
  }
}
