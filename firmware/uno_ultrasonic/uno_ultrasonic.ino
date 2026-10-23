/*
  UNO: 3x HC-SR04 ultrasonic -> serial -> Raspberry Pi (ROS ultrasonic_bridge)

  Wiring (HC-SR04 -> Uno):          LEFT    CENTER   RIGHT
    TRIG                            D2      D4       D8
    ECHO                            D3      D5       D9
  (RIGHT moved from D6/D7 to D8/D9 on 2026-10-22: no echo on D6/D7 even with a new sensor)
    VCC -> 5V (via breadboard rail)  GND -> GND  (all three share 5V and GND)

  Output, about 10 times a second, one line over USB serial (115200 baud):
      D,<left_mm>,<center_mm>,<right_mm>
  -1 means "no echo" (nothing within ~4 m, or the surface absorbed the pulse).
  A reading closer than 20 mm is reported as 20 (blocked), not as "clear".
*/

const uint8_t TRIG[3] = {2, 4, 8};
const uint8_t ECHO[3] = {3, 5, 9};

const unsigned long ECHO_TIMEOUT_US = 25000UL;  // ~4.3 m round trip
const unsigned long GAP_MS = 30;                // pause between sensors (avoids cross-talk)

long readMM(uint8_t i) {
  digitalWrite(TRIG[i], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);

  unsigned long us = pulseIn(ECHO[i], HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;                 // no echo
  long mm = (long)(us * 0.1715);          // speed of sound, round trip
  if (mm > 4000) return -1;
  if (mm < 20) mm = 20;                   // too close to measure: treat as blocked
  return mm;
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    digitalWrite(TRIG[i], LOW);
  }
}

void loop() {
  long d[3];
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = readMM(i);
    delay(GAP_MS);
  }
  Serial.print("D,");
  Serial.print(d[0]);
  Serial.print(",");
  Serial.print(d[1]);
  Serial.print(",");
  Serial.println(d[2]);
}
