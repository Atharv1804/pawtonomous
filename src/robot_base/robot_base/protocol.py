"""Wire formats shared by the nodes. Pure Python, no ROS imports.

Pi -> ESP32 (UDP text):   "<key> VEL <left> <right>"  (each -255..255), or "<key> STOP"
                          The ESP32 ignores packets without its secret key (see firmware secrets.h).
Uno -> Pi  (serial line): "D,<left_mm>,<center_mm>,<right_mm>"  (-1 = no echo)
"""

STOP = b"STOP"
MAX_LINE = 512


def with_key(payload, key=""):
    """Prefix a command with the shared secret key: b"STOP" -> b"<key> STOP"."""
    return (key.encode("ascii") + b" " + payload) if key else payload


def format_vel(left, right, key=""):
    return with_key(("VEL %d %d" % (int(left), int(right))).encode("ascii"), key)


def parse_range_line(line):
    """Parse 'D,230,1500,-1' -> [0.23, 1.5, None] (metres; None = no echo).

    Returns None if the line is not a valid distance frame."""
    line = line.strip()
    if not line.startswith("D,"):
        return None
    parts = line.split(",")
    if len(parts) != 4:
        return None
    out = []
    for p in parts[1:]:
        try:
            mm = int(p)
        except ValueError:
            return None
        out.append(None if mm < 0 else mm / 1000.0)
    return out


class LineBuffer:
    """Collects raw serial bytes and returns complete text lines."""

    def __init__(self):
        self._buf = b""

    def feed(self, data):
        self._buf += data
        lines = []
        while b"\n" in self._buf:
            raw, self._buf = self._buf.split(b"\n", 1)
            lines.append(raw.decode("ascii", errors="ignore").strip("\r "))
        if len(self._buf) > MAX_LINE:  # garbage with no newline: drop it
            self._buf = b""
        return lines
