"""Decides whether the ultrasonic sensors can be trusted. Pure Python.

An HC-SR04 that is unpowered or has a loose wire looks exactly like "nothing in
range" (no echo). To avoid driving blind into a wall because VCC/GND was never
connected, a sensor that reports *no echo continuously* for `max_noecho_s`
seconds is treated as faulty. Set max_noecho_s to 0 to disable this check
(e.g. for a big open hall).
"""

NAMES = ("left", "center", "right")


class SensorHealth:
    def __init__(self, names=NAMES, timeout=0.5, max_noecho_s=8.0):
        self.names = tuple(names)
        self.timeout = timeout
        self.max_noecho_s = max_noecho_s
        self.last_seen = {n: None for n in self.names}
        self.noecho_since = {n: None for n in self.names}

    def update(self, name, distance, now):
        """distance in metres, or None for 'no echo'."""
        self.last_seen[name] = now
        if distance is None:
            if self.noecho_since[name] is None:
                self.noecho_since[name] = now
        else:
            self.noecho_since[name] = None

    def status(self, now):
        """Return (ok, reason)."""
        for n in self.names:
            seen = self.last_seen[n]
            if seen is None:
                return False, "no data from %s sensor" % n
            if now - seen > self.timeout:
                return False, "%s sensor data stale" % n
            since = self.noecho_since[n]
            if self.max_noecho_s > 0 and since is not None and now - since > self.max_noecho_s:
                return False, "%s sensor stuck on no-echo (check VCC/GND/wiring)" % n
        return True, "ok"
