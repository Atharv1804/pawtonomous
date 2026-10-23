"""Battery status of the Waveshare UPS HAT (E) that powers the Pi (read-only, I2C bus 1, address 0x2D).

Register map: https://www.waveshare.com/wiki/UPS_HAT_(E)_Register (16-bit values, low byte first).
This module ONLY reads. Never write to the HAT: writing 0x55 to register 0x01 powers the Pi off.
"""

import os

I2C_BUS = int(os.environ.get("ROBOT_UPS_BUS", "1"))
I2C_ADDR = int(os.environ.get("ROBOT_UPS_ADDR", "0x2d"), 16)
CHARGE_STATES = {0: "standby", 1: "trickle charging", 2: "charging (constant current)",
                 3: "charging (constant voltage)", 4: "charge pending", 5: "full", 6: "charge timeout"}


def _u16(read, reg):
    return read(reg) | (read(reg + 1) << 8)


def _s16(read, reg):
    v = _u16(read, reg)
    return v - 65536 if v > 32767 else v


def decode(read):
    """Turn register reads (read(reg) -> byte) into a status dict. Separate from I/O so it can be tested."""
    if read(0x00) != 0x0A:
        return {"ok": False, "error": "UPS HAT not found at 0x%02x (wrong ID 0x%02x)" % (I2C_ADDR, read(0x00))}
    status = read(0x02)
    current_ma = _s16(read, 0x22)
    usb_mv = _u16(read, 0x10)
    out = {
        "ok": True,
        "percent": _u16(read, 0x24),
        "battery_v": round(_u16(read, 0x20) / 1000.0, 2),
        "current_ma": current_ma,                      # + charging, - powering the Pi
        "charging": current_ma > 0,
        "usb_c_power_in": usb_mv > 4000,
        "charge_state": CHARGE_STATES.get(status & 0x07, "unknown"),
        "remaining_mah": _u16(read, 0x26),
        "cells_v": [round(_u16(read, r) / 1000.0, 3) for r in (0x30, 0x32, 0x34, 0x36)],
    }
    minutes = _u16(read, 0x2A if out["charging"] else 0x28)
    if minutes != 0xFFFF:                              # 65535 = not applicable
        out["minutes_to_full" if out["charging"] else "minutes_left"] = minutes
    if out["percent"] <= 15 and not out["charging"]:
        out["warning"] = "battery low: plug in USB-C to charge the UPS HAT"
    return out


def battery_status():
    try:
        from smbus2 import SMBus
    except ImportError:
        return {"ok": False, "error": "smbus2 is not installed in the MCP server venv"}
    try:
        with SMBus(I2C_BUS) as bus:
            return decode(lambda reg: bus.read_byte_data(I2C_ADDR, reg))
    except OSError as e:
        return {"ok": False, "error": "cannot read the UPS HAT over I2C: %s" % e}
