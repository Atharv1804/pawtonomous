"""UPS HAT (E) register decoding (tools/robot_power.py), with fake registers - no hardware needed."""
import importlib.util
import os


def _load():
    path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools", "robot_power.py")
    spec = importlib.util.spec_from_file_location("robot_power", os.path.abspath(path))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _regs(**words):
    """Build a register reader from 16-bit values keyed by register address name, e.g. r0x24=49."""
    regs = {0x00: 0x0A, 0x02: 0x00}
    for name, value in words.items():
        reg = int(name[1:], 16)
        value &= 0xFFFF
        regs[reg], regs[reg + 1] = value & 0xFF, value >> 8
    return lambda r: regs.get(r, 0)


def test_discharging_status():
    m = _load()
    s = m.decode(_regs(r0x24=49, r0x20=14790, r0x22=-585, r0x26=2319, r0x28=238, r0x2A=0xFFFF,
                       r0x30=3690, r0x32=3698, r0x34=3702, r0x36=3700))
    assert s["ok"] and s["percent"] == 49 and s["battery_v"] == 14.79
    assert s["current_ma"] == -585 and not s["charging"] and s["minutes_left"] == 238
    assert "minutes_to_full" not in s and "warning" not in s
    assert s["cells_v"] == [3.69, 3.698, 3.702, 3.7]


def test_charging_and_low_warning():
    m = _load()
    s = m.decode(_regs(r0x24=12, r0x22=900, r0x10=5100, r0x2A=95, r0x28=0xFFFF))
    assert s["charging"] and s["usb_c_power_in"] and s["minutes_to_full"] == 95
    assert "warning" not in s                       # low but charging: no warning
    low = m.decode(_regs(r0x24=12, r0x22=-400))
    assert "warning" in low


def test_wrong_device_id():
    m = _load()
    assert m.decode(lambda r: 0x55)["ok"] is False
