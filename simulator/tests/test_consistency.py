"""Cross-file consistency checks: firmware pin header vs. the pin table and
wiring diagram; firmware geometry defaults vs. simulator and CAD."""
import csv
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def pins_h():
    src = (ROOT / "firmware/src/hal_esp32/pins.h").read_text()
    out = {}
    m = re.search(r"kSensor\[\d+\]\s*=\s*\{([^}]*)\}", src)
    body = re.sub(r"/\*.*?\*/", "", m.group(1))  # drop /*wings*/ style comments
    for i, v in enumerate(body.split(",")):
        out[f"sensor{i}"] = int(v)
    for name, val in re.findall(r"constexpr uint8_t (k\w+) = (\d+);", src):
        out[name] = int(val)
    return out


def pin_table():
    return {r["gpio"]: r for r in csv.DictReader((ROOT / "hardware/pin_mapping.csv").open())}


def test_every_firmware_pin_is_in_the_table():
    t = pin_table()
    p = pins_h()
    for name, gpio in p.items():
        if name in ("kPwmBits", "kPwmChA", "kPwmChB", "kPwmChA2", "kPwmChB2"):
            continue
        assert str(gpio) in t, f"{name}=GPIO{gpio} missing from pin_mapping.csv"


def test_table_symbols_match_header():
    t = pin_table()
    p = pins_h()
    for gpio, row in t.items():
        sym = row["firmware_symbol"].strip()
        if not sym:
            continue
        m = re.match(r"pins::(\w+)(?:\[(\d)\])?", sym)
        key = f"sensor{m.group(2)}" if m.group(1) == "kSensor" else m.group(1)
        assert p[key] == int(gpio), f"{sym}: header GPIO{p[key]} vs table GPIO{gpio}"


def test_no_pin_used_twice_and_no_forbidden_pins():
    p = pins_h()
    used = [v for k, v in p.items() if k not in ("kPwmBits", "kPwmChA", "kPwmChB", "kPwmChA2", "kPwmChB2")]
    assert len(used) == len(set(used))
    assert not set(used) & {6, 7, 8, 9, 10, 11, 12}, "flash pins or GPIO12 strapping used"
    outputs = [p[k] for k in ("kPwmA", "kAin1", "kAin2", "kPwmB", "kBin1", "kBin2", "kStby", "kLedRed", "kLedStatus")]
    assert not set(outputs) & {34, 35, 36, 39}, "input-only pin used as output"


def _cad_params():
    return (ROOT / "hardware/cad/robot_cad.py").read_text()


def test_firmware_sim_cad_geometry_match():
    cfg = (ROOT / "firmware/include/config.h").read_text()

    def fw(name):
        return float(re.search(rf"X\(\w+, {name}, ([\d.]+)f?,", cfg).group(1))

    cad = _cad_params()

    def cadv(name):
        return float(re.search(rf"    {name}: float = ([\d.]+)", cad).group(1))

    from simulator.config import SimParams, n20_hardware

    s = SimParams()
    assert fw("wheel_diameter_mm") == s.wheel_diameter_mm
    assert fw("track_width_mm") == s.track_width_mm
    assert fw("sensor_forward_mm") == s.sensor_forward_mm
    # the parametric CAD models the original N20 design (n20_hardware preset)
    n20 = n20_hardware()
    assert cadv("wheel_d") == n20["wheel_diameter_mm"]
    assert cadv("track") == n20["track_width_mm"]
    assert cadv("sensor_fwd") == n20["sensor_forward_mm"]
    assert fw("sensor_pitch_mm") == s.sensor_pitch_mm
    assert fw("counts_per_wheel_rev") == s.counts_per_wheel_rev
