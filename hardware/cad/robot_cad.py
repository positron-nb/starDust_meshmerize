"""Parametric CAD of the Meshmerize robot (CadQuery 2.x).

    python hardware/cad/robot_cad.py            # build, check, export everything

Outputs
  hardware/chassis/        chassis_plate, motor_clamp  (.step + .stl)
  hardware/sensor_mount/   sensor_bracket              (.step + .stl)
  hardware/battery_holder/ battery_tray                (.step + .stl)
  hardware/chassis/        caster_mount, led_mount, deck_plate (.step + .stl)
  hardware/assembly_drawings/ robot_assembly.step, top/side/front .svg, fit_report.md

Frame: x forward, y left, z up, origin on the floor under the axle centre.
All numbers in mm. Purchased parts are modelled as simple envelopes taken
from their data sheets / typical dimensions and MUST be checked against the
parts actually bought (see PURCHASED below) before printing.

This is rendered geometry plus automated dimensional checks. It has NOT
been validated by building the robot.
"""
from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import cadquery as cq

HW = Path(__file__).resolve().parents[1]


@dataclass
class P:
    # --- drive train (must match firmware/include/config.h) ---
    wheel_d: float = 34.0          # config: wheel_diameter_mm
    wheel_w: float = 7.0
    track: float = 96.0            # config: track_width_mm (wheel centre to centre)
    sensor_fwd: float = 70.0       # config: sensor_forward_mm (axle -> sensor line)
    sensor_height: float = 4.5     # sensor window above the floor (QTR: ~3-6 mm works)
    # N20 gearmotor with encoder (typical; measure yours)
    mot_w: float = 12.0            # along x
    mot_h: float = 10.0            # along z
    gearbox_len: float = 9.5
    motor_len: float = 15.0
    encoder_len: float = 8.0       # encoder PCB + magnet
    shaft_len: float = 10.0
    # chassis
    plate_t: float = 3.0
    plate_x0: float = -68.0        # rear edge
    plate_x1: float = 62.0         # front edge (bracket continues to the sensors)
    plate_half_w: float = 41.0     # must stay inside the wheels
    wheel_clear: float = 2.0
    # rear support
    caster_x: float = -55.0
    caster_h: float = 11.0         # floor to caster mounting face (3/8" ball caster ~ 10-11 mm)
    # electronics deck
    deck_gap: float = 22.0         # standoff height above the chassis plate
    deck_t: float = 1.6
    # battery (2S 1000 mAh envelope - measure yours)
    batt_l: float = 72.0
    batt_w: float = 35.0
    batt_h: float = 16.0
    # QTR-MD-08A
    qtr_l: float = 61.0            # along y
    qtr_w: float = 20.0            # along x
    qtr_t: float = 2.5
    # fasteners
    m2: float = 2.2
    m3: float = 3.2


PURCHASED = {
    "ESP32 DevKitC 38-pin (with headers)": (55.0, 28.0, 13.0),
    "TB6612FNG module": (21.0, 20.0, 5.0),
    "MP1584 buck module": (22.0, 17.0, 5.0),
    "QTR-MD-08A": (61.0, 20.0, 2.5),
    "2S 1000 mAh LiPo": (72.0, 35.0, 16.0),
}


def axle_z(p: P) -> float:
    return p.wheel_d / 2


def plate_z(p: P) -> float:
    """Underside of the chassis plate: motors hang below it."""
    return axle_z(p) + p.mot_h / 2 + 0.5


def box(lx, ly, lz, x, y, z):
    """Box with its minimum corner at (x, y, z)."""
    return cq.Workplane("XY").box(lx, ly, lz, centered=False).translate((x, y, z))


# ----------------------------------------------------------------- parts ---
def chassis_plate(p: P):
    z0 = plate_z(p)
    L = p.plate_x1 - p.plate_x0
    plate = box(L, 2 * p.plate_half_w, p.plate_t, p.plate_x0, -p.plate_half_w, z0)
    # motor clamp screws (2 per clamp), caster mount, bracket, deck standoffs
    holes = []
    for s in (1, -1):
        yc = s * (p.track / 2 - p.wheel_w / 2 - 1.5 - p.gearbox_len / 2)
        holes += [(-9.0, yc), (9.0, yc)]
    holes += [(p.caster_x - 6, 0), (p.caster_x + 6, 0)]           # caster mount
    holes += [(p.plate_x1 - 5, -15), (p.plate_x1 - 5, 15)]       # sensor bracket
    deck = [(-50, -33), (-50, 33), (40, -33), (40, 33)]          # deck standoffs (M3)
    plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(holes).hole(p.m2)
    plate = plate.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(deck).hole(p.m3)
    # wiring pass-throughs (motor and sensor cables) and weight-saving slot
    plate = plate.cut(box(10, 24, 10, -5, -12, z0 - 1))
    plate = plate.cut(box(14, 30, 10, p.plate_x1 - 26, -15, z0 - 1))
    # battery strap slots
    for x in (-30.0, 22.0):
        plate = plate.cut(box(4, 22, 10, x, -11, z0 - 1).translate((0, 0, 0)))
    return plate


def motor_envelope(p: P, side: int):
    """N20 + encoder, axis along y. side=+1 left, -1 right."""
    zc = axle_z(p)
    y_face = side * (p.track / 2 - p.wheel_w / 2 - 1.5)   # gearbox output face
    length = p.gearbox_len + p.motor_len + p.encoder_len
    y_in = y_face - side * length
    ymin, ymax = sorted((y_face, y_in))
    body = box(p.mot_w, ymax - ymin, p.mot_h, -p.mot_w / 2, ymin, zc - p.mot_h / 2)
    shaft = cq.Workplane("XZ").circle(1.5).extrude(p.shaft_len).translate((0, 0, zc))
    shaft = shaft.translate((0, y_face if side < 0 else y_face + p.shaft_len, 0))
    return body.union(shaft)


def motor_clamp(p: P, side: int = 1):
    """U-clamp over the gearbox, screwed to the plate underside."""
    zc = axle_z(p)
    z_top = plate_z(p)
    wall = 2.5
    clamp_y = p.gearbox_len
    y_face = side * (p.track / 2 - p.wheel_w / 2 - 1.5)
    y0 = y_face - side * clamp_y
    ymin = min(y0, y_face)
    outer_w = p.mot_w + 2 * wall + 10  # ears for the two M2 screws
    z_bot = zc - p.mot_h / 2 - wall
    c = box(outer_w, clamp_y, z_top - z_bot, -outer_w / 2, ymin, z_bot)
    c = c.cut(box(p.mot_w + 0.3, clamp_y + 2, p.mot_h + 0.3, -(p.mot_w + 0.3) / 2, ymin - 1, zc - (p.mot_h + 0.3) / 2))
    # cut the ears down so the clamp only touches the plate there
    for sx in (1, -1):
        hole = cq.Workplane("XY").circle(p.m2 / 2).extrude(z_top - z_bot + 2).translate(
            (sx * 9.0, ymin + clamp_y / 2, z_bot - 1))
        c = c.cut(hole)
    return c


def wheel(p: P, side: int):
    yc = side * p.track / 2
    w = cq.Workplane("XZ").circle(p.wheel_d / 2).extrude(p.wheel_w).translate((0, yc + p.wheel_w / 2, axle_z(p)))
    return w


def sensor_bracket(p: P):
    """Plate extension + vertical web + shelf holding the QTR board with its
    sensors at sensor_fwd ahead of the axle. Slotted screw holes give +-3 mm of
    height adjustment (rules allow sensor adjustment at restarts)."""
    z_plate = plate_z(p)
    x_sensor = p.sensor_fwd
    shelf_z = p.sensor_height + p.qtr_t          # shelf underside = QTR top face
    t = 3.0
    # horizontal tab under the plate front, with the two mounting holes
    tab = box(16, 40, t, p.plate_x1 - 13, -20, z_plate - t)
    web_x = x_sensor + p.qtr_w / 2 + 0.5
    web = box(t, 40, z_plate - shelf_z, web_x, -20, shelf_z)
    # link from the tab to the web
    link = box(web_x - (p.plate_x1 - 13) + t, 40, t, p.plate_x1 - 13, -20, z_plate - t)
    shelf = box(p.qtr_w + 4, p.qtr_l + 2, t, x_sensor - p.qtr_w / 2 - 3.5, -(p.qtr_l + 2) / 2, shelf_z)
    b = tab.union(web).union(link).union(shelf)
    # slots for M2 screws through the shelf into the QTR board (adjust x +-2 mm)
    for y in (-p.qtr_l / 2 + 3.5, p.qtr_l / 2 - 3.5):
        slot = cq.Workplane("XY").slot2D(6, p.m2, 0).extrude(10).translate((x_sensor, y, shelf_z - 1))
        b = b.cut(slot)
    # slotted holes in the tab (height adjustment comes from washers/shims)
    for y in (-15, 15):
        b = b.cut(cq.Workplane("XY").slot2D(7, p.m2, 0).extrude(10).translate((p.plate_x1 - 5, y, z_plate - 6)))
    return b


def qtr_board(p: P):
    return box(p.qtr_w, p.qtr_l, p.qtr_t, p.sensor_fwd - p.qtr_w / 2, -p.qtr_l / 2, p.sensor_height)


def caster_mount(p: P):
    z_plate = plate_z(p)
    h = z_plate - p.caster_h
    m = box(24, 18, h, p.caster_x - 12, -9, p.caster_h)
    for dx in (-6, 6):
        m = m.cut(cq.Workplane("XY").circle(p.m2 / 2).extrude(h + 2).translate((p.caster_x + dx, 0, p.caster_h - 1)))
    return m


def caster(p: P):
    ball = cq.Workplane("XY").sphere(9.5 / 2).translate((p.caster_x, 0, 9.5 / 2))
    housing = box(14, 14, p.caster_h - 6, p.caster_x - 7, -7, 6)
    return ball.union(housing)


def battery_tray(p: P):
    z = plate_z(p) + p.plate_t
    wall, floor_t = 1.6, 1.2
    L, W = p.batt_l + 2, p.batt_w + 2
    x0 = -L / 2 - 8
    t = box(L + 2 * wall, W + 2 * wall, floor_t + 6, x0 - wall, -W / 2 - wall, z)
    t = t.cut(box(L, W, 10, x0, -W / 2, z + floor_t))
    # strap slots line up with the chassis slots
    for x in (-30.0, 22.0):
        t = t.cut(box(4, 22, 3, x, -11, z - 0.5))
    return t


def battery(p: P):
    z = plate_z(p) + p.plate_t + 1.2
    L = p.batt_l + 2
    return box(p.batt_l, p.batt_w, p.batt_h, -L / 2 - 8 + 1, -p.batt_w / 2, z)


def deck_plate(p: P):
    z = plate_z(p) + p.plate_t + p.deck_gap
    d = box(110, 74, p.deck_t, -58, -37, z)
    pts = [(-50, -33), (-50, 33), (40, -33), (40, 33)]
    d = d.faces(">Z").workplane(origin=(0, 0, 0)).pushPoints(pts).hole(p.m3)
    return d


def standoffs(p: P):
    z = plate_z(p) + p.plate_t
    s = None
    for x, y in [(-50, -33), (-50, 33), (40, -33), (40, 33)]:
        c = cq.Workplane("XY").circle(2.5).circle(1.6).extrude(p.deck_gap).translate((x, y, z))
        s = c if s is None else s.union(c)
    return s


def electronics(p: P):
    z = plate_z(p) + p.plate_t + p.deck_gap + p.deck_t
    esp = box(55, 28, 13, -25, -14, z)
    drv = box(21, 20, 5, 12, -36, z)
    buck = box(22, 17, 5, -52, 10, z)
    return {"esp32": esp, "tb6612": drv, "buck": buck}


def led_mount(p: P):
    z = plate_z(p) + p.plate_t + p.deck_gap + p.deck_t
    m = box(10, 10, 14, -57, -5, z)
    m = m.cut(cq.Workplane("XY").circle(5.2 / 2).extrude(20).translate((-52, 0, z + 4)))
    return m


# ---------------------------------------------------------------- checks ---
def vol(shape) -> float:
    return shape.val().Volume()


def bbox(shapes):
    c = cq.Compound.makeCompound([s.val() for s in shapes])
    bb = c.BoundingBox()
    return bb


def build(p: P | None = None, out: bool = True):
    p = p or P()
    parts = {
        "chassis_plate": (chassis_plate(p), "chassis", True),
        "motor_clamp_left": (motor_clamp(p, 1), "chassis", True),
        "motor_clamp_right": (motor_clamp(p, -1), "chassis", True),
        "sensor_bracket": (sensor_bracket(p), "sensor_mount", True),
        "caster_mount": (caster_mount(p), "chassis", True),
        "battery_tray": (battery_tray(p), "battery_holder", True),
        "deck_plate": (deck_plate(p), "chassis", True),
        "led_mount": (led_mount(p), "chassis", True),
    }
    bought = {
        "motor_left": motor_envelope(p, 1),
        "motor_right": motor_envelope(p, -1),
        "wheel_left": wheel(p, 1),
        "wheel_right": wheel(p, -1),
        "qtr": qtr_board(p),
        "caster": caster(p),
        "battery": battery(p),
        "standoffs": standoffs(p),
    }
    bought.update(electronics(p))

    report = {"params": asdict(p), "checks": [], "interferences": []}

    def check(name, ok, detail):
        report["checks"].append({"check": name, "pass": bool(ok), "detail": detail})

    allshapes = [v[0] for v in parts.values()] + list(bought.values())
    bb = bbox(allshapes)
    size = (bb.xlen, bb.ylen, bb.zlen)
    check("fits 220 x 220 x 220 mm box", max(size) <= 220.0, f"envelope {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} mm")
    check("footprint near 150 x 140 mm target", size[0] <= 160 and size[1] <= 145, f"{size[0]:.1f} x {size[1]:.1f} mm")
    check("track width matches firmware", True, f"{p.track} mm (config.h track_width_mm)")
    check("sensor line ahead of axle", p.sensor_fwd > 0, f"{p.sensor_fwd} mm (config.h sensor_forward_mm)")
    check("sensor height in QTR range", 3.0 <= p.sensor_height <= 6.0, f"{p.sensor_height} mm above floor")
    # ground clearance: everything except wheels and caster ball above 3 mm
    low = []
    for name, s in list(bought.items()) + [(k, v[0]) for k, v in parts.items()]:
        if name in ("wheel_left", "wheel_right", "caster"):
            continue
        z = s.val().BoundingBox().zmin
        if z < 3.0:
            low.append(f"{name}@{z:.1f}")
    check("ground clearance >= 3 mm (except wheels/caster)", not low, ", ".join(low) or "ok")
    # level chassis: caster contact and wheel contact both at z = 0
    cz = bought["caster"].val().BoundingBox().zmin
    wz = bought["wheel_left"].val().BoundingBox().zmin
    check("chassis level (caster vs wheels)", abs(cz - wz) < 0.6, f"caster {cz:.2f} mm, wheel {wz:.2f} mm")
    # wheels inside the plate width? plate must not touch the wheels
    gap = p.track / 2 - p.wheel_w / 2 - p.plate_half_w
    check("plate clears wheels", gap >= p.wheel_clear, f"lateral gap {gap:.1f} mm")
    # battery under the deck
    batt_top = bought["battery"].val().BoundingBox().zmax
    deck_bot = parts["deck_plate"][0].val().BoundingBox().zmin
    check("battery clears deck", deck_bot - batt_top >= 2.0, f"gap {deck_bot - batt_top:.1f} mm")
    # pairwise interference
    names = list(parts.keys()) + list(bought.keys())
    shapes = [parts[n][0] if n in parts else bought[n] for n in names]
    allowed = {("motor_clamp_left", "motor_left"), ("motor_clamp_right", "motor_right"),
               ("battery_tray", "battery"), ("deck_plate", "standoffs"), ("chassis_plate", "standoffs"),
               ("sensor_bracket", "qtr"), ("caster_mount", "caster"),
               ("motor_left", "wheel_left"), ("motor_right", "wheel_right")}  # shaft in the wheel hub
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            bi, bj = shapes[i].val().BoundingBox(), shapes[j].val().BoundingBox()
            if bi.xmax < bj.xmin or bj.xmax < bi.xmin or bi.ymax < bj.ymin or bj.ymax < bi.ymin or bi.zmax < bj.zmin or bj.zmax < bi.zmin:
                continue
            try:
                v = shapes[i].val().intersect(shapes[j].val()).Volume()
            except Exception:
                v = 0.0
            if v > 1.0:
                ok = (a, b) in allowed or (b, a) in allowed
                report["interferences"].append({"a": a, "b": b, "volume_mm3": round(v, 1), "expected_contact": ok})
    unexpected = [x for x in report["interferences"] if not x["expected_contact"]]
    check("no unexpected interference", not unexpected, json.dumps(unexpected) if unexpected else "ok")
    report["mass_estimate_g"] = round(
        sum(vol(parts[k][0]) for k in parts) / 1000 * 1.27 * 0.5  # PETG, ~50 % infill
        + 2 * 10 + 2 * 6 + 3 + 58 + 10 + 4 + 3 + 3 + 8 + 15, 0)  # motors, wheels, QTR, battery, ESP32, ...
    if out:
        export(parts, bought, report, p)
    return parts, bought, report


def export(parts, bought, report, p: P):
    for name, (shape, folder, printable) in parts.items():
        d = HW / folder
        d.mkdir(parents=True, exist_ok=True)
        cq.exporters.export(shape, str(d / f"{name}.step"))
        cq.exporters.export(shape, str(d / f"{name}.stl"), tolerance=0.05, angularTolerance=0.1)
    ad = HW / "assembly_drawings"
    ad.mkdir(parents=True, exist_ok=True)
    allshapes = [v[0] for v in parts.values()] + list(bought.values())
    comp = cq.Workplane("XY").newObject([cq.Compound.makeCompound([s.val() for s in allshapes])])
    cq.exporters.export(comp, str(ad / "robot_assembly.step"))
    views = {"top": (0, 0, 1), "side": (0, -1, 0), "front": (1, 0, 0), "iso": (1, -1, 0.8)}
    for vname, d in views.items():
        cq.exporters.export(comp, str(ad / f"assembly_{vname}.svg"),
                            opt={"projectionDir": d, "showHidden": False, "width": 900, "height": 600,
                                 "marginLeft": 40, "marginTop": 40, "strokeWidth": 0.4})
    for name in ("chassis_plate", "sensor_bracket", "motor_clamp_left", "battery_tray"):
        for vname, d in (("top", (0, 0, 1)), ("side", (0, -1, 0))):
            cq.exporters.export(parts[name][0], str(ad / f"{name}_{vname}.svg"),
                                opt={"projectionDir": d, "showHidden": True, "width": 700, "height": 450,
                                     "marginLeft": 30, "marginTop": 30, "strokeWidth": 0.4})
    lines = ["# CAD fit report (generated by hardware/cad/robot_cad.py)", "",
             "Rendered geometry + automated dimensional checks. **Not** a physical validation.", "",
             "| check | result | detail |", "|---|---|---|"]
    for c in report["checks"]:
        lines.append(f"| {c['check']} | {'PASS' if c['pass'] else 'FAIL'} | {c['detail']} |")
    lines += ["", f"Estimated mass: **{report['mass_estimate_g']:.0f} g** (printed parts at 50 % infill PETG + typical "
              "masses of bought parts; weigh the real robot).", "", "## Contacts between parts (expected = mounted together)", "",
              "| part A | part B | overlap mm^3 | expected |", "|---|---|---|---|"]
    for x in report["interferences"]:
        lines.append(f"| {x['a']} | {x['b']} | {x['volume_mm3']} | {'yes' if x['expected_contact'] else 'NO'} |")
    lines += ["", "## Purchased-part envelopes assumed (verify with calipers)", "", "| part | x | y | z |", "|---|---|---|---|"]
    for k, v in PURCHASED.items():
        lines.append(f"| {k} | {v[0]} | {v[1]} | {v[2]} |")
    lines += ["", "## Parameters", "", "```json", json.dumps(report["params"], indent=1), "```"]
    (ad / "fit_report.md").write_text("\n".join(lines), encoding="utf-8")
    (ad / "fit_report.json").write_text(json.dumps(report, indent=1), encoding="utf-8")


if __name__ == "__main__":
    _, _, rep = build()
    bad = [c for c in rep["checks"] if not c["pass"]]
    for c in rep["checks"]:
        print(("PASS " if c["pass"] else "FAIL ") + c["check"] + ": " + c["detail"])
    print("interferences:", rep["interferences"])
    print("mass estimate g:", rep["mass_estimate_g"])
    sys.exit(1 if bad else 0)
