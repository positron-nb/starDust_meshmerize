"""Explanatory diagrams for the illustrated guide (docs/Meshmerize_Robot_Guide.pdf).

All figures are drawn from the project's real parameters (pin map, geometry,
thresholds) so the guide stays consistent with firmware and CAD.

  python tools/guide/figures.py      # writes PNGs to tools/guide/_fig/
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "_fig"
OUT.mkdir(exist_ok=True)

C = {
    "blue": "#3B6FB6", "lblue": "#DCE8F7", "orange": "#E07B24", "lorange": "#FBE6D3", "green": "#3A9A4A",
    "lgreen": "#DFF2E1", "red": "#C8352E", "lred": "#F8DEDC", "purple": "#7E57A8", "lpurple": "#EBE3F5",
    "grey": "#6B6B6B", "lgrey": "#EFEFEF", "yellow": "#F2C230", "lyellow": "#FFF5CC", "black": "#222222",
}
plt.rcParams.update({"font.size": 10, "font.family": "DejaVu Sans"})


def canvas(w, h, xmax=100, ymax=None):
    fig, ax = plt.subplots(figsize=(w, h))
    ymax = ymax if ymax is not None else xmax * h / w
    ax.set_xlim(0, xmax)
    ax.set_ylim(0, ymax)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def box(ax, x, y, w, h, text, fc="lblue", ec="blue", fs=10, bold=False, r=1.2, tc=None, ha="center"):
    p = FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=C.get(fc, fc), ec=C.get(ec, ec), lw=1.4)
    ax.add_patch(p)
    tx = x + w / 2 if ha == "center" else x + 1.2
    ax.text(tx, y + h / 2, text, ha=ha, va="center", fontsize=fs, fontweight="bold" if bold else "normal",
            color=C.get(tc, tc) if tc else C["black"], wrap=True)
    return (x, y, w, h)


def arrow(ax, p0, p1, color="grey", lw=1.6, text=None, fs=8.5, off=(0, 1.2), style="-|>", ls="-", rad=0.0):
    a = FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=13, color=C.get(color, color), lw=lw, ls=ls,
                        connectionstyle=f"arc3,rad={rad}")
    ax.add_patch(a)
    if text:
        ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], text, ha="center", va="center",
                fontsize=fs, color=C["black"], bbox=dict(fc="white", ec="none", pad=0.6))


def save(fig, name):
    fig.savefig(OUT / f"{name}.png", dpi=200, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


# ------------------------------------------------------------------ figures --
def big_picture():
    fig, ax = canvas(11, 3.6, 110)
    steps = [
        ("1  CALIBRATE", "1 minute allowed\nrobot sweeps over START line\nlearns black / white", "lyellow", "yellow"),
        ("2  DRY RUN", "<= 3 min\nexplore junctions, build a MAP\nLED ON in the white box", "lblue", "blue"),
        ("3  PLAN", "inside the robot\nDijkstra -> shortest ROUTE\nsaved to flash memory", "lpurple", "purple"),
        ("4  ACTUAL RUN", "<= 2 min 30 s\nreplay ROUTE fast from START\nLED ON in the white box", "lgreen", "green"),
    ]
    x = 2
    for i, (t, d, fc, ec) in enumerate(steps):
        box(ax, x, 10, 23, 22, "", fc, ec)
        ax.text(x + 11.5, 28, t, ha="center", fontsize=12, fontweight="bold", color=C[ec])
        ax.text(x + 11.5, 18, d, ha="center", va="center", fontsize=8.3)
        if i < 3:
            arrow(ax, (x + 23.3, 21), (x + 27.2, 21), lw=2.2)
        x += 27.5
    ax.text(55, 4.5, "Points:  A = 25 x checkpoints   B = 30   C = 180 - dry time   |   S = 30   T = 150 - actual time   (S, T only on the SHORTEST path)   L = 5 for the LED",
            ha="center", fontsize=8.8, color=C["grey"])
    save(fig, "big_picture")


def block_diagram():
    fig, ax = canvas(11, 6.4, 110)
    box(ax, 42, 22, 26, 22, "ESP32\nDevKit V1\n(the brain)\n\n1000 times per second:\nread -> decide -> drive", "lblue", "blue", fs=10, bold=False)
    ins = [("5-ch IR array\n(digital D1-D5)", 52, "5 x black/white"),
           ("2 x FC-51 wing\nsensors (left, right)", 41, "side branches"),
           ("MPU6050 gyro\n(I2C)", 30, "turn rate"),
           ("START = BOOT\nbutton", 19, "press")]
    for t, y, lab in ins:
        box(ax, 2, y - 4, 22, 9, t, "lyellow", "yellow", fs=8.8)
        arrow(ax, (24.3, y + 0.5), (41.7, max(23.5, min(43, y + 0.5))), text=lab, fs=8)
    outs = [("L298N\nmotor driver", 46, "PWM + direction"), ("Red LED\n(end zone)", 31, "ON in box"),
            ("Status LED\n(on board)", 21, "stage / fault")]
    for t, y, lab in outs:
        box(ax, 86, y - 4, 20, 9, t, "lgreen", "green", fs=8.8)
        arrow(ax, (68.3, max(23.5, min(43, y + 0.5))), (85.7, y + 0.5), text=lab, fs=8)
    box(ax, 86, 55, 9.5, 7, "Left\nBO", "lgrey", "grey", fs=8)
    box(ax, 96.5, 55, 9.5, 7, "Right\nBO", "lgrey", "grey", fs=8)
    arrow(ax, (92, 51.3), (90.5, 54.7))
    arrow(ax, (100, 51.3), (101, 54.7))
    box(ax, 22, 2, 66, 10, "POWER:  4 x 18650 (16.8 V) -> 3 A fuse -> switch\n-> L298N (motors)   and   -> LM2596 5 V -> ESP32 -> 3.3 V for sensors + gyro",
        "lred", "red", fs=8.6)
    ax.text(55, 66, "SYSTEM BLOCK DIAGRAM", ha="center", fontsize=13, fontweight="bold")
    save(fig, "block_diagram")


def power_tree():
    fig, ax = canvas(11, 5.2, 110)
    chain = [("4 x 18650\nin series\n16.8 V full", "lred", "red"), ("3 A fuse\n(glass)", "lgrey", "grey"),
             ("Rocker\nswitch", "lgrey", "grey")]
    x = 2
    for t, fc, ec in chain:
        box(ax, x, 40, 15, 12, t, fc, ec, fs=8.6)
        if x > 2:
            arrow(ax, (x - 4.7, 46), (x - 0.3, 46), color="red", lw=2)
        x += 19.5
    ax.text(x - 2, 46, "VBAT", fontsize=10, fontweight="bold", color=C["red"], va="center")
    arrow(ax, (x - 4.7, 46), (x - 2.5, 46), color="red", lw=2)
    ax.plot([x + 6, x + 6, 48, 48], [43.5, 37.8, 37.8, 9], color=C["red"], lw=2)
    targets = [("L298N 12V terminal (+470 uF, optional)\n-> left and right BO motors", 26, "lgreen", "green"),
               ("LM2596 buck -> 5.0 V\n-> ESP32 VIN + L298N 5V (logic)", 15, "lblue", "blue"),
               ("47k / 10k divider -> GPIO26\n(optional battery sense)", 4, "lyellow", "yellow")]
    for t, y, fc, ec in targets:
        box(ax, 52, y, 34, 10, t, fc, ec, fs=8.4)
        arrow(ax, (48, y + 5), (51.7, y + 5), color="red", lw=2)
    box(ax, 90, 12, 18, 16, "ESP32 on-board\n3.3 V regulator\n->  IR array,\nFC-51s, MPU6050", "lpurple", "purple", fs=8)
    arrow(ax, (86.3, 20), (89.7, 20), color="purple", lw=2)
    ax.text(2, 31, "Why this way?", fontsize=10, fontweight="bold")
    ax.text(2, 4.5, "- Highest voltage on the robot: 16.8 V (rule limit 24 V)\n"
                    "- BO motors are a 6 V class part: firmware limits the\n  AVERAGE motor voltage to 6 V by PWM\n"
                    "- Sensors run at 3.3 V so their signal can never\n  damage the ESP32 pins\n"
                    "- L298N 5V-EN jumper OFF: its regulator is only for <= 12 V",
            fontsize=8.6, va="bottom")
    ax.text(55, 57, "POWER TREE", ha="center", fontsize=13, fontweight="bold")
    save(fig, "power_tree")


def wiring():
    """The full circuit diagram (same drawing as docs/Circuit_Diagram.pdf page 1)."""
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    import make_wiring_diagram as mwd
    sheet = mwd.schematic(mwd.gpio_table())
    sheet.fig.savefig(OUT / "wiring.png", dpi=170, bbox_inches="tight", pad_inches=0.05)
    plt.close(sheet.fig)


def robot_top():
    fig, ax = canvas(10.5, 7.2, 260)
    ox, oy = 95, 88  # axle centre on the drawing

    def R(x, y, w, h, fc, ec, label=None, fs=8):
        ax.add_patch(Rectangle((ox + x, oy + y), w, h, fc=C.get(fc, fc), ec=C.get(ec, ec), lw=1.3))
        if label:
            ax.text(ox + x + w / 2, oy + y + h / 2, label, ha="center", va="center", fontsize=fs)
    R(-75, -50, 140, 100, "lgrey", "grey")                    # kit plate (measure yours)
    R(-32.5, 54, 65, 26, "#444", "#222")                      # wheels 65 mm
    R(-32.5, -80, 65, 26, "#444", "#222")
    for side in (1, -1):                                      # BO motors inside the wheels
        ax.add_patch(Rectangle((ox - 35, oy + (30 if side > 0 else -52)), 65, 22, fc="none", ec=C["grey"], ls="--", lw=1.2))
    R(-38, -40, 76, 80, "lyellow", "yellow", "4 x 18650\nholder\n(strapped)", 8)
    R(-62, -12, 20, 16, "lblue", "blue", "MPU\n6050", 7)
    R(65, -8, 30, 16, "lorange", "orange")                    # bracket
    R(92, -36, 14, 72, "lpurple", "purple")                   # array board
    for i in range(5):
        ax.add_patch(Circle((ox + 100, oy + 30 - i * 15), 3.2, fc=C["yellow"], ec="#333", lw=0.8))
    for y in (45, -45):
        ax.add_patch(Rectangle((ox + 94, oy + y - 6), 12, 12, fc=C["lgreen"], ec=C["green"]))
        ax.add_patch(Circle((ox + 100, oy + y), 3.2, fc=C["yellow"], ec="#333", lw=0.8))
    ax.add_patch(Circle((ox - 62, oy + 28), 7, fc="#bbb", ec="#555"))
    ax.plot([ox, ox], [oy - 85, oy + 85], color=C["blue"], ls="-.", lw=1)
    ax.plot([ox - 80, ox + 115], [oy, oy], color="#999", ls=":", lw=0.8)

    def lab(x, y, tx, ty, t):
        ax.annotate(t, xy=(ox + x, oy + y), xytext=(ox + tx, oy + ty), fontsize=8.4,
                    arrowprops=dict(arrowstyle="-", color="#444", lw=0.8))
    lab(20, 74, 40, 92, "left wheel (65 mm)")
    lab(-20, -46, -95, -82, "BO motor (under the plate)")
    lab(100, 30, 125, 62, "5 array sensors,\n15 mm apart")
    lab(100, 45, 128, 85, "LEFT wing FC-51 (45 mm out)")
    lab(100, -45, 125, -70, "RIGHT wing FC-51")
    lab(-62, 28, -98, 62, "caster")
    lab(-52, -6, -110, -40, "gyro near\nthe middle")
    ax.annotate("", xy=(ox + 100, oy - 92), xytext=(ox, oy - 92), arrowprops=dict(arrowstyle="<->", color=C["red"]))
    ax.text(ox + 50, oy - 99, "100 mm: sensor line ahead of the axle", ha="center", fontsize=8.6, color=C["red"])
    ax.annotate("", xy=(ox + 50, oy + 67), xytext=(ox + 50, oy - 67), arrowprops=dict(arrowstyle="<->", color=C["red"]))
    ax.text(ox + 53, oy + 25, "track\n~135 mm", ha="left", fontsize=8.6, color=C["red"])
    ax.annotate("FORWARD", xy=(ox + 150, oy), xytext=(ox + 116, oy), fontsize=9, fontweight="bold", color=C["blue"],
                va="center", arrowprops=dict(arrowstyle="-|>", color=C["blue"], lw=1.8))
    ax.text(130, 186, "TOP VIEW (purchased 2WD chassis; measure your own)", ha="center", fontsize=12.5, fontweight="bold")
    ax.text(130, -2, "Everything, including the sensor bracket, must fit in a 220 mm cube", ha="center", fontsize=9, color=C["grey"])
    ax.set_ylim(-8, 192)
    save(fig, "robot_top")


def robot_layers():
    fig, ax = canvas(10.5, 5.6, 200)
    ax.plot([5, 195], [8, 8], color="#333", lw=2)
    ax.text(100, 3, "floor / arena", ha="center", fontsize=8.5, color=C["grey"])
    ax.add_patch(Circle((80, 40.5), 32.5, fc="#555", ec="#222"))
    ax.add_patch(Rectangle((60, 34), 40, 13, fc=C["lgrey"], ec=C["grey"]))
    ax.text(116, 30, "wheel Ø65 mm with the\nBO motor behind it", fontsize=8)
    ax.add_patch(Rectangle((20, 8), 12, 30, fc=C["lgrey"], ec=C["grey"]))
    ax.add_patch(Circle((26, 14), 6, fc="#bbb", ec="#555"))
    ax.text(26, 3, "caster", ha="center", fontsize=8)
    ax.add_patch(Rectangle((15, 47), 150, 3.5, fc=C["blue"], ec=C["blue"]))
    ax.text(18, 53, "chassis plate (kit)", fontsize=8.5, color=C["blue"])
    ax.add_patch(Polygon([(150, 47), (186, 47), (186, 22), (182, 22), (182, 43.5), (150, 43.5)], closed=True, fc=C["lorange"], ec=C["orange"]))
    ax.add_patch(Rectangle((170, 16), 22, 5, fc=C["lpurple"], ec=C["purple"]))
    ax.text(160, 24, "IR array +\nFC-51s", fontsize=7.5)
    ax.text(150, 11, "5-10 mm", fontsize=7.5, color=C["red"])
    ax.plot([80, 80], [74, 93], color=C["red"], ls=":", lw=0.8)
    ax.plot([181, 181], [22, 93], color=C["red"], ls=":", lw=0.8)
    ax.annotate("", xy=(181, 93), xytext=(80, 93), arrowprops=dict(arrowstyle="<->", color=C["red"]))
    ax.text(150, 95.8, "100 mm: axle to sensor line", ha="center", fontsize=8, color=C["red"])
    ax.add_patch(Rectangle((50, 50.5), 78, 21, fc=C["lyellow"], ec=C["yellow"]))
    ax.text(89, 61, "4 x 18650 (strapped)", ha="center", fontsize=8.5)
    ax.add_patch(Rectangle((48, 71.5), 82, 2.5, fc=C["blue"], ec=C["blue"]))
    ax.add_patch(Rectangle((52, 74), 34, 10, fc="#333", ec="#111"))
    ax.text(69, 79, "ESP32", ha="center", color="white", fontsize=8)
    ax.add_patch(Rectangle((90, 74), 22, 14, fc=C["lred"], ec=C["red"]))
    ax.text(101, 81, "L298N", ha="center", fontsize=7.5)
    ax.add_patch(Rectangle((114, 74), 14, 6, fc=C["lpurple"], ec=C["purple"]))
    ax.text(121, 85, "LM2596", ha="center", fontsize=7)
    ax.add_patch(Circle((40, 80), 2.6, fc=C["red"], ec="#600"))
    ax.text(30, 86, "red LED", fontsize=8, color=C["red"])
    ax.text(85, 103, "SIDE VIEW: what sits on top of what (example layout)", ha="center", fontsize=12.5, fontweight="bold")
    ax.annotate("FORWARD", xy=(195, 103), xytext=(165, 103), fontsize=9, fontweight="bold", color=C["blue"], va="center",
                arrowprops=dict(arrowstyle="-|>", color=C["blue"], lw=1.8))
    save(fig, "robot_layers")


def sensor_patterns():
    # array sensors at -30..30 mm (15 mm pitch), wings at -45 / +45 mm (index 5 = left, 6 = right)
    xs = [-30, -15, 0, 15, 30, -45, 45]
    pats = [
        ("Straight line", [(0, -50, 0, 50)], [1, 2, 3], "follow it (PD control)"),
        ("Branch on the LEFT", [(0, -50, 0, 50), (-55, 0, 0, 0)], [0, 1, 2, 3, 5], "left wing sees white\n-> branch on the left"),
        ("Branch on the RIGHT", [(0, -50, 0, 50), (0, 0, 55, 0)], [1, 2, 3, 4, 6], "right wing sees white"),
        ("Cross line / T", [(0, -50, 0, 0), (-55, 0, 55, 0)], list(range(7)), "all 5 + both wings white\n(then: is there a straight?)"),
        ("Dead end", [(0, -50, 0, -14)], [], "nothing seen while the line\nwas centred -> U-turn"),
        ("END BOX (400 mm)", [], list(range(7)), "all white for 70 mm\n-> LED ON"),
    ]
    fig, axs = plt.subplots(1, 6, figsize=(12, 3.3))
    for ax, (title, lines, lit, note) in zip(axs, pats):
        ax.set_xlim(-55, 55)
        ax.set_ylim(-55, 55)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.add_patch(Rectangle((-55, -55), 110, 110, fc="#111", ec="#888", lw=1))
        if title.startswith("END"):
            ax.add_patch(Rectangle((-55, -20), 110, 75, fc="white", ec="none"))
            ax.add_patch(Rectangle((-15, -55), 30, 35, fc="white"))
        for x1, y1, x2, y2 in lines:
            if x1 == x2:
                ax.add_patch(Rectangle((x1 - 15, min(y1, y2)), 30, abs(y2 - y1), fc="white"))
            else:
                ax.add_patch(Rectangle((min(x1, x2), y1 - 15), abs(x2 - x1), 30, fc="white"))
        for i, x in enumerate(xs):
            on = i in lit
            wing = i >= 5
            ax.add_patch(Circle((x, 0), 4.2 if wing else 3.6, fc=C["yellow"] if on else "#555",
                                ec=C["green"] if wing else (C["orange"] if on else "#888"), lw=1.6 if wing else 1))
        ax.set_title(title, fontsize=9.5, fontweight="bold")
        ax.text(0, -64, note, ha="center", va="top", fontsize=8)
    fig.suptitle("What the 5 array sensors + 2 wing sensors (green ring) see (yellow = white).  The robot moves UP the page.",
                 fontsize=11, fontweight="bold", y=0.98)
    fig.subplots_adjust(top=0.8, bottom=0.18, wspace=0.12)
    save(fig, "sensor_patterns")


def junction_timing():
    fig, ax = canvas(11, 5.4, 220)
    ax.add_patch(Rectangle((5, 38), 210, 10, fc="white", ec="#999"))
    ax.add_patch(Rectangle((170, 16), 10, 62, fc="white", ec="#999"))
    ax.text(184, 74, "cross line\n(30 mm wide)", fontsize=8.5)
    ax.add_patch(FancyBboxPatch((55, 28), 112, 30, boxstyle="round,pad=0,rounding_size=4", fc=C["lblue"], ec=C["blue"], alpha=0.6))
    ax.add_patch(Circle((70, 43), 3, fc=C["blue"]))
    ax.text(70, 35, "axle", ha="center", fontsize=8)
    ax.add_patch(Rectangle((168, 30), 4, 26, fc=C["purple"]))
    ax.text(150, 61, "sensors", fontsize=8, color=C["purple"])
    ax.annotate("", xy=(170, 22), xytext=(70, 22), arrowprops=dict(arrowstyle="<->", color=C["red"]))
    ax.text(120, 17, "100 mm", ha="center", color=C["red"], fontsize=9)
    ax.plot([175, 175], [16, 78], color=C["green"], ls="--")
    ax.text(10, 100, "1. Sensors pass over the cross line -> record where it started (s0) and ended (s1); steering holds the gyro heading.", fontsize=8.6)
    ax.text(10, 93, "2. 16 mm later: is there still a line straight ahead?  -> exits = left / straight / right.", fontsize=8.6)
    ax.text(10, 2, "3. Junction centre is under the AXLE at  s = (s0+s1)/2 + 100 mm.  The robot brakes to stop there, then pivots\n"
                   "    90 deg (gyro angle) until the sensors find the new line (centred within ~9 mm = 0.6 x sensor pitch).", fontsize=8.6)
    ax.annotate("", xy=(175, 43), xytext=(72, 43), arrowprops=dict(arrowstyle="-|>", color=C["green"], lw=1.6, ls="--"))
    ax.text(186, 50, "the axle will\nstop here", fontsize=8.5, color=C["green"])
    save(fig, "junction_timing")


def control_loop():
    fig, ax = canvas(11, 3.9, 120)
    box(ax, 1, 24, 17, 10, "5 sensors + 2 wings\n-> line position\n(mm left/right)", "lyellow", "yellow", fs=7.6)
    box(ax, 23, 24, 17, 10, "PD steering\nturn = Kp x error\n+ Kd x error rate", "lblue", "blue", fs=7.6)
    box(ax, 23, 6, 17, 10, "speed profile\n(accelerate, cruise,\nbrake before turns)", "lblue", "blue", fs=7.6)
    box(ax, 45, 15, 15, 10, "mix\nleft = v - turn\nright = v + turn", "lpurple", "purple", fs=7.8)
    box(ax, 65, 15, 17, 10, "speed model +\nlearned L/R trim\n(<= 6 V average)", "lgreen", "green", fs=7.6)
    box(ax, 87, 15, 13, 10, "L298N\n-> BO\nmotors", "lgreen", "green", fs=8)
    box(ax, 104, 15, 14, 10, "MPU6050\ngyro\n(heading)", "lorange", "orange", fs=8)
    arrow(ax, (18.3, 29), (22.7, 29))
    arrow(ax, (40.3, 29), (44.7, 22))
    arrow(ax, (40.3, 11), (44.7, 18))
    arrow(ax, (60.3, 20), (64.7, 20))
    arrow(ax, (82.3, 20), (86.7, 20))
    arrow(ax, (100.3, 20), (103.7, 20))
    arrow(ax, (111, 14.7), (31, 5.7), color="orange", rad=-0.25, text="turn angle, heading hold", off=(0, -5.5))
    ax.text(60, 40, "CONTROL LOOP - runs 1000 times every second (no wheel encoders)", ha="center", fontsize=12, fontweight="bold")
    save(fig, "control_loop")


def state_machine():
    fig, ax = canvas(11, 7.8, 120)
    B = {}

    def S(name, x, y, fc, ec, w=19, h=7):
        B[name] = (x, y, w, h)
        box(ax, x, y, w, h, name, fc, ec, fs=8.2, bold=True)

    def A(a, b, t=None, rad=0.0, off=(0, 1.3)):
        xa, ya, wa, ha = B[a]
        xb, yb, wb, hb = B[b]
        pa = (xa + wa / 2, ya + ha / 2)
        pb = (xb + wb / 2, yb + hb / 2)
        # start/end on box borders
        def border(x, y, w, h, tx, ty):
            dx, dy = tx - (x + w / 2), ty - (y + h / 2)
            k = min(abs((w / 2) / dx) if dx else 1e9, abs((h / 2) / dy) if dy else 1e9)
            return (x + w / 2 + dx * k * 1.04, y + h / 2 + dy * k * 1.04)
        arrow(ax, border(xa, ya, wa, ha, *pb), border(xb, yb, wb, hb, *pa), text=t, fs=7.3, rad=rad, off=off)

    S("IDLE", 4, 46, "lgrey", "grey")
    S("CALIBRATING", 4, 26, "lyellow", "yellow")
    S("ARMED", 32, 46, "lgrey", "grey")
    S("DRY RUN", 60, 56, "lblue", "blue")
    S("ACTUAL RUN", 60, 36, "lgreen", "green")
    S("GOAL CONFIRM", 92, 46, "lred", "red", w=22)
    S("PLAN ROUTE", 92, 66, "lpurple", "purple", w=22)
    S("WAIT FOR\nACTUAL", 60, 72, "lpurple", "purple", h=8)
    S("FINISHED", 92, 26, "lgreen", "green", w=22)
    S("RECOVERY", 32, 16, "lorange", "orange")
    S("FAULT", 60, 6, "lred", "red")
    A("IDLE", "CALIBRATING", "hold 1.5-4 s")
    A("CALIBRATING", "IDLE", None, rad=0.4)
    A("IDLE", "ARMED", "short press")
    A("ARMED", "DRY RUN", "no route yet")
    A("ARMED", "ACTUAL RUN", "route stored")
    A("DRY RUN", "GOAL CONFIRM", "70 mm white", off=(2, 2))
    A("ACTUAL RUN", "GOAL CONFIRM", "70 mm white", off=(2, -2))
    A("GOAL CONFIRM", "PLAN ROUTE", "dry")
    A("PLAN ROUTE", "WAIT FOR\nACTUAL", "saved")
    A("WAIT FOR\nACTUAL", "ARMED", "press", rad=0.25)
    A("GOAL CONFIRM", "FINISHED", "actual")
    A("DRY RUN", "RECOVERY", "line lost", rad=0.2, off=(-4, 0))
    A("RECOVERY", "ACTUAL RUN", "found", rad=-0.2)
    A("RECOVERY", "FAULT", "3 fails")
    A("ACTUAL RUN", "FAULT", "stall /\nbutton", off=(5, 0))
    A("FAULT", "ARMED", "press = restart", rad=-0.35, off=(-8, 0))
    ax.text(60, 84, "WHAT THE ROBOT IS DOING (state machine)", ha="center", fontsize=12.5, fontweight="bold")
    ax.text(4, 0.5, "LED red = in the end box.   The stage (no map / map interrupted / route stored) is saved in flash, so the robot knows by itself which run comes next.",
            fontsize=8, color=C["grey"])
    save(fig, "state_machine")


def exploration_rule():
    fig, ax = canvas(11, 4.6, 120)
    # toy graph
    nodes = {"S": (8, 10), "A": (8, 28), "B": (30, 28), "C": (30, 46), "D": (52, 28), "E": (8, 46)}
    edges = [("S", "A", True), ("A", "B", True), ("B", "C", True), ("B", "D", False), ("A", "E", False), ("C", "D", False)]
    for a, b, known in edges:
        (x1, y1), (x2, y2) = nodes[a], nodes[b]
        ax.plot([x1, x2], [y1, y2], color=C["blue"] if known else "#bbb", lw=3 if known else 2, ls="-" if known else "--")
    for k, (x, y) in nodes.items():
        col = C["green"] if k == "S" else (C["orange"] if k in ("A", "B") else "#999")
        ax.add_patch(Circle((x, y), 2.6, fc=col, ec="#333"))
        ax.text(x, y, k, ha="center", va="center", color="white", fontsize=9, fontweight="bold")
    ax.add_patch(Polygon([(30, 46 + 3.5), (28, 46 + 7.5), (32, 46 + 7.5)], fc=C["red"]))
    ax.text(34, 51, "robot here", fontsize=8, color=C["red"])
    ax.text(2, 54, "blue = mapped,  grey dashed = not yet explored", fontsize=8)
    ax.text(62, 52, "At every junction the robot asks:", fontsize=10, fontweight="bold")
    ax.text(62, 47, "Which junction with an UNEXPLORED line is best?", fontsize=9)
    ax.text(62, 40, "cost = (distance from the robot, + 140 mm per turn)\n        + 2 x (distance from START)", fontsize=9.2,
            family="monospace", bbox=dict(fc=C["lyellow"], ec=C["yellow"]))
    ax.text(62, 31, "- If the best one is right here: take its unexplored line\n   (order: Straight, then Left, then Right).", fontsize=8.8)
    ax.text(62, 24, "- Otherwise drive there over KNOWN lines (faster, 0.7 m/s).", fontsize=8.8)
    ax.text(62, 17, "- The '2 x distance from START' makes it finish branches\n   near START first, so the first route it finds to the box\n   is almost always the SHORTEST (97-99 % in simulation).", fontsize=8.8)
    ax.text(62, 7, "- Each junction is stored with its position (motor speed model\n   + gyro). Coming back to a known junction = 'loop closure'.", fontsize=8.8)
    save(fig, "exploration_rule")


def goal_detection():
    fig, ax = plt.subplots(figsize=(11, 3.6))
    x = list(range(0, 760))
    def white(s):
        if 100 <= s < 133:
            return 1
        if 300 <= s < 336:
            return 1
        if s >= 480:
            return 1
        return 0
    run = []
    c = 0
    for s in x:
        c = c + 1 if white(s) else 0
        run.append(c)
    ax.plot(x, run, color=C["blue"], lw=2, label="length of continuous 'all white' (mm)")
    ax.axhline(70, color=C["red"], ls="--", lw=1.4)
    ax.text(5, 76, "70 mm threshold", color=C["red"], fontsize=9)
    ax.axvspan(100, 133, color=C["lyellow"])
    ax.axvspan(300, 336, color=C["lyellow"])
    ax.axvspan(480, 760, color="#eeeeee")
    ax.text(116, 140, "junction\n(30 mm)\nignored", ha="center", fontsize=8.5)
    ax.text(318, 140, "crossing\nignored", ha="center", fontsize=8.5)
    ax.text(620, 230, "END BOX (400 mm)", ha="center", fontsize=9.5, fontweight="bold")
    ax.axvline(550, color=C["red"], lw=1.4)
    ax.text(553, 20, "LED ON\n(70 mm into the box)", fontsize=8.5, color=C["red"])
    ax.axvspan(550, 690, color=C["lred"], alpha=0.6)
    ax.text(620, 100, "drive 140 mm more,\nthen STOP fully inside", ha="center", fontsize=8.5)
    ax.set_xlabel("distance travelled by the robot (mm)")
    ax.set_ylabel("white length (mm)")
    ax.set_ylim(0, 280)
    ax.set_xlim(0, 760)
    ax.set_title("End box vs. junction: decided by DISTANCE of continuous white, not by one reading", fontsize=11, fontweight="bold")
    ax.legend(loc="upper left", fontsize=8.5)
    save(fig, "goal_detection")


def relocalization():
    fig, axs = plt.subplots(1, 3, figsize=(11, 3.5))
    old = [((0, 0), (0, 3)), ((0, 3), (3, 3)), ((3, 3), (3, 6)), ((0, 3), (-3, 3)), ((3, 3), (6, 3))]
    for ax in axs:
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_xlim(-4.5, 9.5)
        ax.set_ylim(-1.5, 8.5)
    for a, b in old:
        axs[0].plot([a[0], b[0]], [a[1], b[1]], color=C["blue"], lw=3)
    axs[0].plot(3, 3, "o", color=C["green"], ms=14)
    axs[0].text(3.6, 4.2, "checkpoint\n(robot failed\nafter this)", fontsize=8)
    axs[0].set_title("1. Map before the restart", fontsize=10, fontweight="bold")
    loc = [((0, 0), (3, 0)), ((3, 0), (3, 3)), ((3, 0), (6, 0))]
    for a, b in loc:
        axs[1].plot([a[0], b[0]], [a[1], b[1]], color=C["orange"], lw=3)
    axs[1].plot(0, 0, "s", color=C["red"], ms=12)
    axs[1].text(-0.6, -1.45, "placed here, position unknown", fontsize=8)
    axs[1].set_title("2. New small map after restart", fontsize=10, fontweight="bold")
    for a, b in old:
        axs[2].plot([a[0], b[0]], [a[1], b[1]], color=C["blue"], lw=3, alpha=0.5)
    for a, b in [((3, 3), (6, 3)), ((6, 3), (6, 6)), ((6, 3), (9, 3))]:
        axs[2].plot([a[0], b[0]], [a[1], b[1]], color=C["orange"], lw=3)
    axs[2].plot(3, 3, "o", color=C["green"], ms=14)
    axs[2].set_title("3. Fitted on top -> one map again", fontsize=10, fontweight="bold")
    fig.suptitle("Restart from a checkpoint: the robot tries every old junction x 4 directions and keeps the one that fits",
                 fontsize=10.5, fontweight="bold", y=0.98)
    fig.subplots_adjust(top=0.84, bottom=0.02)
    save(fig, "relocalization")


def match_timeline():
    fig, ax = canvas(11, 2.8, 120)
    segs = [(2, 12, "calibration\n1 min", "lyellow", "yellow"), (14, 52, "DRY RUN: up to 3:00 (180 s)", "lblue", "blue"),
            (52, 54, "", "lgrey", "grey"), (54, 98, "ACTUAL RUN: up to 2:30 (150 s)", "lgreen", "green")]
    for a, b, t, fc, ec in segs:
        box(ax, a, 12, b - a, 9, t, fc, ec, fs=8.5)
    ax.text(53, 24, "carry robot\nback to START", ha="center", fontsize=7.5)
    ax.text(56, 4, "Total for both runs: 5:30.  If the dry run takes longer than 3:00, the extra time is cut from the actual run.  Max 3 restarts in the whole match; the clock never stops.",
            ha="center", fontsize=8)
    ax.text(102, 16.5, "simulated robot:\ndry ~60 s\nactual ~14 s", fontsize=8, va="center")
    save(fig, "match_timeline")


def results_charts():
    import csv as _csv
    import statistics
    rows = list(_csv.DictReader((ROOT / "results/policy_study.csv").open()))
    order = [("dfs_LSR", "Left first"), ("left_hand", "Left-hand rule"), ("dfs_SLR", "Straight first"),
             ("uc2_SLR", "Straight first +\nSTART weight 2\n(DEFAULT)"), ("defer_SLR", "Peek-and-prove\n(needs OK)")]
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.6))
    sh, sc = [], []
    for lab, _ in order:
        rr = [r for r in rows if r["label"] == lab]
        sh.append(100 * sum(r["shortest_path"] == "1" for r in rr) / len(rr))
        sc.append(statistics.mean(float(r["total"]) for r in rr))
    names = [n for _, n in order]
    cols = ["#999", "#999", C["blue"], C["green"], C["orange"]]
    axs[0].bar(names, sh, color=cols)
    axs[0].set_ylabel("% actual runs on the TRUE shortest path")
    axs[0].set_ylim(0, 105)
    for i, v in enumerate(sh):
        axs[0].text(i, v + 2, f"{v:.0f}%", ha="center", fontsize=9)
    axs[1].bar(names, sc, color=cols)
    axs[1].set_ylabel("average total score")
    axs[1].set_ylim(0, 470)
    for i, v in enumerate(sc):
        axs[1].text(i, v + 6, f"{v:.0f}", ha="center", fontsize=9)
    for a in axs:
        a.tick_params(axis="x", labelsize=7.6)
        a.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Which exploration rule scores best? (81 simulated arenas each, realistic noise, original N20 robot)", fontsize=11, fontweight="bold")
    fig.tight_layout()
    save(fig, "results_policy")

    b = list(_csv.DictReader((ROOT / "results/benchmark_results_n20_v1.0.csv").open()))  # study done on the N20 robot
    ls = [("Ls_40", 40), ("Ls_55", 55), ("final_real", 70), ("Ls_90", 90)]
    comp, score = [], []
    for lab, _ in ls:
        rr = [r for r in b if r["label"] == lab]
        comp.append(100 * sum(r["dry_completed"] == "1" for r in rr) / len(rr))
        score.append(statistics.mean(float(r["total"]) for r in rr))
    fig, ax = plt.subplots(figsize=(7, 3.0))
    xs = [v for _, v in ls]
    ax.plot(xs, comp, "o-", color=C["blue"], label="dry run completed (%)")
    ax.plot(xs, [s / 4.3 for s in score], "s--", color=C["green"], label="average score / 4.3")
    ax.axvline(70, color=C["red"], ls=":")
    ax.text(71, 30, "chosen: 70 mm", color=C["red"], fontsize=9)
    ax.set_xlabel("sensor distance ahead of the axle (mm)")
    ax.set_ylim(0, 110)
    ax.legend(fontsize=8.5, loc="lower right")
    ax.set_title("Too short (40 mm) = no room to brake before turning (N20 robot)", fontsize=10.5, fontweight="bold")
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    save(fig, "results_sensor_distance")


def results_bo():
    """Purchased-hardware benchmark: no encoders vs. optional slot encoders."""
    import csv as _csv
    import statistics
    b = list(_csv.DictReader((ROOT / "results/benchmark_results.csv").open()))
    sets = [("final_ideal", "no noise"), ("final_real", "realistic"), ("final_harsh", "harsh"),
            ("slot_encoders_real", "realistic +\nslot encoders"), ("slot_encoders_harsh", "harsh +\nslot encoders")]
    sets = [(k, n) for k, n in sets if any(r["label"] == k for r in b)]
    sh, sc = [], []
    for k, _ in sets:
        rr = [r for r in b if r["label"] == k]
        sh.append(100 * sum(r["shortest_path"] == "1" for r in rr) / len(rr))
        sc.append(statistics.mean(float(r["total"]) for r in rr))
    names = [n for _, n in sets]
    cols = [C["grey"], C["blue"], C["red"], C["green"], C["orange"]][:len(sets)]
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.4))
    axs[0].bar(names, sh, color=cols)
    axs[0].set_ylabel("% actual runs on the TRUE shortest path")
    axs[0].set_ylim(0, 105)
    for i, v in enumerate(sh):
        axs[0].text(i, v + 2, f"{v:.0f}%", ha="center", fontsize=9)
    axs[1].bar(names, sc, color=cols)
    axs[1].set_ylabel("average total score")
    axs[1].set_ylim(0, 470)
    for i, v in enumerate(sc):
        axs[1].text(i, v + 6, f"{v:.0f}", ha="center", fontsize=9)
    for a in axs:
        a.tick_params(axis="x", labelsize=8)
        a.spines[["top", "right"]].set_visible(False)
    fig.suptitle("Purchased robot in simulation (BO motors, no encoders; 100 random mazes + sample each)", fontsize=11, fontweight="bold")
    fig.tight_layout()
    save(fig, "results_bo")


ALL = [big_picture, block_diagram, power_tree, wiring, robot_top, robot_layers, sensor_patterns, junction_timing,
       control_loop, state_machine, exploration_rule, goal_detection, relocalization, match_timeline, results_charts,
       results_bo]

if __name__ == "__main__":
    for f in ALL:
        f()
        print("ok", f.__name__)
