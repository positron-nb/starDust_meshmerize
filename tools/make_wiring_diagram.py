"""Draw the circuit diagram of the PURCHASED hardware.

Outputs
    hardware/wiring_diagram.svg / .png   page 1 only (schematic)
    docs/Circuit_Diagram.pdf             3 pages: schematic, connection list, charging + safety

The GPIO numbers are read from hardware/pin_mapping.csv (which
simulator/tests/test_consistency.py checks against firmware pins.h), so the
drawing cannot silently disagree with the firmware.

    python tools/make_wiring_diagram.py
"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "hardware" / "pin_mapping.csv"
SVG = ROOT / "hardware" / "wiring_diagram.svg"
PNG = ROOT / "hardware" / "wiring_diagram.png"
PDF = ROOT / "docs" / "Circuit_Diagram.pdf"

W, H = 200.0, 142.0  # drawing units (A3 landscape: 1 unit ~ 2.1 mm)
C = {
    "vbat": "#D62728", "5v": "#FF7F0E", "3v3": "#1F77B4", "gnd": "#222222",
    "ir": "#9467BD", "motor": "#2CA02C", "i2c": "#17BECF", "led": "#E377C2", "opt": "#888888",
}

# 30-pin ESP32 DevKit V1, antenna at the top, USB at the bottom (top -> bottom)
ESP_LEFT = ["EN", "VP 36", "VN 39", "D34", "D35", "D32", "D33", "D25", "D26", "D27", "D14", "D12", "D13", "GND", "VIN"]
ESP_RIGHT = ["D23", "D22", "TX0 1", "RX0 3", "D21", "D19", "D18", "D5", "TX2 17", "RX2 16", "D4", "D2", "D15", "GND", "3V3"]
ESP_X0, ESP_X1, ESP_TOP, ESP_STEP = 85.0, 112.0, 112.0, 5.0


def gpio_table() -> dict[int, dict]:
    rows = csv.DictReader(CSV.open(encoding="utf-8"))
    return {int(r["gpio"]): r for r in rows if r["gpio"].isdigit()}


def t_sym(t: dict[int, dict], sym: str) -> int:
    """GPIO number of a firmware symbol, from the pin table."""
    for g, r in t.items():
        if r["firmware_symbol"].strip() == sym:
            return g
    raise KeyError(sym)


def esp_pin(gpio_or_name) -> tuple[float, float]:
    """(x, y) of an ESP32 header pin by GPIO number or label (first match)."""
    for side, x in ((ESP_LEFT, ESP_X0), (ESP_RIGHT, ESP_X1)):
        for i, lab in enumerate(side):
            last = lab.split()[-1]
            num = last[1:] if last.startswith("D") else last
            if (isinstance(gpio_or_name, int) and num.isdigit() and int(num) == gpio_or_name) or lab == gpio_or_name:
                return x, ESP_TOP - i * ESP_STEP
    raise KeyError(gpio_or_name)


class Sheet:
    def __init__(self, title: str):
        self.fig = plt.figure(figsize=(16.5, 11.7))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, W)
        self.ax.set_ylim(0, H)
        self.ax.set_aspect("equal")
        self.ax.axis("off")
        self.fig.patch.set_facecolor("white")
        self.ax.text(4, H - 4, title, fontsize=17, weight="bold", va="top")

    def text(self, x, y, s, **kw):
        kw.setdefault("fontsize", 8)
        kw.setdefault("va", "center")
        self.ax.text(x, y, s, **kw)

    def box(self, x, y, w, h, title, sub="", color="#F4F4F4", edge="#333"):
        self.ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=1.2",
                                         fc=color, ec=edge, lw=1.6, zorder=2))
        if title:
            self.text(x + w / 2, y + h + (4.6 if sub else 2.2), title, ha="center", fontsize=9.5, weight="bold")
        if sub:
            self.text(x + w / 2, y + h + 1.9, sub, ha="center", fontsize=7, color="#555")

    def pin(self, x, y, label, side, color="#333"):
        """Pin circle on a box edge; the label goes inside the box."""
        self.ax.add_patch(Circle((x, y), 0.7, fc="white", ec=color, lw=1.2, zorder=4))
        off = 1.6 if side == "left" else -1.6
        self.text(x + off, y, label, ha="left" if side == "left" else "right", fontsize=7.2, zorder=4)

    def wire(self, pts, color, lw=1.8, ls="-"):
        xs, ys = zip(*pts)
        self.ax.plot(xs, ys, color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=3)

    def dot(self, x, y, color):
        self.ax.add_patch(Circle((x, y), 0.8, fc=color, ec=color, zorder=5))

    def tag(self, x, y, net, side="right"):
        """Net label: every tag with the same name is the same wire."""
        color = {"3V3": C["3v3"], "5V": C["5v"], "VBAT": C["vbat"]}[net]
        w = 1.9 * len(net) + 2.2
        x0 = x + 1.0 if side == "right" else x - 1.0 - w
        self.wire([(x, y), (x0 if side == "right" else x0 + w, y)], color, lw=1.4)
        self.ax.add_patch(FancyBboxPatch((x0, y - 1.3), w, 2.6, boxstyle="round,pad=0,rounding_size=0.6",
                                         fc=color, ec=color, zorder=4))
        self.text(x0 + w / 2, y, net, ha="center", fontsize=6.8, color="white", weight="bold", zorder=5)

    def gnd(self, x, y):
        """Ground symbol hanging below (x, y)."""
        self.wire([(x, y), (x, y - 2.0)], C["gnd"], lw=1.4)
        for i, half in enumerate((1.8, 1.2, 0.6)):
            yy = y - 2.0 - i * 0.6
            self.ax.plot([x - half, x + half], [yy, yy], color=C["gnd"], lw=1.3, zorder=3)

    def gnd_side(self, x, y, side):
        dx = 3.0 if side == "right" else -3.0
        self.wire([(x, y), (x + dx, y)], C["gnd"], lw=1.4)
        self.gnd(x + dx, y)


# ----------------------------------------------------------------- page 1 --
def schematic(t: dict[int, dict]) -> Sheet:
    s = Sheet("Meshmerize robot - circuit diagram")
    s.text(4, H - 10, "ESP32 DevKit V1 (30-pin) | 5-ch digital IR array + 2 x FC-51 | L298N + 2 x BO 500 RPM | MPU6050",
           fontsize=8.6, color="#444")
    s.text(4, H - 13.6, "Power: 4S 18650 pack -> 3 A fuse -> rocker switch -> L298N 12V + LM2596 (set to 5.0 V)",
           fontsize=8.6, color="#444")
    s.text(4, H - 17.2, "Blue 3V3 tags are all one wire. Every ground symbol is ONE common ground. "
           "Dashed / grey = optional.", fontsize=8, color="#444")
    lg = [("VBAT 12.8-16.8 V", C["vbat"]), ("5 V", C["5v"]), ("3.3 V", C["3v3"]), ("GND", C["gnd"]),
          ("IR sensor signal", C["ir"]), ("motor control", C["motor"]), ("I2C (MPU6050)", C["i2c"]), ("LED", C["led"])]
    s.text(170, 42.5, "Wire colours", fontsize=8, weight="bold")
    for i, (name, col) in enumerate(lg):
        y = 38.5 - i * 3.4
        s.wire([(170, y), (175, y)], col, lw=2.6)
        s.text(176.2, y, name, fontsize=7.2)

    # --- ESP32
    n = len(ESP_LEFT)
    bot = ESP_TOP - (n - 1) * ESP_STEP - 4
    s.box(ESP_X0, bot, ESP_X1 - ESP_X0, ESP_TOP + 4 - bot, "ESP32 DevKit V1", "30-pin, antenna end up")
    s.ax.add_patch(Rectangle((ESP_X0 + 9.5, bot - 2.5), 8, 3, fc="#BBB", ec="#333", zorder=2))
    s.text((ESP_X0 + ESP_X1) / 2, bot - 4.5, "micro-USB", ha="center", fontsize=6.5, color="#555")
    s.text((ESP_X0 + ESP_X1) / 2, ESP_TOP - 33, "BOOT button\n= START\n(GPIO0)", ha="center", fontsize=6.6, color="#555")
    s.text((ESP_X0 + ESP_X1) / 2, ESP_TOP - 50, "on-board LED\n= status\n(GPIO2)", ha="center", fontsize=6.6, color="#555")
    for i, lab in enumerate(ESP_LEFT):
        s.pin(ESP_X0, ESP_TOP - i * ESP_STEP, lab, "left")
    for i, lab in enumerate(ESP_RIGHT):
        s.pin(ESP_X1, ESP_TOP - i * ESP_STEP, lab, "right")
    for lab, note in (("D26", "opt. VBAT"), ("D27", "n.c."), ("D12", "n.c."), ("D13", "n.c.")):
        _, y = esp_pin(lab)
        s.text(72, y, note, ha="right", fontsize=6.3, color=C["opt"])
    for lab, note in (("TX0 1", "USB"), ("RX0 3", "USB"), ("TX2 17", "n.c."), ("RX2 16", "n.c.")):
        x, y = esp_pin(lab)
        s.text(x + 9.5, y, note, ha="left", fontsize=6.3, color=C["opt"])

    # --- 5-channel IR array (left)
    ax0, ax1 = 16.0, 44.0
    s.box(ax0, 84, ax1 - ax0, 27, "5-channel IR array", "digital D1-D5, VCC = 3V3 (NOT 5 V)")
    s.pin(ax0, 104, "VCC", "left")
    s.pin(ax0, 96, "GND", "left")
    s.tag(ax0, 104, "3V3", side="left")
    s.gnd_side(ax0, 96, "left")
    s.text(ax0 + 12, 89.5, "D1 = robot's LEFT side", ha="center", fontsize=6.5, color="#555")
    for k in range(5):
        x, y = esp_pin(t_sym(t, f"pins::kSensor[{k}]"))
        s.pin(ax1, y, f"D{k + 1}", "right", C["ir"])
        s.wire([(ax1, y), (x, y)], C["ir"])

    # --- FC-51 wings: each output gets its own vertical (x=58 left wing, x=64 right wing)
    for k, (y0, name, vx) in enumerate(((68.0, "LEFT wing FC-51", 58.0), (52.0, "RIGHT wing FC-51", 64.0))):
        ex, ey = esp_pin(t_sym(t, f"pins::kSensor[{5 + k}]"))
        s.box(ax0, y0, 24, 9, name, "")
        s.pin(ax0, y0 + 6.5, "VCC", "left")
        s.pin(ax0, y0 + 2.5, "GND", "left")
        s.tag(ax0, y0 + 6.5, "3V3", side="left")
        s.gnd_side(ax0, y0 + 2.5, "left")
        oy = y0 + 4.5
        s.pin(ax0 + 24, oy, "OUT", "right", C["ir"])
        s.wire([(ax0 + 24, oy), (vx, oy), (vx, ey), (ex, ey)], C["ir"])
    s.text(ax0 + 12, 48.5, "both point DOWN, 5-10 mm above the floor,\n~45 mm left / right of the array centre",
           ha="center", fontsize=6.5, color="#555", va="top")

    # ESP32 power pins
    xg, yg = esp_pin("GND")
    s.wire([(xg, yg), (xg - 3.5, yg)], C["gnd"], lw=1.4)
    s.gnd(xg - 3.5, yg)
    x3, y3 = ESP_X1, ESP_TOP - 14 * ESP_STEP
    s.wire([(x3, y3), (x3 + 4, y3)], C["3v3"], lw=1.4)
    s.tag(x3 + 4, y3, "3V3")
    s.text(x3 + 1, y3 - 3.6, "3V3 feeds: IR array, 2 x FC-51, MPU6050", fontsize=6, color=C["3v3"])

    # --- MPU6050 (top right)
    mx = 140.0
    s.box(mx, 112, 22, 16, "MPU6050 (GY-521)", "VCC = 3V3")
    s.pin(mx, 125.5, "VCC", "left")
    s.pin(mx, 122, "GND", "left")
    s.pin(mx, 118.5, "SCL", "left", C["i2c"])
    s.pin(mx, 115, "SDA", "left", C["i2c"])
    s.text(mx + 15, 117, "AD0, INT,\nXDA, XCL:\nn.c.", ha="center", fontsize=6, color="#555")
    s.tag(mx, 125.5, "3V3", side="left")
    s.wire([(mx, 122), (mx - 2.5, 122)], C["gnd"], lw=1.4)
    s.gnd(mx - 2.5, 122)
    xs, ys = esp_pin(t_sym(t, "pins::kScl"))
    s.wire([(xs, ys), (116, ys), (116, 118.5), (mx, 118.5)], C["i2c"])
    xd, yd = esp_pin(t_sym(t, "pins::kSda"))
    s.wire([(xd, yd), (118, yd), (118, 115), (mx, 115)], C["i2c"])

    # --- L298N
    lx0, lx1, ly0, ly1 = 140.0, 168.0, 50.0, 103.0
    s.box(lx0, ly0, lx1 - lx0, ly1 - ly0, "L298N motor driver", "red board with heatsink")
    # (name, firmware symbol, pin y, column of the vertical wire segment)
    logic = [("ENA", "pins::kPwmA", 99.5, 131.0), ("IN1", "pins::kAin1", 94.5, 120.0),
             ("IN2", "pins::kAin2", 89.5, 123.0), ("IN3", "pins::kBin1", 84.5, 126.0),
             ("IN4", "pins::kBin2", 79.5, 129.0), ("ENB", "pins::kPwmB", 74.5, 137.0)]
    for name, sym, y, vx in logic:
        s.pin(lx0, y, name, "left", C["motor"])
        gx, gy = esp_pin(t_sym(t, sym))
        if gx == ESP_X0:  # GPIO14 is on the left header: route under the ESP32
            s.wire([(gx, gy), (74, gy), (74, 32), (vx, 32), (vx, y), (lx0, y)], C["motor"])
        else:
            s.wire([(gx, gy), (vx, gy), (vx, y), (lx0, y)], C["motor"])
    s.text(152.5, 67, "REMOVE the ENA and ENB\njumper caps (the ESP32\ndrives them with PWM)", ha="center",
           fontsize=6.6, color="#B00")
    s.text(152.5, 58, "REMOVE the 5V-EN jumper\n(battery is above 12 V)", ha="center", fontsize=6.6, color="#B00")
    for (o1, o2, y1, y2, name) in (("OUT1", "OUT2", 97, 91, "LEFT motor"), ("OUT3", "OUT4", 70, 64, "RIGHT motor")):
        s.pin(lx1, y1, o1, "right", C["motor"])
        s.pin(lx1, y2, o2, "right", C["motor"])
        cy = (y1 + y2) / 2
        s.wire([(lx1, y1), (177, y1), (177, cy + 4.4)], "#555")
        s.wire([(lx1, y2), (177, y2), (177, cy - 4.4)], "#555")
        s.ax.add_patch(Circle((177, cy), 4.4, fc="#FFF8E1", ec="#333", lw=1.5, zorder=4))
        s.text(177, cy, "M", ha="center", fontsize=12, weight="bold", zorder=5)
        s.text(183, cy + 2.2, name, fontsize=8, weight="bold")
        s.text(183, cy - 0.8, "BO 500 RPM", fontsize=6.6, color="#555")
        s.text(183, cy - 3.6, "100 nF across\nterminals (opt.)", fontsize=5.8, color=C["opt"], va="top")
    terms = {"12V": 146.0, "GND": 154.0, "5V": 162.0}
    for name, x in terms.items():
        s.ax.add_patch(Rectangle((x - 2.2, ly0 - 1.2), 4.4, 2.4, fc="#2E7D32", ec="#333", zorder=4))
        s.text(x, ly0 + 2.8, name, ha="center", fontsize=7, weight="bold", zorder=5)

    # --- red LED (end zone)
    xl, yl = esp_pin(t_sym(t, "pins::kLedRed"))
    lx = 128.0
    s.wire([(xl, yl), (lx, yl), (lx, 49)], C["led"])
    s.ax.fill([lx - 2.2, lx + 2.2, lx], [49, 49, 45.6], color="#E53935", zorder=4)
    s.ax.plot([lx - 2.4, lx + 2.4], [45.5, 45.5], color="#333", lw=1.4, zorder=4)
    s.wire([(lx, 45.5), (lx, 43.5)], C["gnd"], lw=1.4)
    s.gnd(lx, 43.5)
    s.text(125.3, 49.2, "RED LED", ha="right", fontsize=6.6, weight="bold")
    s.text(125.3, 46.9, "long leg (+) to D15", ha="right", fontsize=6)
    s.text(125.3, 44.8, "short leg (-) to GND", ha="right", fontsize=6)

    # --- power section (bottom)
    yb = 22.0
    s.box(4, 5, 28, 13, "Battery: 4 x 18650 in SERIES (4S)", "14.8 V nominal, 16.8 V full", color="#FFF3C4")
    for i in range(4):
        s.ax.add_patch(Rectangle((6.5 + i * 6.2, 8.5), 5.2, 5, fc="#90CAF9", ec="#333", zorder=3))
        s.text(9.1 + i * 6.2, 11, "3.7V", ha="center", fontsize=5.5, zorder=4)
    s.pin(32, 15, "+", "right", C["vbat"])
    s.pin(32, 8, "-", "right", C["gnd"])
    s.wire([(32, 15), (37, 15), (37, yb), (40, yb)], C["vbat"], lw=2.6)
    s.wire([(32, 8), (36, 8)], C["gnd"], lw=2.0)
    s.gnd(36, 8)
    s.ax.add_patch(Rectangle((40, yb - 1.6), 9, 3.2, fc="#EEE", ec="#333", lw=1.4, zorder=4))
    s.ax.plot([40, 49], [yb, yb], color="#555", lw=1.0, zorder=5)
    s.text(44.5, yb + 3.5, "FUSE 3 A", ha="center", fontsize=7.5, weight="bold")
    s.text(44.5, yb - 3.6, "glass, twist holder", ha="center", fontsize=6, color="#555")
    s.wire([(49, yb), (54, yb)], C["vbat"], lw=2.6)
    s.dot(54, yb, C["vbat"])
    s.ax.plot([54, 60], [yb, yb + 3], color="#333", lw=2.0, zorder=5)
    s.dot(61, yb, C["vbat"])
    s.text(57.5, yb + 5.6, "ROCKER SWITCH", ha="center", fontsize=7.5, weight="bold")
    s.text(57.5, yb - 3.6, "KCD1 (2 pins used)", ha="center", fontsize=6, color="#555")
    s.wire([(61, yb), (terms["12V"], yb), (terms["12V"], ly0 - 1.2)], C["vbat"], lw=2.6)
    s.text(122, yb + 1.8, "VBAT (fused, switched) 12.8-16.8 V", ha="center", fontsize=7, color=C["vbat"],
           weight="bold")
    s.dot(72, yb, C["vbat"])
    bx0, bx1, by0, by1 = 66.0, 92.0, 2.0, 14.0
    s.box(bx0, by0, bx1 - bx0, by1 - by0, "", "")
    s.text((bx0 + bx1) / 2, by0 + 7.8, "LM2596 buck", ha="center", fontsize=8.5, weight="bold", zorder=5)
    s.text((bx0 + bx1) / 2, by0 + 3.8, "set the pot to 5.0 V\nBEFORE connecting the ESP32", ha="center",
           fontsize=6, color="#B00", zorder=5)
    s.ax.add_patch(Circle((72, by1), 0.7, fc="white", ec=C["vbat"], lw=1.2, zorder=4))
    s.text(73.6, by1 - 1.7, "IN+", fontsize=7.2, zorder=4)
    s.wire([(72, yb), (72, by1)], C["vbat"], lw=2.2)
    s.pin(bx0, 5, "IN-", "left")
    s.wire([(bx0, 5), (bx0 - 3, 5)], C["gnd"], lw=1.4)
    s.gnd(bx0 - 3, 5)
    s.pin(bx1, 11, "OUT+", "right", C["5v"])
    s.pin(bx1, 5, "OUT-", "right")
    s.wire([(bx1, 5), (bx1 + 3, 5)], C["gnd"], lw=1.4)
    s.gnd(bx1 + 3, 5)
    xv, yv = esp_pin("VIN")
    s.wire([(bx1, 11), (terms["5V"], 11), (terms["5V"], ly0 - 1.2)], C["5v"], lw=2.2)
    s.dot(100, 11, C["5v"])
    s.wire([(100, 11), (100, 27), (78, 27), (78, yv), (xv, yv)], C["5v"], lw=2.2)
    s.text(120, 13.2, "5 V  ->  ESP32 VIN and L298N 5V terminal", fontsize=7, color=C["5v"], weight="bold")
    s.wire([(terms["GND"], ly0 - 1.2), (terms["GND"], 30)], C["gnd"], lw=2.0)
    s.gnd(terms["GND"], 30)
    # bulk capacitor (optional), between the 12V and GND terminal wires
    yc = 38.0
    s.wire([(terms["12V"], yc), (149.4, yc)], C["opt"], lw=1.0, ls="--")
    s.wire([(150.6, yc), (terms["GND"], yc)], C["opt"], lw=1.0, ls="--")
    s.ax.plot([149.4, 149.4], [yc - 1.6, yc + 1.6], color=C["opt"], lw=1.6)
    s.ax.plot([150.6, 150.6], [yc - 1.6, yc + 1.6], color=C["opt"], lw=1.6)
    s.text(155.6, yc, "470 uF\n35 V opt.", fontsize=6, color=C["opt"])
    return s


# ----------------------------------------------------------------- page 2 --
CONNECTIONS = [
    ("POWER", None),
    ("Battery +  (4S pack, red)", "Fuse (3 A), one end"),
    ("Fuse, other end", "Rocker switch, pin 1"),
    ("Rocker switch pin 2  (= VBAT)", "L298N 12V terminal  AND  LM2596 IN+"),
    ("Battery -  (black)", "L298N GND terminal  AND  LM2596 IN-"),
    ("LM2596 OUT+  (set to 5.0 V first!)", "ESP32 VIN  AND  L298N 5V terminal"),
    ("LM2596 OUT-", "ESP32 GND"),
    ("ESP32 3V3", "IR array VCC, left FC-51 VCC, right FC-51 VCC, MPU6050 VCC"),
    ("ESP32 GND", "IR array GND, both FC-51 GND, MPU6050 GND, LED short leg"),
    ("SENSORS", None),
    ("IR array D1 (robot's left)", "ESP32 VP (GPIO36)"),
    ("IR array D2", "ESP32 VN (GPIO39)"),
    ("IR array D3 (centre)", "ESP32 D34"),
    ("IR array D4", "ESP32 D35"),
    ("IR array D5 (robot's right)", "ESP32 D32"),
    ("Left FC-51 OUT", "ESP32 D33"),
    ("Right FC-51 OUT", "ESP32 D25"),
    ("MPU6050 SCL", "ESP32 D22"),
    ("MPU6050 SDA", "ESP32 D21"),
    ("MOTORS", None),
    ("ESP32 D23", "L298N ENA  (jumper cap removed)"),
    ("ESP32 D19", "L298N IN1"),
    ("ESP32 D18", "L298N IN2"),
    ("ESP32 D5", "L298N IN3"),
    ("ESP32 D4", "L298N IN4"),
    ("ESP32 D14", "L298N ENB  (jumper cap removed)"),
    ("L298N OUT1 / OUT2", "LEFT BO motor"),
    ("L298N OUT3 / OUT4", "RIGHT BO motor"),
    ("OPERATOR", None),
    ("ESP32 D15", "Red LED long leg (+);  short leg (-) to GND"),
    ("ESP32 BOOT button (on board)", "START button - nothing to wire"),
]

SETUP = [
    "L298N jumpers: REMOVE the ENA cap, the ENB cap and the 5V-EN cap (the on-board 5 V",
    "   regulator is only safe up to 12 V; ours gets 16.8 V, so 5 V comes from the LM2596).",
    "LM2596: connect ONLY the battery first, switch ON, turn the brass pot until OUT reads",
    "   5.0 V on a multimeter (anticlockwise usually lowers it). Then connect the ESP32.",
    "All sensors on 3V3, never 5 V: their outputs then stay <= 3.3 V (ESP32 pins are NOT",
    "   5 V tolerant). FC-51 and the IR array work at 3.3 V; re-adjust their blue pots.",
    "IR pots: the module LED should switch cleanly between the black floor and the white",
    "   line (either way round - the firmware learns each channel's polarity in calibration).",
    "FC-51: it is an obstacle sensor; mounted 5-10 mm above the floor pointing DOWN it",
    "   works as a 0/1 line sensor. Keep its IR LEDs shielded from sunlight.",
    "Firmware: flash the 'esp32_l298n' environment (1 kHz PWM suits the L298N).",
    "USB: switch the battery OFF while the USB cable is plugged in for flashing.",
    "A motor turns the wrong way? Swap that motor's two wires on the L298N.",
    "Common ground: every GND (battery -, LM2596 IN-/OUT-, L298N GND, ESP32, sensors)",
    "   must be connected, otherwise nothing reads correctly.",
    "First power-up: wheels OFF the floor. Run the calibration sweep, then check the",
    "   serial log: 'CAL ok', the gyro reading, each IR channel toggling over the line.",
]


def connection_page() -> Sheet:
    s = Sheet("Connection list (wire by wire) and setup")
    y = H - 12
    s.text(4, y, "FROM", fontsize=9, weight="bold")
    s.text(52, y, "TO", fontsize=9, weight="bold")
    y -= 4
    for a, b in CONNECTIONS:
        if b is None:
            y -= 1
            s.ax.add_patch(Rectangle((3, y - 1.6), 108, 3.2, fc="#E8EEF7", ec="none"))
            s.text(4, y, a, fontsize=8.5, weight="bold", color="#1F3B70")
        else:
            s.text(4, y, a, fontsize=8)
            s.text(48, y, "->", fontsize=8, color="#999")
            s.text(52, y, b, fontsize=8)
        y -= 3.55
    x = 118
    y = H - 12
    s.text(x, y, "SETUP - do these before the first power-up", fontsize=9.5, weight="bold")
    y -= 4.5
    for line in SETUP:
        bullet = not line.startswith("   ")
        s.text(x + (0 if bullet else 2), y, ("- " if bullet else "") + line.strip(), fontsize=7.4)
        y -= 3.1 if bullet else 3.6
    y -= 3
    s.text(x, y, "GPIO notes", fontsize=9.5, weight="bold")
    for line in ("VP/VN/D34/D35 are input-only: fine for sensors.",
                 "D12 must stay free (boot strap). D16/D17 are kept for",
                 "   optional slot encoders, D26 for optional battery sense.",
                 "D14 and D5 wiggle for a moment at boot: a tiny motor",
                 "   twitch at power-on is normal."):
        y -= 3.4
        s.text(x + (2 if line.startswith("   ") else 0), y, line.strip(), fontsize=7.4)
    return s


# ----------------------------------------------------------------- page 3 --
def charging_page() -> Sheet:
    s = Sheet("Charging, battery safety and three cheap upgrades")
    blocks = [
        ("IMPORTANT: the TP4056 cannot charge the 4S pack", "#B00", [
            "The TP4056 charges ONE cell to 4.2 V. The pack is 4 cells in series (16.8 V full).",
            "Never connect the TP4056 B+/B- or OUT+/OUT- to the series pack or to the robot circuit.",
            "Its OUT+/OUT- pins are not used anywhere in this robot.",
        ]),
        ("Option A (recommended, roughly Rs 450-700): charge inside the robot", "#1F3B70", [
            "Buy a 4S BMS board (e.g. 4S 20-40 A, with balancing) and a 16.8 V 1-2 A Li-ion charger adapter.",
            "Pack (B-, B1, B2, B3, B+) -> BMS; BMS P+/P- -> fuse/switch input; charger -> DC jack -> P+/P-.",
            "The BMS also cuts off on over-discharge, which protects the cells while we have no voltage sensing.",
        ]),
        ("Option B (with what you already have): one cell at a time", "#1F3B70", [
            "Take the 4 cells out of the holder. Put ONE cell in a single 18650 holder wired to TP4056 B+/B-.",
            "Charge until the TP4056 LED changes colour (full = 4.2 V). Repeat for all 4 cells.",
            "All 4 cells then start equal (this also balances them). Slow (2-3 h per cell) but safe.",
        ]),
        ("Battery rules", "#1F3B70", [
            "CHARGE FULLY before every match: without a battery-sense divider the firmware assumes a full",
            "pack (16.2 V). In simulation a pack voltage the firmware did not know made the no-encoder speed",
            "model ~20 % wrong and the harsh-condition actual runs failed; with the right voltage they finished.",
            "Stop using the pack below ~12.8 V (3.2 V per cell): check it with a multimeter between runs.",
            "Insert cells with the correct polarity; a reversed cell in a series holder can overheat.",
        ]),
        ("Three cheap upgrades (each is 1-2 parts; resistors have no polarity)", "#2E7D32", [
            "1. 330 ohm resistor in series with the red LED (D15 -> 330R -> LED + ; LED - -> GND).",
            "   Without it the firmware sets the weakest pin drive, but a resistor is the proper way.",
            "2. Battery sense: VBAT -> 47 k -> D26 -> 10 k -> GND  (16.8 V becomes 2.95 V at D26).",
            "   Then build with -DMESH_VBAT_SENSE=1: the robot measures the pack, keeps its speeds right",
            "   as the battery drains, and refuses to start on a flat pack.",
            "3. 470 uF / 35 V electrolytic across L298N 12V-GND (striped (-) leg to GND): absorbs motor spikes.",
        ]),
    ]
    y = H - 13
    for title, col, lines in blocks:
        s.text(4, y, title, fontsize=11, weight="bold", color=col)
        y -= 4.6
        for line in lines:
            s.text(6, y, line, fontsize=8.6)
            y -= 3.5
        y -= 3.5
    return s


def main():
    t = gpio_table()
    PDF.parent.mkdir(parents=True, exist_ok=True)
    sheet1 = schematic(t)
    sheet1.fig.savefig(SVG)
    sheet1.fig.savefig(PNG, dpi=110)
    with PdfPages(PDF) as pdf:
        pdf.savefig(sheet1.fig)
        pdf.savefig(connection_page().fig)
        pdf.savefig(charging_page().fig)
    plt.close("all")
    for p in (SVG, PNG, PDF):
        print("wrote", p.relative_to(ROOT))


if __name__ == "__main__":
    main()
