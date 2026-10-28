"""Build the illustrated guide: docs/Meshmerize_Robot_Guide.pdf

  python tools/guide/figures.py       # (re)draw the diagrams
  python tools/guide/build_guide.py   # assemble the PDF

Plain-language explanation of how the robot works, how to build and wire it
and how a match runs. All numbers come from the project files (config.h,
pin_mapping.csv, bom.csv, CAD, results/).
"""
from __future__ import annotations

import csv
import datetime
import statistics
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, ListFlowable, ListItem, PageBreak, Paragraph, SimpleDocTemplate,
                                Spacer, Table, TableStyle)

ROOT = Path(__file__).resolve().parents[2]
FIG = Path(__file__).resolve().parent / "_fig"
IMG = ROOT / "docs" / "images"
OUT = ROOT / "docs" / "Meshmerize_Robot_Guide.pdf"

# Unicode-capable font (arrows, <=, degree, Ohm)
FONT, FONT_B = "Helvetica", "Helvetica-Bold"
for reg, bold in (("C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"),
                  ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")):
    if Path(reg).exists() and Path(bold).exists():
        pdfmetrics.registerFont(TTFont("Body", reg))
        pdfmetrics.registerFont(TTFont("Body-Bold", bold))
        pdfmetrics.registerFontFamily("Body", normal="Body", bold="Body-Bold", italic="Body", boldItalic="Body-Bold")
        FONT, FONT_B = "Body", "Body-Bold"
        break

BLUE = colors.HexColor("#3B6FB6")
LBLUE = colors.HexColor("#DCE8F7")
GREY = colors.HexColor("#6B6B6B")
LGREY = colors.HexColor("#F2F2F2")
RED = colors.HexColor("#C8352E")
LYELLOW = colors.HexColor("#FFF5CC")
GREEN = colors.HexColor("#3A9A4A")

ss = getSampleStyleSheet()
H1 = ParagraphStyle("H1", parent=ss["Heading1"], fontName=FONT_B, fontSize=17, textColor=BLUE, spaceBefore=2, spaceAfter=8)
H2 = ParagraphStyle("H2", parent=ss["Heading2"], fontName=FONT_B, fontSize=12.5, textColor=colors.black, spaceBefore=8, spaceAfter=4)
BODY = ParagraphStyle("Body", parent=ss["BodyText"], fontName=FONT, fontSize=10, leading=14, spaceAfter=5)
SMALL = ParagraphStyle("Small", parent=BODY, fontSize=8.5, leading=11, textColor=GREY)
CELL = ParagraphStyle("Cell", parent=BODY, fontSize=8.6, leading=11, spaceAfter=0)
CELLB = ParagraphStyle("CellB", parent=CELL, fontName=FONT_B)
NOTE = ParagraphStyle("Note", parent=BODY, backColor=LYELLOW, borderColor=colors.HexColor("#F2C230"), borderWidth=0.8,
                      borderPadding=6, spaceBefore=6, spaceAfter=10)
TITLE = ParagraphStyle("Title", parent=H1, fontSize=28, leading=34, alignment=TA_CENTER, spaceAfter=6)
SUB = ParagraphStyle("Sub", parent=BODY, fontSize=13, leading=17, alignment=TA_CENTER, textColor=GREY)

W = A4[0] - 30 * mm  # usable width


def P(t, st=BODY):
    return Paragraph(t, st)


def fig(path, width=W, max_h=None):
    ir = ImageReader(str(path))
    iw, ih = ir.getSize()
    h = width * ih / iw
    if max_h and h > max_h:
        width, h = max_h * iw / ih, max_h
    im = Image(str(path), width=width, height=h)
    im.hAlign = "CENTER"
    return im


def caption(t):
    return P(f"<i>{t}</i>", ParagraphStyle("cap", parent=SMALL, alignment=TA_CENTER, spaceAfter=8))


def bullets(items, st=BODY):
    return ListFlowable([ListItem(P(i, st), leftIndent=12, value="•") for i in items], bulletType="bullet",
                        start="•", leftIndent=12, bulletFontName=FONT)


def numbered(items, st=BODY):
    return ListFlowable([ListItem(P(i, st), leftIndent=20) for i in items], bulletType="1", leftIndent=20,
                        bulletFontName=FONT, bulletDedent=18)


def table(rows, widths, header=True, zebra=True, fs=None):
    data = [[P(str(c), CELLB if (header and r == 0) else CELL) for c in row] for r, row in enumerate(rows)]
    t = Table(data, colWidths=widths, repeatRows=1 if header else 0)
    style = [("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#BBBBBB")), ("VALIGN", (0, 0), (-1, -1), "TOP"),
             ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4),
             ("TOPPADDING", (0, 0), (-1, -1), 2.5), ("BOTTOMPADDING", (0, 0), (-1, -1), 2.5)]
    if header:
        style.append(("BACKGROUND", (0, 0), (-1, 0), LBLUE))
    if zebra:
        for r in range(1 if header else 0, len(rows)):
            if r % 2 == 0:
                style.append(("BACKGROUND", (0, r), (-1, r), LGREY))
    t.setStyle(TableStyle(style))
    return t


def page_deco(canvas, doc):
    canvas.saveState()
    canvas.setFont(FONT, 8)
    canvas.setFillColor(GREY)
    if doc.page > 1:
        canvas.drawString(15 * mm, A4[1] - 10 * mm, "Meshmerize robot - illustrated guide")
        canvas.drawRightString(A4[0] - 15 * mm, A4[1] - 10 * mm, "Techfest IIT Bombay 2026-27 · ESP32 version")
        canvas.setStrokeColor(colors.HexColor("#DDDDDD"))
        canvas.line(15 * mm, A4[1] - 11.5 * mm, A4[0] - 15 * mm, A4[1] - 11.5 * mm)
    canvas.drawCentredString(A4[0] / 2, 8 * mm, f"{doc.page}")
    canvas.restoreState()


# ---------------------------------------------------------------- content --
def bom_rows():
    rows = [["Part", "Exact model / spec", "Qty", "≈ ₹ each", "Price"]]
    for r in csv.DictReader((ROOT / "hardware/bom.csv").open(encoding="utf-8")):
        if not r["id"]:
            continue
        ps = r["price_status"]
        status = "owned" if ps.startswith("PURCHASED") else ("verified" if ps.startswith("VERIFIED") else "estimate")
        rows.append([r["item"], r["model_or_spec"], r["qty"], r["unit_price_inr"] or "-", status])
    return rows


def pin_rows():
    rows = [["GPIO", "Job", "Connect to"]]
    for r in csv.DictReader((ROOT / "hardware/pin_mapping.csv").open(encoding="utf-8")):
        if r["gpio"].isdigit() and r["firmware_symbol"]:
            rows.append([r["gpio"], r["function"], r["connected_to"]])
    return rows


def bench(label):
    rows = [r for r in csv.DictReader((ROOT / "results/benchmark_results.csv").open()) if r["label"] == label]
    n = len(rows)
    return {
        "n": n,
        "dry": 100 * sum(r["dry_completed"] == "1" for r in rows) / n,
        "act": 100 * sum(r["actual_completed"] == "1" for r in rows) / n,
        "short": 100 * sum(r["shortest_path"] == "1" for r in rows) / n,
        "score": statistics.mean(float(r["total"]) for r in rows),
        "dry_s": statistics.mean(float(r["dry_time_s"]) for r in rows if r["dry_completed"] == "1"),
        "act_s": statistics.mean(float(r["actual_time_s"]) for r in rows if r["actual_completed"] == "1"),
        "faults": sum(1 for r in rows if r["faults"]),
        "false": sum(int(r["false_goal_led"] or 0) for r in rows),
    }


def build():
    s = []
    today = datetime.date.today().strftime("%d %B %Y")

    # ------------------------------------------------------------- cover --
    s += [Spacer(1, 18 * mm), P("Meshmerize Robot", TITLE), P("Illustrated guide", SUB), Spacer(1, 3 * mm),
          P("How the robot works · how to build and wire it · how a match runs", SUB), Spacer(1, 8 * mm),
          fig(FIG / "robot_top.png", width=W * 0.85, max_h=100 * mm), Spacer(1, 6 * mm)]
    s.append(table([
        ["Brain", "ESP32 DevKit V1 (Wi-Fi and Bluetooth never switched on)"],
        ["Eyes", "5-channel digital IR array 100 mm in front of the wheels + 2 FC-51 'wing' sensors"],
        ["Balance", "MPU6050 gyro: turn angles and heading"],
        ["Legs", "2 × BO 500 RPM gear motors (no encoders), 65 mm wheels, 2WD chassis kit"],
        ["Muscles", "L298N motor driver"],
        ["Power", "4 × 18650 (16.8 V) → 3 A fuse → switch → L298N / LM2596 5 V → 3.3 V"],
        ["Size", "must fit a 220 mm cube including the sensor bracket (measure the built robot)"],
        ["Extra cost", "parts already bought; recommended additions about ₹900 (charger + BMS, a few resistors and capacitors)"],
    ], [32 * mm, W - 32 * mm], header=False))
    s += [Spacer(1, 5 * mm),
          P("<b>Honest status:</b> the robot has been designed, programmed and tested in a computer simulation that runs the "
            "real robot program, modelled on the parts the team bought. Nothing has been tested on real hardware yet. Every number marked "
            "'simulation' must be checked on the real robot.", NOTE),
          P(f"Generated {today} from the project files (firmware/include/config.h, hardware/pin_mapping.csv, hardware/bom.csv, "
            "CAD, results/).", SMALL), PageBreak()]

    # ------------------------------------------------------------ contents --
    s += [P("Contents", H1)]
    toc = ["The idea in one page", "Meet the robot (layout and parts)", "Parts list", "How the electronics fit together",
           "Power: from battery to every part", "Wiring: every wire, pin by pin", "Building it step by step",
           "How the robot sees the line", "How it drives straight and steers", "Junctions and 90° turns",
           "Dry run: exploring and making a map", "Actual run: the shortest route at speed", "Finding the end box and the red LED",
           "What the robot is doing (states, button, lights)", "Restarts", "Match day, minute by minute",
           "Calibration and tuning", "How well does it work? (simulation)", "Open questions and next steps"]
    s += [numbered(toc), PageBreak()]

    # ------------------------------------------------------------- 1 idea --
    s += [P("1. The idea in one page", H1),
          P("Meshmerize has two runs on the same white-line maze. In the <b>dry run</b> the robot does not know the maze: "
            "it follows the white lines, notes every junction and how far apart they are, and builds a <b>map</b>. When it "
            "drives into the 400 × 400 mm white end box it lights its red LED. It then works out the <b>shortest route</b> "
            "on its map and saves it. In the <b>actual run</b> it starts again from START and drives only that route, "
            "as fast as it safely can."),
          fig(FIG / "big_picture.png"), caption("The four steps of a match."),
          P("Where the points come from", H2),
          table([["Term", "Points", "What earns it"],
                 ["A", "25 per checkpoint", "crossing the (invisible) checkpoint spots during the dry run, each counted once"],
                 ["B", "30", "finishing the dry run"],
                 ["C", "180 − dry-run seconds", "a quick dry run"],
                 ["S", "30", "actual run along the SHORTEST path"],
                 ["T", "150 − actual-run seconds", "only if the actual run used the shortest path"],
                 ["L", "5", "the red LED glows in the end box"],
                 ["P", "minus penalties", "not defined in the rulebook (question for the organisers)"]],
                [14 * mm, 38 * mm, W - 52 * mm]),
          P("The big lesson: missing the shortest path costs about <b>170 points</b> (S + T), but an extra second of dry run "
            "costs only <b>1 point</b>. So the robot spends some extra time exploring to be sure its route is really the shortest.",
            NOTE), PageBreak()]

    # ------------------------------------------------------------ 2 robot --
    s += [P("2. Meet the robot", H1), fig(FIG / "robot_top.png", max_h=125 * mm),
          caption("Top view. The sensors are 100 mm in front of the wheel axle so the robot sees a junction before it reaches it. "
                  "The two wing sensors, 45 mm out on each side, see side lines the 60 mm wide array cannot."),
          fig(FIG / "robot_layers.png", max_h=95 * mm),
          caption("Side view (example layout): wheels and caster, chassis plate, battery holder, electronics on top."),
          PageBreak(),
          P("Measure these on your robot", H2),
          P("The robot program needs a few sizes of the real robot. Measure them with a ruler or calipers after building "
            "and enter them in <font face='Courier'>firmware/include/config.h</font>:"),
          table([["What", "Design value", "config.h name"],
                 ["Sensor line ahead of the wheel axle", "100 mm", "sensor_forward_mm"],
                 ["Distance between array sensors", "15 mm", "sensor_pitch_mm"],
                 ["Wing sensor from the robot's centre line", "45 mm", "wing_lateral_mm"],
                 ["Wheel centre to wheel centre", "135 mm", "track_width_mm"],
                 ["Wheel diameter", "65 mm", "wheel_diameter_mm"]],
                [75 * mm, 35 * mm, W - 110 * mm]),
          P("The original design (N20 motors with encoders, an 8-sensor Pololu array, a 3D-printed chassis) is kept in the "
            "project as a parametric CAD model and as a simulator preset, but the robot being built is the one in this guide.", SMALL),
          PageBreak()]

    # -------------------------------------------------------------- 3 BOM --
    s += [P("3. Parts list", H1),
          P("'Owned' = already bought. 'Verified' prices were checked on Indian shop pages in October 2026; 'estimate' means a "
            "typical price you should confirm. Items marked RECOMMENDED are cheap and make the robot safer or more accurate."),
          table(bom_rows(), [30 * mm, 86 * mm, 14 * mm, 18 * mm, 18 * mm]),
          Spacer(1, 3 * mm),
          P("<b>Recommended additions about ₹900</b> (4S BMS + 16.8 V charger, 330 Ω, 47 kΩ + 10 kΩ, 470 µF, 2 × 100 nF, wiring). "
            "Optional: slot encoders (about ₹140 for two).", BODY), PageBreak()]

    # ----------------------------------------------------- 4 block diagram --
    s += [P("4. How the electronics fit together", H1), fig(FIG / "block_diagram.png"),
          P("Think of it as a body:"),
          bullets(["<b>Eyes</b>: each of the 5 array sensors (and the 2 wing sensors) shines infrared light on the floor and says "
                   "'white' or 'black'. White vinyl reflects a lot, the black floor very little. A small pot on each sensor sets the switching point.",
                   "<b>Sense of direction</b>: the MPU6050 gyro measures how fast the robot turns, so the robot knows its turn angles and heading.",
                   "<b>Sense of distance</b>: the BO motors have no encoders, so the robot <i>estimates</i> its speed from the voltage it gives "
                   "the motors (a 'speed model' you calibrate once on a measured line). Optional slot encoders can measure it instead.",
                   "<b>Brain</b>: the ESP32 reads everything 1000 times per second, decides what to do, and sends commands.",
                   "<b>Muscles</b>: the L298N driver turns the ESP32's weak 3.3 V signals into strong motor currents.",
                   "<b>Driver's panel</b>: one START button (the only switch needed to start), a red LED for the end zone, and a status LED."]),
          PageBreak()]

    # -------------------------------------------------------------- 5 power --
    s += [P("5. Power: from battery to every part", H1), fig(FIG / "power_tree.png"),
          table([["Check", "Value", "OK?"],
                 ["Highest voltage on the robot", "16.8 V (full 4S pack)", "rule limit 24 V - OK"],
                 ["Motor voltage", "firmware keeps the average ≤ 6 V", "6 V class BO motors - OK (check yours)"],
                 ["Motor current (wheel blocked)", "≈ 1 A per motor (estimate)", "L298N 2 A - OK; jam detected, motors stop"],
                 ["3.3 V rail load", "≈ 235 mA worst case", "board regulator - OK (check its part number)"],
                 ["Normal current while driving", "≈ 0.4 A", "18650 pack lasts hours - OK"],
                 ["Fuse", "3 A", "protects the wires"]],
                [60 * mm, 55 * mm, W - 115 * mm]),
          P("<b>Charging:</b> the TP4056 charges <b>one</b> cell (4.2 V). It can never charge the 4-cell pack: either take the cells "
            "out and charge them one by one, or buy a 4S BMS board and a 16.8 V charger (about ₹600). Charge fully before every match: "
            "without the two battery-sense resistors the robot assumes a full pack. Stop below about 12.8 V (3.2 V per cell). "
            "Strap the battery holder so it can never fall out: the rules forbid leaving anything on the arena.",
            NOTE), PageBreak()]

    # ------------------------------------------------------------- 6 wiring --
    s += [P("6. Wiring: every wire, pin by pin", H1), fig(FIG / "wiring.png", max_h=175 * mm),
          caption("The full circuit diagram (also docs/Circuit_Diagram.pdf, with a wire-by-wire list). Every pin is checked "
                  "automatically against the robot program by a test."), PageBreak(),
          P("Pin table", H2), table(pin_rows(), [14 * mm, 55 * mm, W - 69 * mm]),
          Spacer(1, 3 * mm),
          P("Rules of thumb", H2),
          bullets(["All grounds are connected together (battery −, L298N, LM2596, ESP32, every sensor).",
                   "L298N: remove the ENA and ENB jumper caps (the ESP32 drives them) and the 5V-EN cap (the battery is above 12 V).",
                   "Power every sensor and the gyro from <b>3.3 V</b>, never 5 V, so their signals can never damage the ESP32.",
                   "Twist the two motor wires together and keep sensor wires away from them; a 100 nF capacitor across each motor helps.",
                   "Before first power-up: set the LM2596 to 5.0 V with a multimeter and check for short circuits."]),
          PageBreak()]

    # ------------------------------------------------------------- 7 build --
    s += [P("7. Building it step by step", H1),
          numbered([
              "<b>Ask the organisers</b> whether a purchased chassis plate is allowed (rule: no ready-made mechanisms; gearboxes are fine).",
              "<b>Motors and wheels</b> with the kit brackets; both shafts in line. A 100 nF capacitor across each motor helps.",
              "<b>Caster</b> at the back; on a flat floor the plate should be level.",
              "<b>Sensor bracket</b> at the front: IR array 100 mm ahead of the axle, 5–10 mm above the floor, sensor D1 on the robot's LEFT; "
              "the two FC-51 sensors at the ends, 45 mm either side of the centre, pointing down.",
              "<b>Battery holder</b> over or just ahead of the axle, strapped down.",
              "<b>Fuse and switch</b> after the battery +; wire the L298N and LM2596 (see the wiring chapter).",
              "<b>LM2596 to 5.0 V</b> with a multimeter before the ESP32 is connected. Remove the three L298N jumper caps.",
              "<b>ESP32, MPU6050 (flat, near the middle, on foam tape) and the red LED</b> where the referee can see it.",
              "<b>Wires</b> tied down; nothing hanging down.",
              "<b>Check</b> the size with a 220 mm box (including the sensor bracket) and weigh the robot. You may not make it lighter between runs.",
              "<b>Upload the firmware</b> (PlatformIO: <font face='Courier'>pio run -e esp32_l298n -t upload</font>) and follow the calibration steps (chapter 17).",
          ]), Spacer(1, 3 * mm), fig(FIG / "robot_layers.png", max_h=80 * mm), PageBreak()]

    # ------------------------------------------------------------- 8 sensing --
    s += [P("8. How the robot sees the line", H1), fig(FIG / "sensor_patterns.png"),
          bullets(["<b>Set the pots first:</b> each sensor's small LED must switch cleanly between the black floor and the white line.",
                   "<b>Calibration:</b> at START the robot swings its sensors left and right over the line for about 1.5 seconds. It learns, "
                   "for each sensor, which reading means white (some modules say 'white' with a high signal, some with a low one).",
                   "<b>Line position:</b> the white line is 30 mm wide, so it lights 1–3 neighbouring sensors (15 mm apart). Their centre "
                   "tells where the line is, in steps of 7.5 mm (left is negative, right is positive).",
                   "<b>Side branch:</b> a wing sensor sees white, or the white area reaches the array edge well beyond the line being "
                   "followed. The second check stops a robot that is slightly off-centre from 'seeing' a branch on the wrong side.",
                   "<b>Cross line:</b> all 5 sensors and both wings white. Only the distance travelled tells a junction (about 30 mm of white) from the end box (400 mm).",
                   "<b>Line gone:</b> if the line disappears while it was in the middle, it is a dead end. If it disappears at the edge, "
                   "the robot slid off and starts a search."]),
          PageBreak()]

    # ------------------------------------------------------------- 9 control --
    s += [P("9. How it drives straight and steers", H1), fig(FIG / "control_loop.png"),
          bullets(["<b>Steering (PD control):</b> if the line is 5 mm to the right, the robot turns right. How much depends on the error "
                   "(P) and on how fast the error is changing (D, which stops wobbling). Faster driving uses a stronger steering gain.",
                   "<b>Speed profile:</b> the robot speeds up smoothly, cruises, and brakes before the next turn. During the "
                   "actual run it knows the distances from its map, so it can go fast on long straight lines.",
                   "<b>Motor voltage:</b> without encoders the robot turns the wanted wheel speed straight into a motor voltage "
                   "(the speed model) and never gives the 6 V motors more than 6 V on average.",
                   "<b>Motor balance:</b> two BO motors are never equally strong. On straight lines the robot notices it keeps steering "
                   "to one side, and slowly learns a left/right correction (the 'trim').",
                   "<b>Heading:</b> the gyro holds the direction while crossing a junction and measures every turn.",
                   "All of this runs 1000 times per second without ever stopping the program (no 'delay' calls)."]),
          P("Tuned values (simulation; re-tune on the real robot): steering P 0.09 + 0.15 per m/s, D 0.004; exploring speed 0.45 m/s, "
            "known lines 0.7 m/s, actual run up to 1.0 m/s.", SMALL), PageBreak()]

    # -------------------------------------------------------- 10 junctions --
    s += [P("10. Junctions and 90° turns", H1), fig(FIG / "junction_timing.png"),
          bullets(["Because the sensors are ahead of the wheels, the robot has finished 'reading' a junction while its wheels are still "
                   "about 65 mm before it. That is the room it needs to brake.",
                   "To turn, it stops with the axle on the junction centre and spins on the spot. It measures the turn with the gyro, "
                   "but it only stops turning when the sensors see the new line in the middle. That way small errors never add up.",
                   "Plain corners (a line that only bends) are not decisions, so they are stored inside a map edge, not as junctions.",
                   "If it tries to turn into a line that does not exist (a misread), it notices that no line appears, deletes that "
                   "exit from the map, turns back and chooses again."]),
          fig(FIG / "results_sensor_distance.png", width=W * 0.75),
          caption("Simulation of the original N20 robot: with the sensors only 40 mm ahead it has no room to brake and fails half its "
                  "dry runs; from 55 mm up it works. The BO robot uses 100 mm."),
          PageBreak()]

    # ------------------------------------------------------------- 11 dry --
    s += [P("11. Dry run: exploring and making a map", H1), fig(FIG / "exploration_rule.png"),
          fig(IMG / "sim_after_dry_run.png", max_h=105 * mm),
          caption("Simulator after a dry run on the sample arena from the rulebook. White = real lines, orange = the robot's own map "
                  "(numbered junctions), magenta = the shortest route it found, green circles = checkpoints."),
          P("<b>The map.</b> Every junction is stored with its estimated position (speed model + gyro), which directions have "
            "lines, and which of those lines have already been driven. Lines between junctions are stored with their length "
            "and any corners. Because all lines meet at 90°, the robot always knows its direction exactly (north, east, south, west); "
            "only distances can drift a little, and that drift is removed every time it recognises a junction it has seen before."),
          PageBreak()]

    # ----------------------------------------------------------- 12 actual --
    s += [P("12. Actual run: the shortest route at speed", H1), fig(IMG / "sim_after_actual_run.png", max_h=125 * mm),
          caption("Simulator after the actual run: the robot drove the stored magenta route straight to the box."),
          bullets(["<b>Planning:</b> Dijkstra's algorithm adds up the line lengths and picks the shortest route. If two routes are "
                   "equally long (within a few centimetres), it picks the one with fewer turns, which is also faster.",
                   "The route is a simple list: 'at junction 1 go straight, at junction 3 turn right...'. It is saved in the ESP32's "
                   "flash memory together with the map and calibration, so it survives switching the robot off.",
                   "<b>Driving it:</b> the robot knows each line's length, so it accelerates on long lines and slows down just before "
                   "the next turn. Each turn is still triggered by the sensors seeing the junction, never by distance alone.",
                   "<b>If something does not match</b> (an unexpected dead end), the robot re-plans from where it is, so it still reaches the box."]),
          PageBreak()]

    # ------------------------------------------------------------- 13 goal --
    s += [P("13. Finding the end box and the red LED", H1), fig(FIG / "goal_detection.png"),
          bullets(["A junction makes the whole sensor array white too, but only for about 30 mm of travel. The end box keeps it white "
                   "for 400 mm. The robot waits for <b>70 mm of continuous white</b> before deciding: never fooled by a junction, and still "
                   "330 mm of box left.",
                   "At that moment the <b>red LED turns on</b> (rule: it must glow in the end zone).",
                   "The robot then drives another 140 mm and stops, so its whole body is inside the box.",
                   "If the white disappears during those 140 mm, it was not the box after all: the LED goes off and the robot reverses to "
                   "the junction it overshot.",
                   "Simulation of the purchased robot: false LED events are counted in every benchmark (chapter 18)."]),
          PageBreak()]

    # ------------------------------------------------------------- 14 states --
    s += [P("14. What the robot is doing (states, button, lights)", H1), fig(FIG / "state_machine.png", max_h=135 * mm),
          P("One button does everything", H2),
          table([["How you press START", "What happens"],
                 ["short press (when idle)", "starts the next run by itself: no map → dry run; map interrupted → dry-run restart; route stored → actual run"],
                 ["hold 1.5–4 s, then release (status LED goes solid)", "sensor calibration sweep on the START line + wiring check"],
                 ["hold more than 4 s, then release (status LED flickers)", "erase the stored map and route (calibration is kept)"],
                 ["any press while the robot is moving", "emergency stop: motors off immediately (the next press = restart)"]],
                [62 * mm, W - 62 * mm]),
          Spacer(1, 3 * mm),
          table([["Status LED", "Meaning"], ["slow blink", "no map yet: next press = dry run"],
                 ["double blink", "dry run was interrupted: next press = restart from the checkpoint"],
                 ["mostly on", "route stored: next press = actual run"], ["fast blink", "fault: press START to restart"],
                 ["RED LED on", "the robot is in the end box"]], [40 * mm, W - 40 * mm]),
          PageBreak()]

    # ----------------------------------------------------------- 15 restart --
    s += [P("15. Restarts", H1),
          P("The rules allow at most 3 restarts in the whole match. The clock never stops, and nobody may give the robot information "
            "about the maze. In the dry run the referee puts the robot back at the <b>checkpoint</b>; in the actual run, at <b>START</b>."),
          fig(FIG / "relocalization.png"),
          bullets(["The robot keeps its old map but does not know which checkpoint it was put on, or which way it faces.",
                   "It starts a small new map. After every new junction it tries laying the new map on top of the old one, at every "
                   "old junction and in all 4 directions. Wrong fits are thrown out as soon as a junction does not match.",
                   "When one fit is clearly the best (at least two junctions agree, and it beats every other fit), the two maps are "
                   "joined into one. The route from START to the box still works, so the actual run is not affected. If several fits are "
                   "still possible when it reaches the box, it picks the junction it visited most recently (the referee uses the <i>last</i> "
                   "checkpoint crossed). START counts as a possible restart point too.",
                   "Actual-run restart: the operator presses START to stop the robot, puts it on START and presses START again; it re-runs the stored route.",
                   "If the robot is bumped onto a different line without a restart, it notices that the map no longer makes sense, "
                   "goes back to the last trusted map, and re-joins it the same way."]),
          PageBreak()]

    # -------------------------------------------------------------- 16 match --
    s += [P("16. Match day, minute by minute", H1), fig(FIG / "match_timeline.png"),
          numbered(["Power on. Check the status LED. If it is not 'slow blink' (no map): hold START for more than 4 s and release, to erase old practice maps.",
                    "<b>Calibration minute:</b> put the robot on the START line, hold START for 2 s, release. It sweeps for about 1.5 s.",
                    "Put the robot at START facing along the line. <b>Wait for the organiser's signal</b>, press START once, hands off.",
                    "Dry run: the red LED comes on in the box and the robot stops inside. The status LED goes solid (route saved).",
                    "Carry it back to START, facing the same way. Wait for the signal, press START once.",
                    "Actual run: it follows the saved route; the red LED comes on in the box.",
                    "Restart only if the referee calls it: press START (stop), place the robot where told, press START. "
                    "Never erase the memory during a match."]),
          P("Not allowed near the arena: laptops, phones with Wi-Fi or Bluetooth on, recording. The robot's own radios are never switched on.",
            NOTE), PageBreak()]

    # --------------------------------------------------------- 17 calibration --
    s += [P("17. Calibration and tuning", H1),
          P("Do these phases in order; do not move on until the 'done when' column is true. The debug build of the firmware prints "
            "live telemetry (distance, angle, line position, every sensor value) over USB for this."),
          table([["Phase", "What you do", "Done when"],
                 ["0 Bench", "set the LM2596 to 5 V; check the gyro reads + for a left turn", "gyro answers; no short circuits"],
                 ["A Sensors", "set every pot; sensor sweep at START; slide the robot sideways over the line by hand", "every sensor switches; line position goes −30 → +30 mm"],
                 ["B Speed model", "drive between two T-junctions 1500 mm apart; correct ff_mm_s_per_v",
                  "measured distance within ±3 % over 10 runs"],
                 ["C Driving and turns", "follow a straight line and an oval; 20 left and 20 right turns; U-turns", "no wobble; every turn finds the new line"],
                 ["D Exploring", "small taped maze with T, crossing, dead end, loop, box", "10 dry runs in a row finish with the right map"],
                 ["E End box", "drive over every junction type 20×; enter the box 20×", "LED never at a junction; always in the box"],
                 ["F Speed", "raise actual-run speed 0.6 → 0.8 → 1.0 m/s", "10 clean runs before every increase"],
                 ["G Robustness", "low/full battery, different lights, power off between runs, a restart, a jammed wheel", "still 10/10"]],
                [27 * mm, 75 * mm, W - 102 * mm]),
          Spacer(1, 3 * mm),
          P("All tuning numbers live in one file, <font face='Courier'>firmware/include/config.h</font>, with their units. Change it, "
            "upload, and erase the memory. Remember: no code changes are allowed after the robot is handed in.", BODY),
          PageBreak()]

    # ------------------------------------------------------------- 18 results --
    fr, fh = bench("final_real"), bench("final_harsh")
    samp = [r for r in csv.DictReader((ROOT / "results/benchmark_results.csv").open())
            if r["label"] == "final_real" and r["arena"] == "techfest_2026_sample"]
    s += [P("18. How well does it work? (simulation)", H1),
          P("Each simulated match runs the <b>real robot program</b> on a model of the purchased robot (BO motors without encoders, "
            "digital sensors, gyro) with imperfections: sensor noise, unequal motors, gyro drift, a speed model that is a few per cent "
            "wrong, slip. Each match is complete: calibration, dry run, switching off, actual run, scored with the official formula. "
            "The mazes are random, rule-compliant ones the program was <b>not</b> tuned on."),
          table([["", "Realistic", "Harsh (worn wheels, noisy sensors)"],
                 ["Mazes", f"{fr['n']}", f"{fh['n']}"],
                 ["Dry run finished", f"{fr['dry']:.0f} %", f"{fh['dry']:.0f} %"],
                 ["Actual run finished", f"{fr['act']:.0f} %", f"{fh['act']:.0f} %"],
                 ["Actual run on the TRUE shortest path", f"{fr['short']:.0f} %", f"{fh['short']:.0f} %"],
                 ["Average dry / actual time", f"{fr['dry_s']:.0f} s / {fr['act_s']:.1f} s", f"{fh['dry_s']:.0f} s / {fh['act_s']:.1f} s"],
                 ["Average score", f"{fr['score']:.0f}", f"{fh['score']:.0f}"],
                 ["False LED · faults", f"{fr['false']} · {fr['faults']}", f"{fh['false']} · {fh['faults']}"]],
                [70 * mm, 45 * mm, W - 115 * mm]),
          Spacer(1, 4 * mm), fig(FIG / "results_bo.png"),
          caption("No encoders vs. optional slot encoders, same mazes."),
          P(("On the sample arena from the rulebook (realistic): " +
             (f"{samp[0]['checkpoints']}/4 checkpoints, {'shortest route' if samp[0]['shortest_path'] == '1' else 'NOT the shortest route'}, "
              f"dry run {float(samp[0]['dry_time_s']):.0f} s, actual run {float(samp[0]['actual_time_s']):.1f} s, score ≈ {float(samp[0]['total']):.0f}."
              if samp and samp[0]['dry_completed'] == '1' and samp[0]['actual_completed'] == '1' else "see results/test_summary.md.")), BODY),
          fig(FIG / "results_policy.png"),
          caption("Choosing the exploration rule (study done on the original N20 robot). 'Peek-and-prove' (look at the box, back out, "
                  "finish exploring, come back) is the best but is only usable if the organisers allow it."),
          PageBreak()]

    # ------------------------------------------------------------- 19 open --
    s += [P("19. Open questions and next steps", H1),
          P("Questions to send to the organisers", H2),
          numbered(["Can the robot see the end box, back out, keep exploring and return later? (decides 'peek-and-prove')",
                    "After a dry-run restart, which checkpoint is the robot placed on (the last one crossed?), and facing which way?",
                    "What exactly counts as 'crossing' a checkpoint? Are checkpoints always on junctions?",
                    "Is a purchased chassis plate a 'ready-made mechanism'? Is an ESP32 allowed if its radios are switched off?",
                    "How is 'shortest path' judged? Are several equally short routes all accepted?",
                    "What is C if the dry run takes longer than 3 minutes? What are the penalties P?",
                    "Is the time spent carrying the robot back to START counted? May we switch the robot off between runs?",
                    "Will START and the END box stay where the sample arena shows them?"]),
          P("Things to measure on the real parts", H2),
          bullets(["the BO motors' rated voltage and stall current (bench supply, under 1 s)",
                   "the ESP32 board's 3.3 V regulator part number",
                   "sensor positions, wheel size and track width (chapter 2) and the speed model (chapter 17, phase B)",
                   "whether the FC-51s work reliably as down-looking line sensors on the real floor and lighting"]),
          P("Your next steps, in order", H2),
          numbered(["Send the questions above, including: is the purchased chassis plate allowed, and is an ESP32 (radios off) allowed?",
                    "Buy the recommended additions: a 4S BMS + 16.8 V charger, and the few resistors and capacitors.",
                    "Assemble and wire the robot (chapters 6–7); check it with a multimeter before power-on.",
                    "Upload the debug firmware; do calibration phases 0, A and B.",
                    "Make a full-size practice arena: matte black sheet, 30 mm white vinyl tape, 400 × 400 mm white box.",
                    "Phases C–G; compare each real run's log with the simulator on the same layout."]),
          Spacer(1, 6 * mm),
          P("Where to find more detail: README.md, docs/ (rules, electrical, mechanical, algorithms, calibration, troubleshooting, "
            "competition checklist), firmware/README.md, simulator/README.md, results/test_summary.md.", SMALL)]

    doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=15 * mm, rightMargin=15 * mm, topMargin=16 * mm,
                            bottomMargin=15 * mm, title="Meshmerize Robot - Illustrated Guide",
                            author="Meshmerize robot project", subject="How the robot works, is built and wired, and runs a match")
    doc.build(s, onFirstPage=page_deco, onLaterPages=page_deco)
    print("wrote", OUT)


if __name__ == "__main__":
    build()
