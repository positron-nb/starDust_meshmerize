# Assembly guide (purchased 2-wheel chassis kit)

Parts: `hardware/bom.csv`. Wiring: `docs/Circuit_Diagram.pdf` and `docs/wiring_and_pinout.md`.
The parametric CAD in `hardware/cad/` models the original N20 design and is not needed for this
build; it can still be used to print a custom sensor bracket.

> **Ask the organisers first:** the rules forbid "ready-made mechanisms" but allow ready-made
> gearboxes (p.5). Ask whether a purchased chassis plate counts. If it does, a printed or
> laser-cut plate of the same size works with everything below.

## Geometry targets (they must match `config.h`; measure after building)
| Item | Target | Config parameter |
|---|---|---|
| IR array sensor line ahead of the wheel axle | **100 mm** (anything 80-120 mm works; measure it) | `sensor_forward_mm` |
| Array height above the floor | 5-10 mm (TCRT5000 best range) | — |
| Array centred on the robot's centreline, D1 on the robot's **left** | | `sensor_lateral_offset_mm`, `sensor_flip` |
| Wing sensors (FC-51) | in line with the array, **45 mm** left and right of the centreline, 5-10 mm above the floor, pointing down | `wing_lateral_mm` |
| Wheel centre to wheel centre | ~135 mm | `track_width_mm` |
| Wheel diameter | 65 mm | `wheel_diameter_mm` |
| Whole robot incl. sensors and wires | inside **220 × 220 × 220 mm** (rule p.5) | — |

Why 100 mm ahead: the robot classifies a junction once the array has crossed it and needs the
remaining distance to brake onto the junction centre. In simulation, distances from 55 to 90 mm all
worked for the original robot (`docs/mechanical_design.md`); the BO robot is tuned at 100 mm.

## Steps
1. **Motors and wheels.** Mount both BO motors with the kit brackets so the shafts are coaxial (lay
   a straight edge across both wheel hubs). Solder a 100 nF ceramic across each motor's terminals
   (recommended). Push the wheels fully on.
2. **Caster.** Fit the kit caster at the back. On a flat floor the plate should be level; shim if not.
3. **Sensor bracket.** Fix a strip (acrylic, aluminium, or a printed bracket) at the front so the IR
   array sits 100 mm ahead of the axle, 5-10 mm above the floor, components facing down. Fix the two
   FC-51 modules at its ends, 45 mm either side of the centre, pointing down. Nothing below the
   sensors may touch the floor.
4. **Battery.** 4S 18650 holder on top, over or just ahead of the axle (weight on the drive wheels
   helps braking and pivots). Strap it with velcro through the chassis slots.
5. **Power parts.** Fuse holder and rocker switch where you can reach them. Wire battery + → fuse →
   switch → L298N 12V and LM2596 IN+; battery − → L298N GND and LM2596 IN−.
6. **LM2596.** Before connecting the ESP32, switch on and set its output to **5.0 V** with a multimeter.
7. **L298N.** Remove the ENA, ENB and 5V-EN jumper caps. Motor wires to OUT1/OUT2 (left) and
   OUT3/OUT4 (right). LM2596 OUT+ to the L298N 5V terminal.
8. **ESP32 and MPU6050.** ESP32 on a small perfboard or breadboard strip with female headers. MPU6050
   flat, near the middle of the robot, on foam tape (vibration adds gyro noise). Wire everything as in
   the connection list (`Circuit_Diagram.pdf` page 2). All sensors on **3V3**.
9. **Red LED** where the referee can see it from above (GPIO15 → LED long leg; short leg → GND;
   330 Ω in series recommended).
10. **Wiring.** Twist each motor pair. Route sensor wires away from the motor leads. Zip-tie
    everything; nothing may hang down or fall off.
11. **Pre-power checks:** `docs/wiring_and_pinout.md` (multimeter list).
12. **Size check** with a real 220 mm box or a ruler, including the sensor bracket.
13. **Weigh** the robot and record it. Rule: you may not *reduce* the weight between runs.

## Firmware upload
See `firmware/README.md`. Competition build: `pio run -e esp32_l298n -t upload`.
Then work through `docs/calibration_guide.md`.
