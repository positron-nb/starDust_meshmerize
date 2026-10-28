# Competition checklist and compliance

## A. Compliance with the official PDF
| Rule (page) | Requirement | Status / evidence |
|---|---|---|
| Size (p.5) | fits in 220 × 220 × 220 mm | purchased 2WD chassis + sensor bracket 100 mm ahead of the axle. ☐ **check the built robot with a box** (the bracket makes it longer). |
| Start (p.5) | started by one switch; on-board switch allowed | one START button (GPIO0) + power switch ✔ |
| LED (p.5) | red LED glows in the end zone | GPIO15 red LED, on at box confirmation in both runs ✔ (simulated) |
| Arena care (p.5) | no damage, no marks, nothing left behind | ☐ battery holder strapped; ☐ nothing touches the floor except wheels and caster; ☐ screws tight. |
| Power (p.5) | on-board supply; ≤ 24 V between any two points | 4S 18650, max 16.8 V ✔ |
| Construction (p.5) | no Lego / ready-made mechanism; ready-made gearboxes allowed | BO gear motors ✔ (gearboxes allowed). ☐ **ask the organisers whether the purchased chassis plate counts as a ready-made mechanism** (fallback: own plate of the same size). |
| Calibration (p.5) | 1 minute | sweep calibration ≈ 1.5 s ✔ |
| Code freeze (p.5) | no code change after deposit | all modes chosen by button + stored stage; no re-flash needed ✔ |
| Wireless (p.6) | Wi-Fi/Bluetooth devices off, no laptops | ESP32 radios never started (`btStop()` at boot) and the competition build has no serial logging ✔. ☐ ask whether an ESP32 is acceptable at all (it *has* radios, switched off). ☐ phones off, laptop away. |
| Restarts (p.4) | max 3; dry restart from checkpoint; no arena info fed | relocalisation without operator input ✔ (simulated) |
| Weight (p.4) | must not reduce weight between runs | ☐ weigh before the match, keep spares at the same weight |
| Team (p.7) | ≤ 4 members, one robot | ☐ |

## B. Packing list
Robot · 4 charged 18650 cells (+ 4 spares, same model) · charger (4S charger or TP4056 + single holder)
· multimeter (check the pack before each run) · spare BO motor · spare L298N and LM2596 · spare 3 A
fuses · small screwdriver for the IR/FC-51 pots · screwdriver set · zip ties · velcro · USB cable ·
printed copy of this checklist and the calibration guide · the last good firmware `.bin` (not
flashed after deposit).

## C. Night before
☐ Flash the **competition build** (`pio run -e esp32_l298n -t upload`) and record the git commit
hash (`git rev-parse --short HEAD`).
☐ Clear memory (hold START for more than 4 s), calibrate, run 3 full practice matches.
☐ Charge all cells full (16.8 V pack / 4.2 V per cell). The firmware assumes a full pack.
☐ Wheels clean (isopropyl wipe). Caster spins freely. IR and FC-51 pots not knocked.

## D. At the arena (one team member)
1. Power on. The status LED must show the stage. If it is not FRESH: hold START for more than 4 s
   to clear (calibration is kept).
2. **Calibration minute:** place the robot on the START line, hold START 2 s, release. Sweep ≈ 1.5 s.
3. Place the robot at START facing the line. **Wait for the signal.** Press START once. Hands off.
4. LED lights in the box (dry run done). The status LED goes solid (route stored). Pick up the
   robot. **Do not switch it off** (safe either way: the map is in flash).
5. Place it at START facing the same way. Wait for the signal. Press START once.
6. Restart (only if the referee calls it):
   * Press START to stop the robot.
   * Dry run: place it on the checkpoint the referee indicates, along the line in the direction of
     travel. Actual run: place it at START.
   * Press START. Do **not** clear memory and do not give the robot any information.

## E. After each match
Note the dry and actual times, the restarts, and anything odd. If allowed, copy the log over the
debug cable (not on competition day).
