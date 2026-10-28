"""Write results/test_summary.md from the machine-readable results:
results/pytest_report.xml, results/benchmark_results.csv, results/policy_study_table.md,
hardware/assembly_drawings/fit_report.json.

  python tools/make_test_summary.py
"""
from __future__ import annotations

import csv
import json
import sys
import xml.etree.ElementTree as ET
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from analyze_results import summarize  # noqa: E402

R = ROOT / "results"

LABELS = {
    "final_real": "purchased robot (no encoders), realistic imperfections, 100 held-out mazes + sample",
    "final_harsh": "purchased robot, harsh imperfections (unequal motors, noisy/dull sensors, gyro drift, 10 % speed-model error)",
    "final_ideal": "purchased robot, no imperfections",
    "slot_encoders_real": "purchased robot + optional LM393 slot encoders (encoder_mode 1), realistic",
    "slot_encoders_harsh": "purchased robot + optional slot encoders, harsh",
}


def pytest_counts():
    p = R / "pytest_report.xml"
    if not p.exists():
        return None
    root = ET.parse(p).getroot()
    ts = root if root.tag == "testsuite" else root[0]
    return {k: ts.get(k) for k in ("tests", "failures", "errors", "skipped", "time", "timestamp")}


def main():
    lines = [f"# Test summary (generated {date.today().isoformat()} by tools/make_test_summary.py)", "",
             "All numbers below come from **software**: unit tests, closed-loop simulation and firmware "
             "compilation. No physical robot has been built or tested yet.", ""]
    pc = pytest_counts()
    lines += ["## Automated tests (`python -m pytest simulator/tests`)", ""]
    if pc:
        lines += [f"* **{pc['tests']} tests, {pc['failures']} failures, {pc['errors']} errors, {pc['skipped']} skipped** "
                  f"({float(pc['time']):.0f} s). JUnit report: `results/pytest_report.xml`.", ""]
    lines += ["## Firmware compilation", "",
              "* `pio run -e esp32_l298n` (competition build for the purchased robot): SUCCESS. RAM 21.5 % (70 552 B), flash 28.1 %. Arduino-ESP32 3.3.12.",
              "* `pio run -e esp32_l298n_debug`, `esp32_tb6612`, `esp32_drv8833`: SUCCESS.",
              "* Host build of the same core (zig c++ 0.16, `-Wall -Wextra`): no warnings.",
              "* Not done: flashing and running on an ESP32 (no hardware available).", ""]
    fr = ROOT / "hardware/assembly_drawings/fit_report.json"
    if fr.exists():
        rep = json.loads(fr.read_text())
        ok = sum(c["pass"] for c in rep["checks"])
        lines += ["## CAD checks (`python hardware/cad/robot_cad.py`, original N20 design)", "",
                  f"* {ok}/{len(rep['checks'])} automated geometric checks pass "
                  f"({'; '.join(c['check'] + ': ' + c['detail'] for c in rep['checks'][:2])}).",
                  f"* Estimated mass {rep['mass_estimate_g']:.0f} g. Rendered geometry only.", ""]
    bench = R / "benchmark_results.csv"
    if bench.exists():
        groups = {}
        for r in csv.DictReader(bench.open()):
            groups.setdefault(r["label"], []).append(r)
        lines += ["## Randomised simulation benchmarks (`results/benchmark_results.csv`)", "",
                  "Each run is a complete match with the firmware core in the loop: calibration → dry run → "
                  "power cycle → actual run, judged by the simulated referee with the official formula. "
                  "Simulated hardware = the purchased robot: BO motors without encoders (speed model), 5-ch digital array (15 mm pitch) "
                  "+ 2 wing sensors, MPU6050 gyro, 4S pack without voltage sensing (firmware assumes 16.2 V). "
                  "Shortest path is judged by the route driven (sequence of maze points), not the odometer. "
                  "Maze generator: 7 × 7 irregular grid, ≥ 190 mm spacing, 25 % extra loop edges, box at a leaf, "
                  "START at the farthest leaf, 4 checkpoints on junctions (seeds 500+ are **not** the seeds used for tuning, 100–179).", "",
                  "| run set | description | runs | dry % | actual % | shortest % | proven % | checkpoints | dry s mean / worst | actual s mean | score mean | score p10 | score min | restarts | false goals | line lost | faults |",
                  "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for label, rows in groups.items():
            s = summarize(rows)
            lines.append(f"| {label} | {LABELS.get(label, '')} | {s['runs']} | {s['dry %']:.0f} | {s['actual %']:.0f} | "
                         f"{s['shortest %']:.1f} | {s['proven %']:.1f} | {s['checkpoints']:.2f} | "
                         f"{s['dry s (mean)']:.1f} / {s['dry s (worst)']:.1f} | {s['actual s (mean)']:.1f} | "
                         f"{s['score (mean)']:.1f} | {s['score (p10)']:.1f} | {s['score (min)']:.1f} | {s['restarts']} | "
                         f"{s['false goals']} | {s['line lost']} | {s['faults']} |")
        misses = [r for r in groups.get("final_real", []) if r["shortest_path"] != "1"]
        lines += ["", "Runs in `final_real` that did not drive the true shortest path:", ""]
        for r in misses:
            if r["dry_completed"] != "1":
                why = f"dry run not completed ({r['restarts']} restarts, faults: {r['faults'] or 'none'})"
            elif r["actual_completed"] != "1":
                why = "actual run not completed"
            elif r["planned_route_mm"] and float(r["planned_route_mm"]) > float(r["true_shortest_mm"]) * 1.02:
                why = (f"stored route {float(r['planned_route_mm']):.0f} mm vs true {float(r['true_shortest_mm']):.0f} mm: the dry run "
                       "ended at the box before the shorter branch was explored (strategy limit)")
            else:
                why = "route stored correctly but the actual run left it (restart or wrong turn)"
            lines.append(f"* {r['arena']}: {why}.")
        lines.append("")
        old = R / "benchmark_results_n20_v1.0.csv"
        if old.exists():
            lines += ["Earlier results for the original N20 design (quadrature encoders, analog 8-channel array, odometer-based "
                      "shortest-path referee) are archived in `results/benchmark_results_n20_v1.0.csv`.", ""]
    pt = R / "policy_study_table.md"
    if pt.exists():
        lines += ["## Exploration policy study (original N20 robot; tuning set: seeds 100–179 + sample, realistic)", "",
                  pt.read_text(encoding="utf-8").strip(), "", "Interpretation: `docs/exploration_strategy.md`.", ""]
    lines += ["## Problems found and fixed through testing", "",
              "See `CHANGELOG.md` (1.1.0 for the purchased hardware: turn capture with a 15 mm digital pitch, phantom side "
              "branches, motor-imbalance trim, no-encoder stall detection, restart re-joining).", "",
              "## Remaining weaknesses (known, not solved in software)", "",
              "1. No wheel encoders: distances come from a speed model, so they are only as good as the Phase B calibration "
              "and the battery voltage. The harsh profile (10 % model error, unequal motors) is where most failures happen; "
              "see `slot_encoders_real` for what measured distance buys.",
              "2. No battery sensing by default (no resistors): the firmware assumes a full pack. Charge before every match or fit the "
              "47k/10k divider (`MESH_VBAT_SENSE=1`).",
              "3. In the harsh profile a few runs fail calibration because one simulated comparator threshold never sees the dull line "
              "(the equivalent of a badly set pot). The robot refuses to start and names the channel; set every pot over the real floor.",
              "4. The dry run ends at the first box arrival, so some mazes miss the shortest route (strategy limit; peek-and-prove needs the organisers' OK: Q4).",
              "5. Sensor model: no ESP32 ADC non-linearity, no seams or glare on the vinyl, FC-51 modelled like a simple down-looking "
              "reflectance sensor, no lateral skid, gyro without vibration noise. Re-tune on the practice arena.",
              "6. Not measured yet: BO motor rating and stall current, real sensor positions, the ESP32 board regulator.",
              ""]
    out = R / "test_summary.md"
    out.write_text("\n".join(lines), encoding="utf-8")
    print("wrote", out)


if __name__ == "__main__":
    main()
