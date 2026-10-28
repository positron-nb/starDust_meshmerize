"""Randomised batch evaluation of the full robot (firmware core in the loop).

Each job = one complete simulated match (calibration, dry run, power cycle,
actual run, referee) on a generated maze (or the reconstructed sample).
Results are appended to a CSV; a summary is printed.

Examples
  python tools/run_batch_tests.py --mazes 40 --profile real --label dfs
  python tools/run_batch_tests.py --mazes 40 --profile real --label uc1 --set explore_start_weight=1.0
  python tools/run_batch_tests.py --mazes 20 --profile harsh --label harsh --include-sample
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

FIELDS = [
    "label", "profile", "arena", "sim_seed", "dry_completed", "dry_time_s", "checkpoints", "actual_completed",
    "actual_time_s", "shortest_path", "true_shortest_mm", "planned_route_mm", "actual_path_mm", "route_certified",
    "A", "B", "C", "S", "T", "L", "P", "total", "restarts", "faults", "false_goal_led", "missed_goal_s",
    "line_lost_events", "off_track", "stuck", "map_nodes", "true_junctions", "loop_closures", "dry_path_mm",
    "energy_mAh", "wall_s", "error",
]


def job(args):
    label, profile, arena_spec, sim_seed, fw, gen, mcfg, simkw = args
    from simulator.arena import GenParams, generate, sample_arena
    from simulator.config import harsh, realistic, SimParams
    from simulator.scoring.match import Match, MatchConfig

    t0 = time.time()
    row = {"label": label, "profile": profile, "sim_seed": sim_seed}
    try:
        if arena_spec == "sample":
            arena = sample_arena()
        else:
            arena = generate(GenParams(seed=int(arena_spec), **gen))
        row["arena"] = arena.name
        mk = {"real": realistic, "harsh": harsh, "ideal": lambda s, **k: SimParams(seed=s, **k)}[profile]
        p = mk(sim_seed, firmware=dict(fw), **dict(simkw))
        r = Match(arena, p, MatchConfig(**mcfg)).run()
        o, s, m = r.outcome, r.score, r.metrics
        row.update(
            dry_completed=int(o.dry_completed), dry_time_s=o.dry_time_s, checkpoints=o.checkpoints,
            actual_completed=int(o.actual_completed), actual_time_s=o.actual_time_s,
            shortest_path=int(o.shortest_path), true_shortest_mm=r.true_shortest_mm,
            planned_route_mm=r.planned_route_mm, actual_path_mm=r.actual_path_mm,
            route_certified=int(r.route_certified), restarts=o.restarts, faults="|".join(m["faults"]),
            false_goal_led=m["false_goal_led"], missed_goal_s=m["missed_goal_s"],
            line_lost_events=m["line_lost_events"], off_track=m["off_track"], stuck=m["stuck"],
            map_nodes=m["map_nodes"], true_junctions=m["true_junctions"], loop_closures=m["loop_closures"],
            dry_path_mm=round(m["dry_path_mm"]), energy_mAh=m["energy_mAh"],
        )
        row.update({k: round(v, 3) for k, v in s.as_dict().items() if k != "total"})
        row["total"] = round(s.total, 3)
    except Exception as e:  # keep the batch going, record the failure
        row["error"] = f"{type(e).__name__}: {e}"
    row["wall_s"] = round(time.time() - t0, 1)
    return row


def summarize(rows: list[dict]) -> dict:
    ok = [r for r in rows if not r.get("error")]
    n = len(ok)
    if not n:
        return {"runs": 0, "errors": len(rows)}

    def mean(key, subset=None):
        vals = [float(r[key]) for r in (subset or ok) if r.get(key) not in (None, "")]
        return round(statistics.mean(vals), 2) if vals else None

    dry_ok = [r for r in ok if r["dry_completed"]]
    act_ok = [r for r in ok if r["actual_completed"]]
    totals = [float(r["total"]) for r in ok]
    return {
        "runs": n,
        "errors": len(rows) - n,
        "dry_completion_rate": round(len(dry_ok) / n, 3),
        "actual_completion_rate": round(len(act_ok) / n, 3),
        "shortest_route_rate": round(sum(r["shortest_path"] for r in ok) / n, 3),
        "route_certified_rate": round(sum(r["route_certified"] for r in ok) / n, 3),
        "mean_checkpoints": mean("checkpoints"),
        "mean_dry_time_s": mean("dry_time_s", dry_ok),
        "worst_dry_time_s": max((float(r["dry_time_s"]) for r in dry_ok), default=None),
        "mean_actual_time_s": mean("actual_time_s", act_ok),
        "worst_actual_time_s": max((float(r["actual_time_s"]) for r in act_ok), default=None),
        "mean_total": round(statistics.mean(totals), 1),
        "min_total": round(min(totals), 1),
        "restarts": sum(int(r["restarts"]) for r in ok),
        "false_goal_led": sum(int(r["false_goal_led"]) for r in ok),
        "missed_goal_runs": sum(1 for r in ok if float(r["missed_goal_s"]) > 1.0),
        "line_lost_events": sum(int(r["line_lost_events"]) for r in ok),
        "off_track": sum(int(r["off_track"]) for r in ok),
        "faults": sorted({f for r in ok for f in str(r["faults"]).split("|") if f}),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mazes", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=100, help="first maze seed")
    ap.add_argument("--profile", default="real", choices=["ideal", "real", "harsh"])
    ap.add_argument("--label", default="default")
    ap.add_argument("--set", action="append", default=[], help="firmware param override key=value")
    ap.add_argument("--gen", action="append", default=[], help="maze generator param key=value")
    ap.add_argument("--match", action="append", default=[], help="MatchConfig field key=value (json)")
    ap.add_argument("--sim", action="append", default=[], help="SimParams field key=value (physical robot), e.g. sensor_forward_mm=55")
    ap.add_argument("--include-sample", action="store_true")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=str(ROOT / "results" / "benchmark_results.csv"))
    a = ap.parse_args()

    fw = {}
    for kv in a.set:
        k, v = kv.split("=")
        fw[k] = float(v)
    gen = {}
    for kv in a.gen:
        k, v = kv.split("=")
        gen[k] = float(v) if "." in v else int(v)
    simkw = {}
    for kv in a.sim:
        k, v = kv.split("=")
        simkw[k] = float(v)
    mcfg = {}
    for kv in a.match:
        k, v = kv.split("=")
        mcfg[k] = json.loads(v)

    # build the core once before forking workers
    from simulator.robot.core_bridge import load_lib

    load_lib()
    specs = [str(a.seed0 + i) for i in range(a.mazes)]
    if a.include_sample:
        specs.insert(0, "sample")
    jobs = [(a.label, a.profile, s, 1000 + i, tuple(fw.items()), gen, mcfg, tuple(simkw.items())) for i, s in enumerate(specs)]
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(job, jobs))
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    new = not out.exists()
    with out.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in FIELDS})
    s = summarize(rows)
    s.update(label=a.label, profile=a.profile, firmware_overrides=fw, sim_overrides=simkw, generator=gen,
             seconds=round(time.time() - t0))
    print(json.dumps(s, indent=1))
    with (out.parent / "batch_summaries.jsonl").open("a") as f:
        f.write(json.dumps(s) + "\n")
    for r in rows:
        if r.get("error"):
            print("ERROR", r["arena"] if "arena" in r else "?", r["error"])


if __name__ == "__main__":
    main()
