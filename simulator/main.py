"""Meshmerize simulator entry point.

  python -m simulator.main                                 # sample arena, GUI
  python -m simulator.main --arena gen:7 --profile harsh   # generated maze 7, harsh conditions
  python -m simulator.main --arena my_arena.json --headless --out results/run.json
  python -m simulator.main --set explore_pref=2 --set explore_start_weight=1
  python -m simulator.main --inject dry:15:press:0         # judge-called restart at t=15 s

The robot is the real firmware core (firmware/src) compiled for the PC; the
simulator only supplies sensor readings and physics.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulator.arena import Arena, GenParams, generate, sample_arena  # noqa: E402
from simulator.config import SimParams, harsh, realistic  # noqa: E402
from simulator.scoring.match import Match, MatchConfig  # noqa: E402


def load_arena(spec: str) -> Arena:
    if spec == "sample":
        return sample_arena()
    if spec.startswith("gen:"):
        return generate(GenParams(seed=int(spec[4:])))
    return Arena.load(spec)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arena", default="sample", help="sample | gen:SEED | path to arena JSON")
    ap.add_argument("--profile", default="real", choices=["ideal", "real", "harsh"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--set", action="append", default=[], help="firmware parameter key=value")
    ap.add_argument("--inject", action="append", default=[], help="run:time_s:kind:value (kinds: kick drag jam press power)")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--speed", type=float, default=4.0, help="GUI speed-up factor")
    ap.add_argument("--no-power-cycle", action="store_true")
    ap.add_argument("--out", default=None, help="write result JSON (and .log next to it)")
    a = ap.parse_args(argv)

    arena = load_arena(a.arena)
    fw = {}
    for kv in a.set:
        k, v = kv.split("=")
        fw[k] = float(v)
    mk = {"ideal": lambda s, **k: SimParams(seed=s, **k), "real": realistic, "harsh": harsh}[a.profile]
    params = mk(a.seed, firmware=fw)
    inject = []
    for spec in a.inject:
        run, t, kind, val = spec.split(":")
        inject.append((run, float(t), kind, float(val)))
    match = Match(arena, params, MatchConfig(inject=inject, power_cycle_between_runs=not a.no_power_cycle))
    if not a.headless:
        from simulator.visualization.viewer import Viewer

        match.callback = Viewer(match, speed=a.speed)
        match.callback_every = 1
    try:
        r = match.run()
    except KeyboardInterrupt:
        print("stopped by user")
        return 1
    s = r.score
    print(f"arena {r.arena}: true shortest {r.true_shortest_mm:.0f} mm, robot route {r.planned_route_mm}")
    print(f"dry run   : completed={r.outcome.dry_completed} time={r.outcome.dry_time_s} checkpoints={r.outcome.checkpoints}")
    print(f"actual run: completed={r.outcome.actual_completed} time={r.outcome.actual_time_s} shortest={r.outcome.shortest_path}")
    print(f"restarts  : {r.outcome.restarts}")
    print(f"SCORE A={s.A} B={s.B} C={s.C:.1f} S={s.S} T={s.T:.1f} L={s.L} P={s.P}  TOTAL={s.total:.1f}")
    if a.out:
        out = Path(a.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"arena": r.arena, "outcome": r.outcome.__dict__, "score": s.as_dict(),
                                   "true_shortest_mm": r.true_shortest_mm, "planned_route_mm": r.planned_route_mm,
                                   "actual_path_mm": r.actual_path_mm, "metrics": r.metrics, "final_map": r.final_map},
                                  indent=1, default=str))
        out.with_suffix(".log").write_text("\n".join(r.log), encoding="utf-8")
        print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
