"""Run a simulated dry run and export what the robot learned: its map, the
stored route (as turn actions per node and edge polylines) and the raw NVS
blob. Useful to compare with a real robot's log, or to inspect a route.

  python tools/export_route.py --arena sample --out results/sample_route.json
  python tools/export_route.py --arena gen:12 --set explore_start_weight=2
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.config import realistic  # noqa: E402
from simulator.main import load_arena  # noqa: E402
from simulator.scoring.match import Match, MatchConfig  # noqa: E402

REL = "SRBL"
DIRS = "NESW"


def actions(snap: dict) -> list[dict]:
    out = []
    h = 0  # robot starts facing N in its own map frame
    nodes, edges = snap["nodes"], snap["edges"]
    for node, exit_, ei in snap["route"]["steps"]:
        e = edges[ei]
        rel = REL[(exit_ - h) % 4]
        fwd = e["a"] == node and e["da"] == exit_
        segs = e["segs"]
        if not fwd:  # reversed edge: segment order flips and each corner turn changes sign
            lens = [l for l, _ in reversed(e["segs"])]
            turns = [-t for _, t in e["segs"][:-1]][::-1] + [0]
            segs = [[l, t] for l, t in zip(lens, turns)]
        arrive = e["db"] if fwd else e["da"]
        out.append({"node": node, "at": [round(nodes[node][0]), round(nodes[node][1])], "action": rel,
                    "exit": DIRS[exit_], "edge_mm": round(e["len"]),
                    "segments": [{"mm": l, "then": {1: "corner L", -1: "corner R", 0: "node"}[t]} for l, t in segs]})
        h = (arrive + 2) % 4
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arena", default="sample")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--out", default=str(ROOT / "results" / "route_export.json"))
    a = ap.parse_args()
    fw = {k: float(v) for k, v in (kv.split("=") for kv in a.set)}
    arena = load_arena(a.arena)
    m = Match(arena, realistic(a.seed, firmware=fw), MatchConfig(skip_actual=True))
    r = m.run()
    snap = r.final_map
    data = {"arena": arena.name, "dry_completed": r.outcome.dry_completed, "dry_time_s": r.outcome.dry_time_s,
            "route_length_mm": snap["route"]["length"], "true_shortest_mm": r.true_shortest_mm,
            "provably_shortest": r.route_certified, "actions": actions(snap) if snap["route"]["valid"] else [],
            "map": {"nodes": snap["nodes"], "edges": snap["edges"]},
            "nvs_blob_base64": base64.b64encode(m.sim.core.storage()).decode()}
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps(data, indent=1))
    print(" ".join(x["action"] for x in data["actions"]), f"({data['route_length_mm']:.0f} mm)")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
