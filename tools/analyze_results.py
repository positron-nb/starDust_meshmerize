"""Summarise benchmark CSVs written by run_batch_tests.py, grouped by label.

  python tools/analyze_results.py results/policy_study.csv
  python tools/analyze_results.py results/benchmark_results.csv --markdown results/benchmark_table.md
"""
from __future__ import annotations

import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def summarize(rows):
    ok = [r for r in rows if not r.get("error")]
    n = len(ok)
    if not n:
        return None
    dry = [f(r["dry_time_s"]) for r in ok if r["dry_completed"] == "1"]
    act = [f(r["actual_time_s"]) for r in ok if r["actual_completed"] == "1"]
    tot = [f(r["total"]) for r in ok]
    return {
        "runs": n,
        "dry %": 100 * sum(r["dry_completed"] == "1" for r in ok) / n,
        "actual %": 100 * sum(r["actual_completed"] == "1" for r in ok) / n,
        "shortest %": 100 * sum(r["shortest_path"] == "1" for r in ok) / n,
        "proven %": 100 * sum(r["route_certified"] == "1" for r in ok) / n,
        "checkpoints": statistics.mean(f(r["checkpoints"]) for r in ok),
        "dry s (mean)": statistics.mean(dry) if dry else float("nan"),
        "dry s (worst)": max(dry) if dry else float("nan"),
        "actual s (mean)": statistics.mean(act) if act else float("nan"),
        "score (mean)": statistics.mean(tot),
        "score (p10)": sorted(tot)[max(0, int(0.1 * n) - 1)],
        "score (min)": min(tot),
        "restarts": sum(int(r["restarts"] or 0) for r in ok),
        "false goals": sum(int(r["false_goal_led"] or 0) for r in ok),
        "line lost": sum(int(r["line_lost_events"] or 0) for r in ok),
        "faults": len([r for r in ok if r["faults"]]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--markdown", default=None)
    a = ap.parse_args()
    groups = defaultdict(list)
    for path in a.csv:
        for r in csv.DictReader(open(path)):
            groups[(r["label"], r["profile"])].append(r)
    table = []
    for (label, prof), rows in groups.items():
        s = summarize(rows)
        if s:
            table.append((label, prof, s))
    keys = list(table[0][2].keys()) if table else []
    lines = ["| policy | profile | " + " | ".join(keys) + " |", "|---|---|" + "---|" * len(keys)]
    for label, prof, s in table:
        cells = []
        for k in keys:
            v = s[k]
            cells.append(f"{v:.1f}" if isinstance(v, float) else str(v))
        lines.append(f"| {label} | {prof} | " + " | ".join(cells) + " |")
    text = "\n".join(lines)
    print(text)
    if a.markdown:
        Path(a.markdown).write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
