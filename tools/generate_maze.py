"""Generate random rule-compliant line mazes as arena JSON (+ PNG preview).

  python tools/generate_maze.py --seed 7 --out simulator/examples/maze_7.json
  python tools/generate_maze.py --count 10 --seed 100 --dir simulator/examples/generated
  python tools/generate_maze.py --sample --out simulator/examples/sample_arena.json
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.arena import Arena, GenParams, TruthGraph, generate, sample_arena  # noqa: E402


def preview(a: Arena, png: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_facecolor("black")
    for x1, y1, x2, y2 in a.segments:
        ax.plot([x1, x2], [y1, y2], color="white", linewidth=3, solid_capstyle="projecting")
    bx, by, bw, bh = a.end_box
    ax.add_patch(plt.Rectangle((bx, by), bw, bh, color="white"))
    ax.text(bx + bw / 2, by + bh / 2, "END", ha="center", va="center")
    for i, (cx, cy) in enumerate(a.checkpoints):
        ax.add_patch(plt.Circle((cx, cy), 60, fill=False, color="lime", linewidth=2))
        ax.text(cx + 70, cy + 70, str(i + 1), color="lime")
    sx, sy, _ = a.start
    ax.plot(sx, sy, "o", color="lime")
    ax.text(sx + 40, sy - 90, "START", color="lime")
    path = TruthGraph(a).shortest_path()
    if path:
        ax.plot([p[0] for p in path], [p[1] for p in path], color="magenta", linewidth=1.2)
    ax.set_xlim(0, a.size[0])
    ax.set_ylim(0, a.size[1])
    ax.set_aspect("equal")
    ax.set_title(f"{a.name}  shortest {TruthGraph(a).shortest_length():.0f} mm")
    fig.savefig(png, dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--out", default=None)
    ap.add_argument("--dir", default=str(ROOT / "simulator" / "examples" / "generated"))
    ap.add_argument("--sample", action="store_true", help="export the reconstructed sample arena")
    ap.add_argument("--loops", type=float, default=0.25)
    ap.add_argument("--cols", type=int, default=7)
    ap.add_argument("--rows", type=int, default=7)
    ap.add_argument("--no-png", action="store_true")
    a = ap.parse_args()
    arenas = []
    if a.sample:
        arenas.append(sample_arena())
    else:
        for i in range(a.count):
            arenas.append(generate(GenParams(seed=a.seed + i, loop_fraction=a.loops, cols=a.cols, rows=a.rows)))
    for ar in arenas:
        out = Path(a.out) if (a.out and len(arenas) == 1) else Path(a.dir) / f"{ar.name}.json"
        out.parent.mkdir(parents=True, exist_ok=True)
        ar.save(out)
        if not a.no_png:
            preview(ar, out.with_suffix(".png"))
        print(f"wrote {out}  (shortest {TruthGraph(ar).shortest_length():.0f} mm, {len(ar.segments)} segments, "
              f"problems: {ar.validate() or 'none'})")


if __name__ == "__main__":
    main()
