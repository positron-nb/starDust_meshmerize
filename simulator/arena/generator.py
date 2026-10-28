"""Random line-maze generator obeying the rulebook geometry:
230 x 230 cm arena, 30 mm lines, all angles 90 degrees, a 400 x 400 mm end
box at the end of a line, START at the end of a stub, loops, dead ends,
T-junctions and 4-way crossings.

Method: pick irregular grid columns/rows (min spacing keeps parallel lines
far enough apart for a 64 mm sensor array), build a random spanning tree on
the grid graph, add extra edges for loops, prune some leaves, then attach
the end box and START to leaf nodes that are far apart.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .arena import ARENA_SIZE, BOX_SIZE, Arena, TruthGraph


@dataclass
class GenParams:
    cols: int = 7
    rows: int = 7
    min_spacing: float = 190.0
    margin: float = 120.0
    loop_fraction: float = 0.25      # extra edges / tree edges
    keep_fraction: float = 0.85      # fraction of grid nodes kept in the tree
    n_checkpoints: int = 4
    seed: int = 0


def _axis(rng: random.Random, n: int, lo: float, hi: float, min_sp: float) -> list[float]:
    for _ in range(200):
        xs = sorted(rng.uniform(lo, hi) for _ in range(n))
        xs = [round(x / 10) * 10 for x in xs]
        if all(b - a >= min_sp for a, b in zip(xs, xs[1:])):
            return xs
    step = (hi - lo) / (n - 1)
    return [round((lo + i * step) / 10) * 10 for i in range(n)]


def generate(p: GenParams) -> Arena:
    for attempt in range(60):
        a = _try_generate(p, attempt)
        if a is not None:
            return a
    raise RuntimeError(f"maze generation failed for seed {p.seed}")


def _try_generate(p: GenParams, attempt: int) -> Arena | None:
    rng = random.Random(p.seed * 1000 + attempt)
    xs = _axis(rng, p.cols, p.margin, ARENA_SIZE - p.margin, p.min_spacing)
    ys = _axis(rng, p.rows, p.margin, ARENA_SIZE - p.margin, p.min_spacing)
    nodes = [(i, j) for i in range(p.cols) for j in range(p.rows)]
    keep = set(rng.sample(nodes, int(len(nodes) * p.keep_fraction)))
    nbrs = {}
    for i, j in keep:
        nbrs[(i, j)] = [(i + di, j + dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)) if (i + di, j + dj) in keep]
    # random spanning tree (randomised DFS) from a random kept node
    root = rng.choice(sorted(keep))
    seen = {root}
    stack = [root]
    edges = set()
    while stack:
        u = stack[-1]
        cand = [v for v in nbrs[u] if v not in seen]
        if not cand:
            stack.pop()
            continue
        v = rng.choice(cand)
        seen.add(v)
        edges.add(frozenset((u, v)))
        stack.append(v)
    if len(seen) < len(keep) * 0.6:
        return None
    # loops
    all_pairs = [frozenset((u, v)) for u in seen for v in nbrs[u] if v in seen]
    extra = [e for e in set(all_pairs) if e not in edges]
    rng.shuffle(extra)
    for e in extra[: int(len(edges) * p.loop_fraction)]:
        edges.add(e)

    def deg(n):
        return sum(1 for e in edges if n in e)

    leaves = [n for n in seen if deg(n) == 1]
    if len(leaves) < 3:
        return None
    # END: a leaf whose single edge allows a box beyond it inside the arena
    def box_for(leaf):
        (e,) = [e for e in edges if leaf in e]
        (other,) = [n for n in e if n != leaf]
        lx, ly = xs[leaf[0]], ys[leaf[1]]
        ox, oy = xs[other[0]], ys[other[1]]
        dx, dy = (lx - ox), (ly - oy)
        L = math.hypot(dx, dy)
        dx, dy = dx / L, dy / L
        cx, cy = lx + dx * BOX_SIZE / 2, ly + dy * BOX_SIZE / 2
        bx, by = cx - BOX_SIZE / 2, cy - BOX_SIZE / 2
        if bx < 10 or by < 10 or bx + BOX_SIZE > ARENA_SIZE - 10 or by + BOX_SIZE > ARENA_SIZE - 10:
            return None
        return (bx, by, BOX_SIZE, BOX_SIZE)

    def seg_of(e):
        u, v = tuple(e)
        return (xs[u[0]], ys[u[1]], xs[v[0]], ys[v[1]])

    def box_clear(box, skip_leaf):
        bx, by, bw, bh = box
        m = 60.0
        for e in edges:
            if skip_leaf in e:
                continue
            x1, y1, x2, y2 = seg_of(e)
            if max(x1, x2) >= bx - m and min(x1, x2) <= bx + bw + m and max(y1, y2) >= by - m and min(y1, y2) <= by + bh + m:
                return False
        return True

    rng.shuffle(leaves)
    end_leaf = None
    box = None
    for lf in leaves:
        b = box_for(lf)
        if b and box_clear(b, lf):
            end_leaf, box = lf, b
            break
    if end_leaf is None:
        return None
    # START: the leaf farthest (Manhattan) from the end leaf
    others = [lf for lf in leaves if lf != end_leaf]
    start_leaf = max(others, key=lambda n: abs(xs[n[0]] - xs[end_leaf[0]]) + abs(ys[n[1]] - ys[end_leaf[1]]))
    (se,) = [e for e in edges if start_leaf in e]
    (snb,) = [n for n in se if n != start_leaf]
    sx, sy = xs[start_leaf[0]], ys[start_leaf[1]]
    heading = math.degrees(math.atan2(ys[snb[1]] - sy, xs[snb[0]] - sx))
    if abs(xs[snb[0]] - sx) + abs(ys[snb[1]] - sy) < 150:
        return None
    segs = [seg_of(e) for e in edges]
    arena = Arena(
        name=f"gen_seed{p.seed}",
        segments=segs,
        end_box=box,
        start=(sx, sy, heading),
        checkpoints=[],
        notes=f"generated: {p}",
    )
    tg = TruthGraph(arena)
    if tg.shortest_length() is None:
        return None
    # checkpoints: 2 junctions on the shortest path + others at random
    path = set(tg.shortest_path())
    juncs = tg.junctions()
    on_path = [j for j in juncs if j in path]
    off_path = [j for j in juncs if j not in path]
    rng.shuffle(on_path)
    rng.shuffle(off_path)
    cps = (on_path[:2] + off_path)[: p.n_checkpoints]
    if len(cps) < p.n_checkpoints:
        cps = (on_path + off_path)[: p.n_checkpoints]
    arena.checkpoints = cps
    if arena.validate():
        return None
    return arena
