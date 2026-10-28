"""Arena model: white 30 mm vinyl lines on black, a 400 x 400 mm white end box,
a start pose and (invisible) checkpoint reference points.

Frame: millimetres, x to the right, y UP (math convention). The rulebook
figure has y pointing down; sample_arena.py converts.

Ground truth lives here. The robot core never sees this object - it only
receives simulated sensor readings computed from it.
"""
from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

LINE_WIDTH = 30.0
ARENA_SIZE = 2300.0
BOX_SIZE = 400.0


@dataclass
class Arena:
    name: str
    segments: list[tuple[float, float, float, float]]  # axis-aligned centrelines
    end_box: tuple[float, float, float, float]  # x, y, w, h (lower-left corner)
    start: tuple[float, float, float]  # x, y, heading_deg (0 = +x, CCW)
    checkpoints: list[tuple[float, float]] = field(default_factory=list)
    size: tuple[float, float] = (ARENA_SIZE, ARENA_SIZE)
    line_width: float = LINE_WIDTH
    notes: str = ""

    # ------------------------------------------------------------------ io --
    def to_json(self) -> dict:
        return {
            "name": self.name,
            "size_mm": list(self.size),
            "line_width_mm": self.line_width,
            "segments": [list(map(float, s)) for s in self.segments],
            "end_box": list(map(float, self.end_box)),
            "start": {"x": self.start[0], "y": self.start[1], "heading_deg": self.start[2]},
            "checkpoints": [list(map(float, c)) for c in self.checkpoints],
            "notes": self.notes,
        }

    def save(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_json(), indent=1))

    @staticmethod
    def from_json(d: dict) -> "Arena":
        st = d["start"]
        return Arena(
            name=d.get("name", "arena"),
            segments=[tuple(s) for s in d["segments"]],
            end_box=tuple(d["end_box"]),
            start=(st["x"], st["y"], st["heading_deg"]),
            checkpoints=[tuple(c) for c in d.get("checkpoints", [])],
            size=tuple(d.get("size_mm", (ARENA_SIZE, ARENA_SIZE))),
            line_width=d.get("line_width_mm", LINE_WIDTH),
            notes=d.get("notes", ""),
        )

    @staticmethod
    def load(path: str | Path) -> "Arena":
        return Arena.from_json(json.loads(Path(path).read_text()))

    # ------------------------------------------------------------ geometry --
    def rects(self) -> np.ndarray:
        """White rectangles [xmin, ymin, xmax, ymax]; lines are extended by half
        a width at both ends so that corners and T-junctions are filled."""
        h = self.line_width / 2
        out = []
        for x1, y1, x2, y2 in self.segments:
            out.append([min(x1, x2) - h, min(y1, y2) - h, max(x1, x2) + h, max(y1, y2) + h])
        bx, by, bw, bh = self.end_box
        out.append([bx, by, bx + bw, by + bh])
        return np.array(out, dtype=float)

    def in_box(self, x: float, y: float, margin: float = 0.0) -> bool:
        bx, by, bw, bh = self.end_box
        return bx + margin <= x <= bx + bw - margin and by + margin <= y <= by + bh - margin

    def validate(self) -> list[str]:
        """Geometric sanity checks (returns a list of problems)."""
        problems = []
        W, H = self.size
        for s in self.segments:
            x1, y1, x2, y2 = s
            if x1 != x2 and y1 != y2:
                problems.append(f"segment {s} is not axis-aligned")
            for x, y in ((x1, y1), (x2, y2)):
                if not (0 <= x <= W and 0 <= y <= H):
                    problems.append(f"segment {s} leaves the arena")
        bx, by, bw, bh = self.end_box
        if abs(bw - BOX_SIZE) > 1 or abs(bh - BOX_SIZE) > 1:
            problems.append("end box is not 400 x 400 mm")
        g = TruthGraph(self)
        if g.shortest_length() is None:
            problems.append("no path from START to the end box")
        return problems


class TruthGraph:
    """Ground-truth line graph used ONLY for evaluation (true shortest path,
    which junction a checkpoint belongs to, path optimality of a run).
    Independent of the C++ planner on purpose (cross-check)."""

    def __init__(self, arena: Arena):
        self.arena = arena
        pts: set[tuple[float, float]] = set()
        segs = arena.segments
        for x1, y1, x2, y2 in segs:
            pts.add((x1, y1))
            pts.add((x2, y2))
        # intersections between horizontal and vertical segments
        hs = [s for s in segs if s[1] == s[3]]
        vs = [s for s in segs if s[0] == s[2]]
        for hx1, hy, hx2, _ in hs:
            for vx, vy1, _, vy2 in vs:
                if min(hx1, hx2) <= vx <= max(hx1, hx2) and min(vy1, vy2) <= hy <= max(vy1, vy2):
                    pts.add((vx, hy))
        sx, sy, _ = arena.start
        pts.add((sx, sy))
        # box entry points: where a segment crosses the box boundary
        bx, by, bw, bh = arena.end_box
        self.goal_points: set[tuple[float, float]] = set()
        for x1, y1, x2, y2 in segs:
            for gx, gy in self._box_crossings(x1, y1, x2, y2, bx, by, bw, bh):
                pts.add((gx, gy))
                self.goal_points.add((gx, gy))
        self.points = sorted(pts)
        self.adj: dict[tuple[float, float], list[tuple[tuple[float, float], float]]] = {p: [] for p in self.points}
        for x1, y1, x2, y2 in segs:
            on = [p for p in self.points if self._on_seg(p, x1, y1, x2, y2)]
            on.sort(key=lambda p: (p[0] - x1) ** 2 + (p[1] - y1) ** 2)
            for a, b in zip(on, on[1:]):
                # do not route through the box interior
                mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
                if arena.in_box(mx, my, margin=1.0):
                    continue
                d = math.hypot(a[0] - b[0], a[1] - b[1])
                self.adj[a].append((b, d))
                self.adj[b].append((a, d))

    @staticmethod
    def _on_seg(p, x1, y1, x2, y2) -> bool:
        return min(x1, x2) - 1e-6 <= p[0] <= max(x1, x2) + 1e-6 and min(y1, y2) - 1e-6 <= p[1] <= max(y1, y2) + 1e-6

    @staticmethod
    def _box_crossings(x1, y1, x2, y2, bx, by, bw, bh):
        out = []
        if y1 == y2 and by <= y1 <= by + bh:
            for ex in (bx, bx + bw):
                if min(x1, x2) <= ex <= max(x1, x2):
                    out.append((ex, y1))
        if x1 == x2 and bx <= x1 <= bx + bw:
            for ey in (by, by + bh):
                if min(y1, y2) <= ey <= max(y1, y2):
                    out.append((x1, ey))
        return out

    def degree(self, p) -> int:
        return len(self.adj.get(p, []))

    def junctions(self) -> list[tuple[float, float]]:
        return [p for p in self.points if self.degree(p) >= 3]

    def dijkstra(self, src):
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, 1e18):
                continue
            for v, w in self.adj[u]:
                nd = d + w
                if nd < dist.get(v, 1e18) - 1e-9:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, prev

    def shortest_length(self) -> float | None:
        sx, sy, _ = self.arena.start
        dist, _ = self.dijkstra((sx, sy))
        best = [dist[g] for g in self.goal_points if g in dist]
        return min(best) if best else None

    def shortest_path(self) -> list[tuple[float, float]]:
        sx, sy, _ = self.arena.start
        dist, prev = self.dijkstra((sx, sy))
        goals = [g for g in self.goal_points if g in dist]
        if not goals:
            return []
        g = min(goals, key=lambda p: dist[p])
        path = [g]
        while path[-1] in prev:
            path.append(prev[path[-1]])
        return path[::-1]

    def total_length(self) -> float:
        return sum(abs(x2 - x1) + abs(y2 - y1) for x1, y1, x2, y2 in self.arena.segments)
