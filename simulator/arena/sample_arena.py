"""Reconstruction of the SAMPLE arena (Figure 1 of the Meshmerize Techfest
2026-27 problem statement).

Method: the embedded 1024 x 1024 figure was rasterised and every white strip
was located by pixel analysis (arena border at x = 102..921 px, y = 118..921
px, i.e. 3.56 px/cm horizontally and 3.49 px/cm vertically). Centrelines were
rounded to whole centimetres. The dimension labels printed in the figure are
not mutually consistent (e.g. the "48" and "28" labels near START span
roughly equal distances), so measured pixel positions were used, not labels.
Expect +-1-2 cm differences from the organisers' intended drawing - and the
real arena "will consist of alterations in the path" anyway.

Coordinates below are FIGURE centimetres (x right, y DOWN); they are
converted to the simulator frame (mm, y up) by `sample_arena()`.
"""
from __future__ import annotations

from .arena import Arena

# (x1, y1, x2, y2) in figure cm
_SEGMENTS_CM = [
    (20, 23, 212, 23),    # top line
    (20, 23, 20, 65),     # left loop, upper part
    (20, 65, 50, 65),
    (50, 23, 50, 131),    # main vertical through checkpoint 3
    (87, 23, 87, 65),
    (87, 65, 118, 65),
    (118, 23, 118, 101),
    (20, 101, 118, 101),  # horizontal through checkpoint 3 (dead end at x=20)
    (79, 101, 79, 159),   # dead-end stub
    (20, 131, 50, 131),
    (20, 131, 20, 200),
    (79, 131, 116, 131),  # dead-end stub
    (20, 200, 48, 200),   # into the END box
    (147, 23, 147, 89),   # stub below checkpoint 1
    (147, 58, 212, 58),
    (178, 58, 178, 91),
    (178, 91, 212, 91),
    (212, 23, 212, 122),  # right vertical through checkpoint 2
    (147, 122, 212, 122), # horizontal through checkpoint 4
    (178, 122, 178, 161),
    (128, 161, 207, 161),
    (128, 161, 128, 190), # dead end
    (157, 161, 157, 225), # dead end at the bottom
    (157, 200, 186, 200), # START stub
]
_BOX_CM = (48, 180, 40, 40)  # x, y(top in figure), w, h
_START_CM = (186, 200)       # robot faces -x (towards the junction at x=157)
_CHECKPOINTS_CM = [(147, 23), (212, 91), (50, 101), (178, 122)]

H = 2300.0


def _pt(x_cm: float, y_cm: float) -> tuple[float, float]:
    return x_cm * 10.0, H - y_cm * 10.0


def sample_arena() -> Arena:
    segs = []
    for x1, y1, x2, y2 in _SEGMENTS_CM:
        a = _pt(x1, y1)
        b = _pt(x2, y2)
        segs.append((a[0], a[1], b[0], b[1]))
    bx, by, bw, bh = _BOX_CM
    box = (bx * 10.0, H - (by + bh) * 10.0, bw * 10.0, bh * 10.0)
    sx, sy = _pt(*_START_CM)
    return Arena(
        name="techfest_2026_sample",
        segments=segs,
        end_box=box,
        start=(sx, sy, 180.0),
        checkpoints=[_pt(*c) for c in _CHECKPOINTS_CM],
        notes="Reconstructed from rulebook Figure 1 by pixel measurement; +-2 cm.",
    )
