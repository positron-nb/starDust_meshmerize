"""Dimensioned top + side drawing generated from the CAD model.

  python hardware/cad/dimension_drawing.py
Writes hardware/assembly_drawings/dimensioned_drawing.pdf and docs/images/dimensioned_drawing.png
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).parent))
import robot_cad as rc

ROOT = Path(__file__).resolve().parents[2]


def bb(shape):
    b = shape.val().BoundingBox()
    return b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax


def dim(ax, p0, p1, text, off=(0, 0), color="#C00"):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="<->", color=color, lw=0.9))
    mx, my = (p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1]
    ax.text(mx, my, text, color=color, ha="center", va="center", fontsize=8,
            bbox=dict(facecolor="white", edgecolor="none", pad=0.5))


def main():
    p = rc.P()
    parts, bought, rep = rc.build(p, out=False)
    shapes = {k: v[0] for k, v in parts.items()}
    shapes.update(bought)
    fig, (top, side) = plt.subplots(1, 2, figsize=(14, 6.5))
    for name, s in shapes.items():
        x0, x1, y0, y1, z0, z1 = bb(s)
        col = "#333" if name.startswith("wheel") else ("#B279A2" if name == "qtr" else "#4C78A8")
        top.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=col, lw=0.8))
        side.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fill=False, ec=col, lw=0.8))
    allb = [bb(s) for s in shapes.values()]
    X0, X1 = min(b[0] for b in allb), max(b[1] for b in allb)
    Y0, Y1 = min(b[2] for b in allb), max(b[3] for b in allb)
    Z1 = max(b[5] for b in allb)
    # top view dimensions
    dim(top, (X0, Y0 - 15), (X1, Y0 - 15), f"overall length {X1 - X0:.1f}", off=(0, -6))
    dim(top, (X1 + 12, Y0), (X1 + 12, Y1), f"width {Y1 - Y0:.1f}", off=(14, 0))
    dim(top, (0, -p.track / 2), (0, p.track / 2), f"track {p.track:.0f}", off=(14, 25))
    dim(top, (0, Y1 + 10), (p.sensor_fwd, Y1 + 10), f"axle→sensors {p.sensor_fwd:.0f}", off=(0, 6))
    dim(top, (p.caster_x, Y1 + 22), (0, Y1 + 22), f"caster→axle {-p.caster_x:.0f}", off=(0, 6))
    top.axvline(0, color="#999", lw=0.5, ls="--")
    top.set_title("TOP VIEW (mm) - x forward, y left")
    top.set_xlim(X0 - 25, X1 + 45)
    top.set_ylim(Y0 - 35, Y1 + 40)
    top.set_aspect("equal")
    # side view dimensions
    dim(side, (X1 + 10, 0), (X1 + 10, Z1), f"height {Z1:.1f}", off=(14, 0))
    dim(side, (p.sensor_fwd + 18, 0), (p.sensor_fwd + 18, p.sensor_height), f"sensor {p.sensor_height}", off=(18, 4))
    dim(side, (-25, 0), (-25, rc.axle_z(p)), f"axle {rc.axle_z(p):.0f}", off=(-14, 0))
    dim(side, (20, 0), (20, rc.plate_z(p)), f"plate {rc.plate_z(p):.1f}", off=(16, 0))
    side.axhline(0, color="#999", lw=0.8)
    side.set_title("SIDE VIEW (mm) - x forward, z up (floor at z = 0)")
    side.set_xlim(X0 - 30, X1 + 45)
    side.set_ylim(-12, Z1 + 20)
    side.set_aspect("equal")
    fig.suptitle(f"Meshmerize robot - envelope {X1 - X0:.1f} x {Y1 - Y0:.1f} x {Z1:.1f} mm (limit 220 cube). "
                 "Generated from hardware/cad/robot_cad.py - rendered geometry, not measured.", fontsize=10)
    fig.tight_layout()
    out1 = ROOT / "hardware" / "assembly_drawings" / "dimensioned_drawing.pdf"
    out2 = ROOT / "docs" / "images" / "dimensioned_drawing.png"
    fig.savefig(out1)
    fig.savefig(out2, dpi=110)
    print("wrote", out1, out2)


if __name__ == "__main__":
    main()
