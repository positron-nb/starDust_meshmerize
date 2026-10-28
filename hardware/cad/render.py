"""Render the CAD assembly to PNG images for the documentation (matplotlib)."""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, str(Path(__file__).parent))
import robot_cad as rc

COLORS = {"chassis_plate": "#4C78A8", "motor_clamp": "#72B7B2", "sensor_bracket": "#F58518", "caster_mount": "#72B7B2",
          "battery_tray": "#54A24B", "deck_plate": "#4C78A8", "led_mount": "#E45756", "motor": "#888888",
          "wheel": "#222222", "qtr": "#B279A2", "caster": "#BBBBBB", "battery": "#EECA3B", "standoffs": "#999999",
          "esp32": "#2F2F2F", "tb6612": "#D62728", "buck": "#9467BD"}


def color(name):
    for k, v in COLORS.items():
        if name.startswith(k):
            return v
    return "#AAAAAA"


def tris(shape):
    verts, faces = shape.val().tessellate(0.5, 0.3)
    pts = [(v.x, v.y, v.z) for v in verts]
    return [[pts[i] for i in f] for f in faces]


def main():
    parts, bought, _ = rc.build(out=False)
    shapes = {k: v[0] for k, v in parts.items()}
    shapes.update(bought)
    out = Path(__file__).resolve().parents[2] / "docs" / "images"
    for name, (elev, azim) in {"robot_iso": (25, -60), "robot_side": (0, -90), "robot_top": (90, -90)}.items():
        fig = plt.figure(figsize=(9, 6.5))
        ax = fig.add_subplot(111, projection="3d")
        for k, s in shapes.items():
            pc = Poly3DCollection(tris(s), facecolor=color(k), edgecolor="none", alpha=0.95)
            ax.add_collection3d(pc)
        ax.set_xlim(-80, 90); ax.set_ylim(-85, 85); ax.set_zlim(0, 80)
        ax.set_box_aspect((170, 170, 80))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("x forward (mm)"); ax.set_ylabel("y left (mm)"); ax.set_zlabel("z (mm)")
        ax.set_title(f"Meshmerize robot CAD ({name.split('_')[1]} view) - rendered geometry, not a photo")
        fig.tight_layout()
        fig.savefig(out / f"{name}.png", dpi=110)
        plt.close(fig)
        print("wrote", out / f"{name}.png")


if __name__ == "__main__":
    main()
