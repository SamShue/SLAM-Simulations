"""Generate the measurement-to-matrix figure used by graphslam_introduction.md.

Run from anywhere:  python Graph_SLAM/Documents/images/generate_intro_figure.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
CELL_TEXT = "#0b1216"
ODOM = "#7fd1ff"
OBS = "#ffd166"
BOTH = "#c792ea"

NAMES = ["x0", "x1", "L0"]
LABELS = [r"$x_0$", r"$x_1$", r"$L_0$"]
IDX = {n: i for i, n in enumerate(NAMES)}
POS = {"x0": (0.0, 0.0), "x1": (2.4, 0.0), "L0": (3.9, 1.9)}


def star(ax, cx, cy, r_out=0.36, r_in=0.1):
    pts = []
    for k in range(8):
        ang = np.pi / 2 + k * np.pi / 4
        r = r_out if k % 2 == 0 else r_in
        pts.append((cx + r * np.cos(ang), cy + r * np.sin(ang)))
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))


def triangle(ax, cx, cy, anchored=False):
    s = 0.36
    pts = [(cx - s * 0.8, cy + s), (cx - s * 0.8, cy - s), (cx + s * 1.1, cy)]
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))
    if anchored:
        ax.plot([cx - s * 0.35], [cy], "o", color=TEXT, markersize=6, zorder=4)


def draw_graph(ax):
    ax.set_xlim(-1.1, 5.3)
    ax.set_ylim(-2.0, 3.1)
    ax.set_aspect("equal")
    ax.axis("off")

    (x0, y0), (x1, y1) = POS["x0"], POS["x1"]
    ax.plot([x0, x1], [y0, y1], color=ODOM, linewidth=3.4, zorder=1)

    (xa, ya), (xb, yb) = POS["x1"], POS["L0"]
    ax.plot([xa, xb], [ya, yb], color=OBS, linewidth=2.8, linestyle=(0, (4, 3)), zorder=1)

    for n in NAMES:
        x, y = POS[n]
        label = LABELS[IDX[n]]
        if n.startswith("x"):
            triangle(ax, x, y, anchored=(n == "x0"))
            ax.text(x, y - 0.66, label, color=TEXT, fontsize=20, ha="center", va="top")
        else:
            star(ax, x, y)
            ax.text(x, y + 0.56, label, color=TEXT, fontsize=20, ha="center", va="bottom")

    ax.plot([-0.9, -0.2], [-1.8, -1.8], color=ODOM, linewidth=3.4)
    ax.text(0.0, -1.8, "odometry edge", color=TEXT, fontsize=13, va="center")
    ax.plot([2.55, 3.25], [-1.8, -1.8], color=OBS, linewidth=2.8, linestyle=(0, (4, 3)))
    ax.text(3.45, -1.8, "observation edge", color=TEXT, fontsize=13, va="center")


def cell(ax, j, i, color):
    ax.add_patch(Rectangle((j, i), 1, 1, facecolor=color, edgecolor=GRID, linewidth=1.2, zorder=2))


def cell_text(ax, j, i, lines, fontsize=14):
    if len(lines) == 1:
        ax.text(j + 0.5, i + 0.5, lines[0], color=CELL_TEXT, fontsize=fontsize, ha="center",
                va="center", fontweight="bold", zorder=3)
        return
    dy = 0.27
    start = -(len(lines) - 1) * dy / 2
    for k, line in enumerate(lines):
        ax.text(j + 0.5, i + 0.5 + start + k * dy, line, color=CELL_TEXT, fontsize=fontsize - 1,
                ha="center", va="center", fontweight="bold", zorder=3)


def draw_matrix(ax):
    n = 3
    ax.set_xlim(-1.4, 9.6)
    ax.set_ylim(n + 1.95, -1.2)
    ax.set_aspect("equal")
    ax.axis("off")
    xi_x, xi_w = 7.7, 1.7

    for j in range(n):
        ax.text(j + 0.5, -0.3, LABELS[j], color=TEXT, fontsize=17, ha="center", va="bottom")
    for i in range(n):
        ax.text(-0.3, i + 0.5, LABELS[i], color=TEXT, fontsize=17, ha="right", va="center")
        ax.text(xi_x - 0.3, i + 0.5, LABELS[i], color=TEXT, fontsize=17, ha="right", va="center")

    # Empty grid first.
    for i in range(n):
        for j in range(n):
            cell(ax, j, i, BG)
        ax.add_patch(Rectangle((xi_x, i), xi_w, 1, facecolor=BG, edgecolor=GRID, linewidth=1.2, zorder=2))

    i0, i1, iL = IDX["x0"], IDX["x1"], IDX["L0"]

    # Odometry edge x0 -> x1 touches the 2x2 block at (x0, x1).
    cell(ax, i0, i0, ODOM)
    cell_text(ax, i0, i0, ["+w"])
    cell(ax, i1, i0, ODOM)
    cell_text(ax, i1, i0, ["\u2212w"])
    cell(ax, i0, i1, ODOM)
    cell_text(ax, i0, i1, ["\u2212w"])

    # Observation edge x1 -> L0 touches the 2x2 block at (x1, L0); (x1, x1) is shared.
    cell(ax, i1, i1, BOTH)
    cell_text(ax, i1, i1, ["+w", "+w'"])
    cell(ax, iL, i1, OBS)
    cell_text(ax, iL, i1, ["\u2212w'"])
    cell(ax, i1, iL, OBS)
    cell_text(ax, i1, iL, ["\u2212w'"])
    cell(ax, iL, iL, OBS)
    cell_text(ax, iL, iL, ["+w'"])

    ax.add_patch(Rectangle((xi_x, i0), xi_w, 1, facecolor=ODOM, edgecolor=GRID, linewidth=1.2, zorder=2))
    cell_text(ax, xi_x, i0, ["\u2212wz"])
    ax.add_patch(Rectangle((xi_x, i1), xi_w, 1, facecolor=BOTH, edgecolor=GRID, linewidth=1.2, zorder=2))
    cell_text(ax, xi_x, i1, ["+wz", "\u2212w'z'"])
    ax.add_patch(Rectangle((xi_x, iL), xi_w, 1, facecolor=OBS, edgecolor=GRID, linewidth=1.2, zorder=2))
    cell_text(ax, xi_x, iL, ["+w'z'"])

    ax.text(n / 2, n + 0.55, r"$\Omega$", color=TEXT, fontsize=26, ha="center", va="center")
    ax.text(xi_x + xi_w / 2, n + 0.55, r"$\xi$", color=TEXT, fontsize=26, ha="center", va="center")

    legend_y = n + 1.25
    ax.add_patch(Rectangle((0.0, legend_y), 0.4, 0.4, facecolor=ODOM, edgecolor=GRID))
    ax.text(0.55, legend_y + 0.2, "from odometry edge", color=TEXT, fontsize=12.5, va="center")
    ax.add_patch(Rectangle((3.4, legend_y), 0.4, 0.4, facecolor=OBS, edgecolor=GRID))
    ax.text(3.95, legend_y + 0.2, "from observation edge", color=TEXT, fontsize=12.5, va="center")
    ax.add_patch(Rectangle((7.3, legend_y), 0.4, 0.4, facecolor=BOTH, edgecolor=GRID))
    ax.text(7.85, legend_y + 0.2, "both add here", color=TEXT, fontsize=12.5, va="center")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    fig = plt.figure(figsize=(15.5, 6.3), facecolor=BG)
    draw_graph(fig.add_axes([0.005, 0.04, 0.315, 0.92]))
    draw_matrix(fig.add_axes([0.35, 0.08, 0.64, 0.84]))

    ax_arrow = fig.add_axes([0.0, 0.0, 1.0, 1.0])
    ax_arrow.set_xlim(0, 1)
    ax_arrow.set_ylim(0, 1)
    ax_arrow.axis("off")
    ax_arrow.add_patch(FancyArrowPatch((0.315, 0.5), (0.355, 0.5), arrowstyle="-|>",
                                        mutation_scale=26, color=TEXT, linewidth=2.4))

    fig.savefig(os.path.join(OUT_DIR, "intro_measurement_matrix.png"), dpi=140, facecolor=BG)
    plt.close(fig)


if __name__ == "__main__":
    main()
