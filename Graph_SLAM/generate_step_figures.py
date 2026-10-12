"""Generate the per-step GraphSLAM figures used by graphslam_1d_example.md.

Run from anywhere:  python Graph_SLAM/generate_step_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")

BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
CELL_ON = "#d5dde2"
CELL_NEW = "#ffd166"
CELL_TEXT = "#0b1216"
ODOM = "#7fd1ff"
OBS = "#ffd166"

NAMES = ["x0", "x1", "x2", "x3", "L0", "L1", "L2"]
LABELS = [r"$x_0$", r"$x_1$", r"$x_2$", r"$x_3$", r"$L_0$", r"$L_1$", r"$L_2$"]
IDX = {n: i for i, n in enumerate(NAMES)}

POS = {
    "x0": (0.0, 0.0),
    "x1": (2.0, 0.0),
    "x2": (4.0, 0.0),
    "x3": (6.0, 0.0),
    "L0": (1.0, 1.7),
    "L1": (4.6, -1.7),
    "L2": (6.8, 1.7),
}

# (kind, i, j, z, weight): kind "prior" fixes a variable, others are v_j - v_i = z.
STEPS = [
    ("Step 1: Anchor the first pose", [("prior", "x0", None, 0.0, 100.0)]),
    ("Step 2: Observe landmark $L_0$ from $x_0$", [("obs", "x0", "L0", 2.05, 25.0)]),
    (
        "Step 3: Move to $x_1$ and observe $L_0$ again",
        [("odom", "x0", "x1", 1.10, 100.0), ("obs", "x1", "L0", 0.85, 25.0)],
    ),
    (
        "Step 4: Move to $x_2$ and observe new landmark $L_1$",
        [("odom", "x1", "x2", 0.90, 100.0), ("obs", "x2", "L1", 1.55, 25.0)],
    ),
    (
        "Step 5: Move to $x_3$ and observe new landmark $L_2$",
        [("odom", "x2", "x3", 1.20, 100.0), ("obs", "x3", "L2", 1.92, 25.0)],
    ),
]
FILES = [
    "step1_anchor.png",
    "step2_landmark.png",
    "step3_move_to_x1.png",
    "step4_move_to_x2.png",
    "step5_final.png",
]


def add_measurement(omega, xi, kind, a, b, z, w):
    i = IDX[a]
    if kind == "prior":
        omega[i, i] += w
        xi[i] += w * z
        return
    j = IDX[b]
    omega[i, i] += w
    omega[j, j] += w
    omega[i, j] -= w
    omega[j, i] -= w
    xi[i] -= w * z
    xi[j] += w * z


def fmt(v):
    return f"{v:.2f}".rstrip("0").rstrip(".") if abs(v - round(v)) > 1e-9 else f"{int(round(v))}"


def star(ax, cx, cy, r_out=0.34, r_in=0.09):
    pts = []
    for k in range(8):
        ang = np.pi / 2 + k * np.pi / 4
        r = r_out if k % 2 == 0 else r_in
        pts.append((cx + r * np.cos(ang), cy + r * np.sin(ang)))
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))


def triangle(ax, cx, cy, anchored):
    s = 0.34
    pts = [(cx - s * 0.8, cy + s), (cx - s * 0.8, cy - s), (cx + s * 1.1, cy)]
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))
    if anchored:
        ax.plot([cx - s * 0.35], [cy], "o", color=TEXT, markersize=6, zorder=4)


def draw_graph(ax, nodes, edges):
    ax.set_xlim(-1.0, 7.9)
    ax.set_ylim(-3.6, 2.7)
    ax.set_aspect("equal")
    ax.axis("off")
    for kind, a, b in edges:
        (x1, y1), (x2, y2) = POS[a], POS[b]
        if kind == "odom":
            ax.plot([x1, x2], [y1, y2], color=ODOM, linewidth=3, zorder=1)
        else:
            ax.plot([x1, x2], [y1, y2], color=OBS, linewidth=2.6, linestyle=(0, (4, 3)), zorder=1)
    for n in nodes:
        x, y = POS[n]
        label = LABELS[IDX[n]]
        if n.startswith("x"):
            triangle(ax, x, y, anchored=(n == "x0"))
            ax.text(x, y - 0.62, label, color=TEXT, fontsize=20, ha="center", va="top")
        else:
            star(ax, x, y)
            ax.text(x, y + 0.5 if y > 0 else y - 0.5, label, color=TEXT, fontsize=20, ha="center",
                    va="bottom" if y > 0 else "top")
    # legend
    ax.plot([-0.9, -0.2], [-3.4, -3.4], color=ODOM, linewidth=3)
    ax.text(0.0, -3.4, "odometry / prior", color=TEXT, fontsize=13, va="center")
    ax.plot([2.9, 3.6], [-3.4, -3.4], color=OBS, linewidth=2.6, linestyle=(0, (4, 3)))
    ax.text(3.8, -3.4, "landmark observation", color=TEXT, fontsize=13, va="center")


def draw_system(ax, omega, xi, prev_omega, prev_xi):
    n = len(NAMES)
    ax.set_xlim(-1.3, 10.6)
    ax.set_ylim(n + 1.5, -1.3)
    ax.set_aspect("equal")
    ax.axis("off")
    xi_x, xi_w = 8.6, 1.9

    for j in range(n):
        ax.text(j + 0.5, -0.35, LABELS[j], color=TEXT, fontsize=18, ha="center", va="bottom")
    for i in range(n):
        ax.text(-0.3, i + 0.5, LABELS[i], color=TEXT, fontsize=18, ha="right", va="center")
        ax.text(xi_x - 0.3, i + 0.5, LABELS[i], color=TEXT, fontsize=18, ha="right", va="center")

    for i in range(n):
        for j in range(n):
            v = omega[i, j]
            on = abs(v) > 1e-9
            new = abs(v - prev_omega[i, j]) > 1e-9
            face = CELL_NEW if new else (CELL_ON if on else BG)
            ax.add_patch(Rectangle((j, i), 1, 1, facecolor=face, edgecolor=GRID, linewidth=1.2))
            if on:
                ax.text(j + 0.5, i + 0.5, fmt(v), color=CELL_TEXT, fontsize=14, ha="center",
                        va="center", fontweight="bold")
    for i in range(n):
        v = xi[i]
        on = abs(v) > 1e-9
        new = abs(v - prev_xi[i]) > 1e-9
        face = CELL_NEW if new else (CELL_ON if on else BG)
        ax.add_patch(Rectangle((xi_x, i), xi_w, 1, facecolor=face, edgecolor=GRID, linewidth=1.2))
        if on:
            ax.text(xi_x + xi_w / 2, i + 0.5, fmt(v), color=CELL_TEXT, fontsize=14, ha="center",
                    va="center", fontweight="bold")

    ax.text(n / 2, n + 0.6, r"$\Omega$", color=TEXT, fontsize=26, ha="center", va="center")
    ax.text(xi_x + xi_w / 2, n + 0.6, r"$\xi$", color=TEXT, fontsize=26, ha="center", va="center")
    ax.add_patch(Rectangle((0.0, n + 1.15), 0.4, 0.4, facecolor=CELL_NEW, edgecolor=GRID))
    ax.text(0.6, n + 1.35, "updated this step", color=TEXT, fontsize=13, va="center")
    ax.add_patch(Rectangle((3.6, n + 1.15), 0.4, 0.4, facecolor=CELL_ON, edgecolor=GRID))
    ax.text(4.2, n + 1.35, "non-zero", color=TEXT, fontsize=13, va="center")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    omega = np.zeros((7, 7))
    xi = np.zeros(7)
    nodes, edges = [], []

    for (title, measurements), fname in zip(STEPS, FILES):
        prev_omega, prev_xi = omega.copy(), xi.copy()
        for kind, a, b, z, w in measurements:
            add_measurement(omega, xi, kind, a, b, z, w)
            for name in (a, b):
                if name and name not in nodes:
                    nodes.append(name)
            if kind != "prior":
                edges.append((kind, a, b))

        fig = plt.figure(figsize=(16, 6.6), facecolor=BG)
        fig.text(0.5, 0.955, title, color=TEXT, fontsize=24, ha="center", va="center")
        draw_graph(fig.add_axes([0.01, 0.03, 0.40, 0.86]), nodes, edges)
        draw_system(fig.add_axes([0.43, 0.01, 0.56, 0.90]), omega, xi, prev_omega, prev_xi)
        fig.savefig(os.path.join(OUT_DIR, fname), dpi=140, facecolor=BG)
        plt.close(fig)

    expected_xi = np.array([-161.25, -1.25, -68.75, 72.0, 72.5, 38.75, 48.0])
    assert np.allclose(xi, expected_xi), xi
    assert omega[2, 2] == 225 and omega[3, 3] == 125 and omega[4, 4] == 50
    print("Generated", len(FILES), "figures in", OUT_DIR)


if __name__ == "__main__":
    main()
