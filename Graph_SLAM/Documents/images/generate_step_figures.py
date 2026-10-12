"""Generate the per-step GraphSLAM figures used by graphslam_1d_example.md.

Run from anywhere:  python Graph_SLAM/generate_step_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Documents", "images")

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
# One list of measurements per step; the figures carry no titles (the Markdown has the headings).
STEPS = [
    [("prior", "x0", None, 0.0, 1.0)],
    [("obs", "x0", "L0", 2.05, 1.0)],
    [("odom", "x0", "x1", 1.10, 1.0), ("obs", "x1", "L0", 0.85, 1.0)],
    [("odom", "x1", "x2", 0.90, 1.0), ("obs", "x2", "L1", 1.55, 1.0)],
    [("odom", "x2", "x3", 1.20, 1.0), ("obs", "x3", "L2", 1.92, 1.0)],
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
    ax.set_ylim(n + 1.8, -1.3)
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


WORLD_FILES = [
    "step1_world_1d.png",
    "step2_world_1d.png",
    "step3_world_1d.png",
    "step4_world_1d.png",
    "step5_world_1d.png",
]
CONFLICT = "#ff8a80"
SOLUTION_FILE = "solution_world_1d.png"


def star_marker():
    pts = []
    for k in range(8):
        ang = np.pi / 2 + k * np.pi / 4
        r = 1.0 if k % 2 == 0 else 0.27
        pts.append((r * np.cos(ang), r * np.sin(ang)))
    return pts


def draw_world(fig, odoms, obs, positions, conflict_note):
    """Literal 1D view: every node sits at its measured position on one metre axis."""
    ax = fig.add_axes([0.02, 0.01, 0.96, 0.98])
    ax.set_facecolor(BG)
    ax.set_xlim(-1.9, 6.1)
    ax.set_ylim(-5.0, 6.8)
    ax.axis("off")

    ax.plot([-0.5, 5.8], [0, 0], color=TEXT, linewidth=2.2, zorder=2)
    for m in np.arange(-0.5, 5.81, 0.5):
        major = abs(m - round(m)) < 1e-9
        ax.plot([m, m], [0, 0.14 if major else 0.08], color=TEXT, linewidth=1.5, zorder=2)
        if major:
            ax.text(m, -0.1, f"{m:g}", color=TEXT, fontsize=13, ha="center", va="top")
    ax.text(5.95, 0.0, "m", color=TEXT, fontsize=14, ha="left", va="center")

    def guide(x, y0, y1, color):
        ax.plot([x, x], [y0, y1], color=color, linewidth=1.4, linestyle=(0, (2, 3)), zorder=1)

    for name, x in positions.items():
        if name.startswith("x"):
            ax.plot([x], [0.42], marker=">", markersize=24, markerfacecolor=BG, markeredgecolor=TEXT,
                    markeredgewidth=2.4, zorder=4)
            if name == "x0":
                ax.plot([x - 0.02], [0.42], "o", color=TEXT, markersize=5, zorder=5)
            ax.text(x, 0.95, LABELS[IDX[name]], color=TEXT, fontsize=20, ha="center", va="bottom",
                    bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)
        else:
            guide(x, 0.0, 1.8, OBS)
            ax.plot([x], [2.0], marker=star_marker(), markersize=28, markerfacecolor=BG,
                    markeredgecolor=TEXT, markeredgewidth=2.4, zorder=4)
            ax.text(x, 2.55, LABELS[IDX[name]], color=TEXT, fontsize=20, ha="center", va="bottom",
                    bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)

    # prior pin under x0
    ax.text(-0.15, -0.95, r"prior: $x_0=0$", color=ODOM, fontsize=14, ha="right", va="center")
    guide(0.0, 0.0, -0.95, ODOM)

    for k, (a, b, z) in enumerate(odoms):
        y = -1.75 - 0.8 * k
        x0, x1 = positions[a], positions[b]
        guide(x0, 0.0, y, ODOM)
        guide(x1, 0.0, y, ODOM)
        ax.annotate("", xy=(x1, y), xytext=(x0, y),
                    arrowprops=dict(arrowstyle="-|>", color=ODOM, lw=3, shrinkA=0, shrinkB=0,
                                    mutation_scale=22), zorder=3)
        ax.text((x0 + x1) / 2, y - 0.12, rf"odometry: ${b[0]}_{b[1]}-{a[0]}_{a[1]}={z:.2f}$",
                color=TEXT, fontsize=14, ha="center", va="top",
                bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)

    for k, (a, b, z, end, conflict) in enumerate(obs):
        y = 3.6 + 0.8 * k
        x0 = positions[a]
        color = CONFLICT if conflict else OBS
        guide(x0, 0.4, y, color)
        guide(end, 0.0 if conflict else 2.3, y, color)
        if conflict:
            ax.plot([end, end], [0, 0.3], color=CONFLICT, linewidth=3.5, zorder=3)
        ax.annotate("", xy=(end, y), xytext=(x0, y),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=3, shrinkA=0, shrinkB=0,
                                    mutation_scale=22), zorder=3)
        ax.text((x0 + end) / 2, y + 0.12, rf"sees: ${b[0]}_{b[1]}-{a[0]}_{a[1]}={z:.2f}$",
                color=TEXT, fontsize=14, ha="center", va="bottom",
                bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)

    if conflict_note:
        ax.text(5.95, -4.6, conflict_note, color=CONFLICT, fontsize=14, ha="right", va="center")


def draw_solution(fig, mu):
    """Number line showing the solved estimate of every pose and landmark."""
    ax = fig.add_axes([0.02, 0.02, 0.96, 0.96])
    ax.set_facecolor(BG)
    ax.set_xlim(-1.9, 6.1)
    ax.set_ylim(-0.9, 3.6)
    ax.axis("off")

    ax.plot([-0.5, 5.8], [0, 0], color=TEXT, linewidth=2.2, zorder=2)
    for m in np.arange(-0.5, 5.81, 0.5):
        major = abs(m - round(m)) < 1e-9
        ax.plot([m, m], [0, 0.14 if major else 0.08], color=TEXT, linewidth=1.5, zorder=2)
        if major:
            ax.text(m, -0.1, f"{m:g}", color=TEXT, fontsize=13, ha="center", va="top")
    ax.text(5.95, 0.0, "m", color=TEXT, fontsize=14, ha="left", va="center")

    for name, value in zip(NAMES, mu):
        label = rf"{LABELS[IDX[name]][:-1]}={value:.3f}$"
        if name.startswith("x"):
            ax.plot([value], [0.42], marker=">", markersize=24, markerfacecolor=BG,
                    markeredgecolor=TEXT, markeredgewidth=2.4, zorder=4)
            if name == "x0":
                ax.plot([value - 0.02], [0.42], "o", color=TEXT, markersize=5, zorder=5)
            ax.text(value, 0.95, label, color=TEXT, fontsize=16, ha="center", va="bottom",
                    bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)
        else:
            ax.plot([value, value], [0.0, 1.8], color=OBS, linewidth=1.4, linestyle=(0, (2, 3)), zorder=1)
            ax.plot([value], [2.0], marker=star_marker(), markersize=28, markerfacecolor=BG,
                    markeredgecolor=TEXT, markeredgewidth=2.4, zorder=4)
            ax.text(value, 2.55, label, color=TEXT, fontsize=16, ha="center", va="bottom",
                    bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    omega = np.zeros((7, 7))
    xi = np.zeros(7)
    nodes, edges = [], []
    positions = {"x0": 0.0}
    odoms, obs = [], []
    conflict_note = ""

    for measurements, fname, world_fname in zip(STEPS, FILES, WORLD_FILES):
        prev_omega, prev_xi = omega.copy(), xi.copy()
        for kind, a, b, z, w in measurements:
            add_measurement(omega, xi, kind, a, b, z, w)
            for name in (a, b):
                if name and name not in nodes:
                    nodes.append(name)
            if kind != "prior":
                edges.append((kind, a, b))
            if kind == "odom":
                positions[b] = positions[a] + z
                odoms.append((a, b, z))
            elif kind == "obs":
                end = positions[a] + z
                conflict = b in positions
                if not conflict:
                    positions[b] = end
                else:
                    gap = abs(end - positions[b])
                    conflict_note = (f"{b[0]}{b[1]} placed at {positions[b]:.2f} m from x0 but "
                                     f"{end:.2f} m from x1: {gap:.2f} m disagreement")
                obs.append((a, b, z, end, conflict))
        conflict_note_fig = conflict_note.replace("L0", "$L_0$").replace("x0", "$x_0$").replace("x1", "$x_1$")

        fig = plt.figure(figsize=(16, 6.0), facecolor=BG)
        draw_graph(fig.add_axes([0.01, 0.02, 0.40, 0.96]), nodes, edges)
        draw_system(fig.add_axes([0.43, 0.0, 0.56, 0.98]), omega, xi, prev_omega, prev_xi)
        fig.savefig(os.path.join(OUT_DIR, fname), dpi=140, facecolor=BG)
        plt.close(fig)

        fig = plt.figure(figsize=(16, 6.6), facecolor=BG)
        draw_world(fig, odoms, obs, positions, conflict_note_fig)
        fig.savefig(os.path.join(OUT_DIR, world_fname), dpi=140, facecolor=BG)
        plt.close(fig)

    expected_xi = np.array([-3.15, -0.65, -1.85, -0.72, 2.9, 1.55, 1.92])
    assert np.allclose(xi, expected_xi), xi
    assert np.allclose(np.diag(omega), [3, 3, 3, 2, 2, 1, 1]), np.diag(omega)

    mu = np.linalg.solve(omega, xi)
    expected_mu = np.array([0.0, 1.1333, 2.0333, 3.2333, 2.0167, 3.5833, 5.1533])
    assert np.allclose(mu, expected_mu, atol=5e-4), mu
    fig = plt.figure(figsize=(16, 4.4), facecolor=BG)
    draw_solution(fig, mu)
    fig.savefig(os.path.join(OUT_DIR, SOLUTION_FILE), dpi=140, facecolor=BG)
    plt.close(fig)

    print("Generated", len(FILES) + len(WORLD_FILES) + 1, "figures in", OUT_DIR)


if __name__ == "__main__":
    main()
