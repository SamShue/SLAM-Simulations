"""Generate the figures (and the numbers) used by graphslam_2d_example.md.

Run from anywhere:  python Graph_SLAM/generate_2d_figures.py
Print the LaTeX blocks quoted in the Markdown:  python Graph_SLAM/generate_2d_figures.py --latex

State layout: [x0 y0 th0 | x1 y1 th1 | x2 y2 th2 | L0x L0y | L1x L1y]
Measurements (all expressed in the frame of the observing pose):
  prior : the pose itself
  odom  : relative pose of pose j in the frame of pose i  (dx, dy, dtheta)
  lm    : position of landmark k in the frame of pose i   (forward, left)
Each constraint adds  J^T W J  to Omega  and  J^T W (z - h(mu) + J mu)  to xi,
where W is the inverse of the measurement covariance.
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_step_figures import (  # noqa: E402
    BG, CELL_NEW, CELL_ON, CELL_TEXT, CONFLICT, GRID, ODOM, OBS, TEXT, star_marker,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")

N_POSES, N_LM = 3, 2
DIM = 3 * N_POSES + 2 * N_LM
BLOCKS = [0, 3, 6, 9, 11, 13]

# Measurement standard deviations: (x, y, theta) for poses, (forward, left) for landmarks.
SIG_PRIOR = (0.1, 0.1, 0.05)
SIG_ODOM = (0.1, 0.1, 0.05)
SIG_LM = (0.1, 0.2)


def information(sig):
    return np.diag(1.0 / np.square(sig))


W_PRIOR, W_ODOM, W_LM = information(SIG_PRIOR), information(SIG_ODOM), information(SIG_LM)

# (kind, i, j, z, W)
MEASUREMENTS = [
    ("prior", 0, None, np.array([0.0, 0.0, 0.0]), W_PRIOR),
    ("lm", 0, 0, np.array([3.0, 1.0]), W_LM),
    ("odom", 0, 1, np.array([2.0, 0.0, np.pi / 2]), W_ODOM),
    ("lm", 1, 0, np.array([1.15, -0.90]), W_LM),
    ("odom", 1, 2, np.array([1.5, 0.0, 0.0]), W_ODOM),
    ("lm", 2, 1, np.array([1.5, 1.5]), W_LM),
]
STEPS = [[0], [1], [2, 3], [4, 5]]
STEP_FILES = [(f"step{n}_world_2d.png", f"step{n}_system_2d.png") for n in range(1, 5)]
LAYOUT_FILE = "layout_2d.png"
SOLUTION_FILE = "solution_2d.png"


def rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]])


def d_rot_t(t):
    """Derivative of R(t)^T with respect to t."""
    c, s = np.cos(t), np.sin(t)
    return np.array([[-s, c], [-c, -s]])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compose(p, z):
    return np.array([*(p[:2] + rot(p[2]) @ z[:2]), wrap(p[2] + z[2])])


def pose_slice(i):
    return slice(3 * i, 3 * i + 3)


def lm_slice(k):
    return slice(3 * N_POSES + 2 * k, 3 * N_POSES + 2 * k + 2)


def initial_guess():
    """Dead reckoning for the poses, first observation for each landmark."""
    mu = np.zeros(DIM)
    mu[pose_slice(1)] = compose(mu[pose_slice(0)], MEASUREMENTS[2][3])
    mu[pose_slice(2)] = compose(mu[pose_slice(1)], MEASUREMENTS[4][3])
    mu[lm_slice(0)] = mu[:2] + rot(mu[2]) @ MEASUREMENTS[1][3]
    p2 = mu[pose_slice(2)]
    mu[lm_slice(1)] = p2[:2] + rot(p2[2]) @ MEASUREMENTS[5][3]
    return mu


def linearize(m, mu):
    """Return [(slice, J)], h(mu), and z_hat = z - h + J mu for one measurement."""
    kind, i, j, z, _ = m
    if kind == "prior":
        s = pose_slice(i)
        h = mu[s].copy()
        pieces = [(s, np.eye(3))]
        jmu = mu[s].copy()
        zhat = z.copy()
        zhat[2] = wrap(z[2] - h[2]) + jmu[2]
        zhat[:2] = z[:2] - h[:2] + jmu[:2]
        return pieces, h, zhat
    if kind == "odom":
        si, sj = pose_slice(i), pose_slice(j)
        pi, pj = mu[si], mu[sj]
        d = pj[:2] - pi[:2]
        h = np.array([*(rot(pi[2]).T @ d), wrap(pj[2] - pi[2])])
        ji = np.zeros((3, 3))
        ji[:2, :2] = -rot(pi[2]).T
        ji[:2, 2] = d_rot_t(pi[2]) @ d
        ji[2, 2] = -1
        jj = np.zeros((3, 3))
        jj[:2, :2] = rot(pi[2]).T
        jj[2, 2] = 1
        jmu = ji @ pi + jj @ pj
        zhat = z - h + jmu
        zhat[2] = wrap(z[2] - h[2]) + jmu[2]
        return [(si, ji), (sj, jj)], h, zhat
    si, sl = pose_slice(i), lm_slice(j)
    pi, lm = mu[si], mu[sl]
    d = lm - pi[:2]
    h = rot(pi[2]).T @ d
    ji = np.zeros((2, 3))
    ji[:, :2] = -rot(pi[2]).T
    ji[:, 2] = d_rot_t(pi[2]) @ d
    jl = rot(pi[2]).T
    zhat = z - h + ji @ pi + jl @ lm
    return [(si, ji), (sl, jl)], h, zhat


def add_measurement(omega, xi, m, mu):
    pieces, _, zhat = linearize(m, mu)
    w = m[4]
    for a, ja in pieces:
        xi[a] += ja.T @ w @ zhat
        for b, jb in pieces:
            omega[a, b] += ja.T @ w @ jb


def build_system(indices, mu):
    omega, xi = np.zeros((DIM, DIM)), np.zeros(DIM)
    for k in indices:
        add_measurement(omega, xi, MEASUREMENTS[k], mu)
    return omega, xi


def solve(mu0, iterations=10):
    """Gauss-Newton in information form: solve, relinearize, repeat."""
    history = [mu0.copy()]
    mu = mu0.copy()
    for _ in range(iterations):
        omega, xi = build_system(range(len(MEASUREMENTS)), mu)
        new = np.linalg.solve(omega, xi)
        new[[2, 5, 8]] = wrap(new[[2, 5, 8]])
        history.append(new.copy())
        done = np.abs(new - mu).max() < 1e-10
        mu = new
        if done:
            break
    return history


# ------------------------------------------------------------------ drawing helpers

def fmt_cell(v):
    return "" if abs(v) < 1e-9 else f"{v:.4g}"


def pose_polygon(p, s=0.3):
    base = np.array([[-0.7 * s, 0.9 * s], [-0.7 * s, -0.9 * s], [1.1 * s, 0.0]])
    return base @ rot(p[2]).T + p[:2]


def draw_pose(ax, p, label, offset, anchored=False, style="solid", color=TEXT, label_color=TEXT):
    ax.add_patch(Polygon(pose_polygon(p), closed=True, facecolor=BG, edgecolor=color, linewidth=2.4,
                         linestyle=style, zorder=4))
    if anchored:
        ax.add_patch(Circle(p[:2], 0.045, facecolor=color, edgecolor="none", zorder=5))
        ax.add_patch(Circle(p[:2], 0.42, facecolor="none", edgecolor=ODOM, linewidth=1.8,
                            linestyle=(0, (3, 3)), zorder=2))
    if label:
        ax.text(p[0] + offset[0], p[1] + offset[1], label, color=label_color, fontsize=20, ha="center",
                va="center", bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)


def draw_landmark(ax, xy, label, offset, edge=TEXT, label_color=TEXT):
    ax.plot([xy[0]], [xy[1]], marker=star_marker(), markersize=30, markerfacecolor=BG, markeredgecolor=edge,
            markeredgewidth=2.4, zorder=4)
    if label:
        ax.text(xy[0] + offset[0], xy[1] + offset[1], label, color=label_color, fontsize=20, ha="center",
                va="center", bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)


def edge_label(ax, xy, text, color, ha="center"):
    ax.text(xy[0], xy[1], text, color=color, fontsize=13, ha=ha, va="center",
            bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)


def style_map_axes(ax):
    ax.set_facecolor(BG)
    ax.set_xlim(-0.9, 4.2)
    ax.set_ylim(-1.0, 3.8)
    ax.set_aspect("equal")
    ax.set_xticks(range(0, 5))
    ax.set_yticks(range(-1, 4))
    ax.grid(color="#33454d", linewidth=1.0)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("x (m)", color=TEXT, fontsize=14)
    ax.set_ylabel("y (m)", color=TEXT, fontsize=14)
    for spine in ax.spines.values():
        spine.set_color(GRID)


POSE_LABEL = [r"$\mathbf{x}_0$", r"$\mathbf{x}_1$", r"$\mathbf{x}_2$"]
POSE_OFFSET = [(-0.12, -0.5), (0.45, -0.1), (0.5, 0.0)]
LM_LABEL = [r"$L_0$", r"$L_1$"]
LM_OFFSET = [(0.22, 0.32), (-0.1, 0.4)]
GHOST_L0 = None  # set in main: where x1's re-observation puts L0


def draw_map(fig, upto, mu):
    """Literal 2D view of every measurement added up to (and including) step index `upto`."""
    ax = fig.add_axes([0.07, 0.1, 0.9, 0.88])
    style_map_axes(ax)
    used = [k for s in STEPS[: upto + 1] for k in s]
    poses = sorted({0} | {m[1] for k in used for m in [MEASUREMENTS[k]] if m[0] != "prior"}
                   | {m[2] for k in used for m in [MEASUREMENTS[k]] if m[0] == "odom"})
    lms = sorted({MEASUREMENTS[k][2] for k in used if MEASUREMENTS[k][0] == "lm"})
    seen_lm = set()

    for k in used:
        kind, i, j, z, _ = MEASUREMENTS[k]
        if kind == "odom":
            a, b = mu[pose_slice(i)][:2], mu[pose_slice(j)][:2]
            ax.plot([a[0], b[0]], [a[1], b[1]], color=ODOM, linewidth=3.2, zorder=1)
            deg = np.degrees(z[2])
            text = rf"$z=({z[0]:g},\,{z[1]:g},\,{deg:g}^\circ)$"
            pos = (1.0, -0.33) if i == 0 else (1.92, 1.15)
            edge_label(ax, pos, text, ODOM, ha="center" if i == 0 else "right")
        elif kind == "lm":
            a = mu[pose_slice(i)]
            predicted = a[:2] + rot(a[2]) @ z
            target = mu[lm_slice(j)]
            conflict = j in seen_lm
            seen_lm.add(j)
            color = CONFLICT if conflict else OBS
            end = predicted if conflict else target
            ax.plot([a[0], end[0]], [a[1], end[1]], color=color, linewidth=2.8, linestyle=(0, (4, 3)), zorder=1)
            if conflict:
                ax.plot([end[0]], [end[1]], marker=star_marker(), markersize=26, markerfacecolor="none",
                        markeredgecolor=CONFLICT, markeredgewidth=2.2, linestyle="none", zorder=3)
                gap = np.linalg.norm(predicted - target)
                ax.text(3.55, 1.62, rf"$L_0$ seen again: {gap:.2f} m off", color=CONFLICT, fontsize=13,
                        ha="center", va="center", bbox=dict(facecolor=BG, edgecolor="none", pad=1.5), zorder=6)
            text = rf"$z=({z[0]:g},\,{z[1]:g})$"
            pos = {(0, 0): (1.35, 0.78), (1, 0): (2.72, 0.35), (2, 1): (1.48, 2.62)}[(i, j)]
            edge_label(ax, pos, text, color, ha="left" if (i, j) == (1, 0) else "center")

    for i in poses:
        draw_pose(ax, mu[pose_slice(i)], POSE_LABEL[i], POSE_OFFSET[i], anchored=(i == 0))
    for j in lms:
        draw_landmark(ax, mu[lm_slice(j)], LM_LABEL[j], LM_OFFSET[j])

    # legend
    ax.plot([1.95, 2.35], [-0.58, -0.58], color=ODOM, linewidth=3.2)
    ax.text(2.42, -0.58, "odometry", color=TEXT, fontsize=12, va="center")
    ax.plot([1.95, 2.35], [-0.84, -0.84], color=OBS, linewidth=2.8, linestyle=(0, (4, 3)))
    ax.text(2.42, -0.84, "landmark observation", color=TEXT, fontsize=12, va="center")


def draw_abstract_graph(ax, nodes, edges):
    ax.set_xlim(-0.9, 5.6)
    ax.set_ylim(-3.2, 2.6)
    ax.set_aspect("equal")
    ax.axis("off")
    pos = {"x0": (0.0, 0.0), "x1": (2.0, 0.0), "x2": (4.0, 0.0), "L0": (1.0, 1.6), "L1": (4.7, -1.6)}
    labels = {"x0": POSE_LABEL[0], "x1": POSE_LABEL[1], "x2": POSE_LABEL[2], "L0": LM_LABEL[0], "L1": LM_LABEL[1]}
    for kind, a, b in edges:
        (x1, y1), (x2, y2) = pos[a], pos[b]
        if kind == "odom":
            ax.plot([x1, x2], [y1, y2], color=ODOM, linewidth=3, zorder=1)
        else:
            ax.plot([x1, x2], [y1, y2], color=OBS, linewidth=2.6, linestyle=(0, (4, 3)), zorder=1)
    for n in nodes:
        x, y = pos[n]
        if n.startswith("x"):
            ax.add_patch(Polygon([(x - 0.27, y + 0.34), (x - 0.27, y - 0.34), (x + 0.38, y)], closed=True,
                                 facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))
            if n == "x0":
                ax.add_patch(Circle((x - 0.05, y), 0.05, facecolor=TEXT, edgecolor="none", zorder=4))
            ax.text(x, y - 0.65, labels[n], color=TEXT, fontsize=20, ha="center", va="top")
        else:
            ax.plot([x], [y], marker=star_marker(), markersize=30, markerfacecolor=BG, markeredgecolor=TEXT,
                    markeredgewidth=2.4, zorder=3)
            ax.text(x, y + (0.5 if y > 0 else -0.5), labels[n], color=TEXT, fontsize=20, ha="center",
                    va="bottom" if y > 0 else "top")
    ax.plot([-0.8, -0.2], [-3.0, -3.0], color=ODOM, linewidth=3)
    ax.text(0.0, -3.0, "odometry / prior", color=TEXT, fontsize=13, va="center")
    ax.plot([2.7, 3.3], [-3.0, -3.0], color=OBS, linewidth=2.6, linestyle=(0, (4, 3)))
    ax.text(3.5, -3.0, "landmark observation", color=TEXT, fontsize=13, va="center")


ROW_LABELS = ([rf"${c}_{i}$" for i in range(N_POSES) for c in ("x", "y", r"\theta")]
              + [rf"$L_{{{k}{c}}}$" for k in range(N_LM) for c in ("x", "y")])


def draw_system(ax, omega, xi, prev_omega, prev_xi, tint=None):
    n = DIM
    ax.set_xlim(-1.9, n + 4.3)
    ax.set_ylim(n + 2.2, -1.5)
    ax.set_aspect("equal")
    ax.axis("off")
    xi_x, xi_w = n + 1.2, 1.7

    for j in range(n):
        ax.text(j + 0.5, -0.3, ROW_LABELS[j], color=TEXT, fontsize=12, ha="center", va="bottom")
    for i in range(n):
        ax.text(-0.15, i + 0.5, ROW_LABELS[i], color=TEXT, fontsize=12, ha="right", va="center")

    def face(v, prev, i=None, j=None):
        if tint is not None:
            return tint(i, j)
        if abs(v - prev) > 1e-9:
            return CELL_NEW
        return CELL_ON if abs(v) > 1e-9 else BG

    for i in range(n):
        for j in range(n):
            v = omega[i, j]
            ax.add_patch(Rectangle((j, i), 1, 1, facecolor=face(v, prev_omega[i, j], i, j), edgecolor=GRID,
                                   linewidth=0.8))
            text = fmt_cell(v) if tint is None else ""
            if text:
                ax.text(j + 0.5, i + 0.5, text, color=CELL_TEXT, fontsize=8, ha="center", va="center",
                        fontweight="bold")
    for i in range(n):
        v = xi[i]
        ax.add_patch(Rectangle((xi_x, i), xi_w, 1, facecolor=face(v, prev_xi[i], i, None), edgecolor=GRID,
                               linewidth=0.8))
        text = fmt_cell(v) if tint is None else ""
        if text:
            ax.text(xi_x + xi_w / 2, i + 0.5, text, color=CELL_TEXT, fontsize=8, ha="center", va="center",
                    fontweight="bold")

    for b in BLOCKS:
        ax.plot([b, b], [0, n], color=TEXT, linewidth=2.0, zorder=5)
        ax.plot([0, n], [b, b], color=TEXT, linewidth=2.0, zorder=5)
        ax.plot([xi_x, xi_x + xi_w], [b, b], color=TEXT, linewidth=2.0, zorder=5)
    ax.plot([xi_x, xi_x], [0, n], color=TEXT, linewidth=2.0, zorder=5)
    ax.plot([xi_x + xi_w] * 2, [0, n], color=TEXT, linewidth=2.0, zorder=5)

    ax.text(n / 2, n + 0.75, r"$\Omega$", color=TEXT, fontsize=26, ha="center", va="center")
    ax.text(xi_x + xi_w / 2, n + 0.75, r"$\xi$", color=TEXT, fontsize=26, ha="center", va="center")
    if tint is None:
        ax.add_patch(Rectangle((0.0, n + 1.5), 0.6, 0.6, facecolor=CELL_NEW, edgecolor=GRID))
        ax.text(0.9, n + 1.8, "updated this step", color=TEXT, fontsize=13, va="center")
        ax.add_patch(Rectangle((6.2, n + 1.5), 0.6, 0.6, facecolor=CELL_ON, edgecolor=GRID))
        ax.text(7.1, n + 1.8, "non-zero", color=TEXT, fontsize=13, va="center")


# ------------------------------------------------------------------ figures

def edges_so_far(upto):
    edges, nodes = [], ["x0"]
    for k in [k for s in STEPS[: upto + 1] for k in s]:
        kind, i, j, _, _ = MEASUREMENTS[k]
        if kind == "odom":
            edges.append(("odom", f"x{i}", f"x{j}"))
            nodes += [f"x{i}", f"x{j}"]
        elif kind == "lm":
            edges.append(("lm", f"x{i}", f"L{j}"))
            nodes += [f"x{i}", f"L{j}"]
    return list(dict.fromkeys(nodes)), edges


def make_layout_figure():
    colors = {"pp": "#7fd1ff", "pl": "#ffd166", "ll": "#a6e3a1"}

    def tint(i, j):
        if j is None:
            return colors["pp"] if i < 9 else colors["ll"]
        pose_i, pose_j = i < 9, j < 9
        return colors["pp"] if pose_i and pose_j else colors["ll"] if not pose_i and not pose_j else colors["pl"]

    fig = plt.figure(figsize=(16, 7.4), facecolor=BG)
    ax = fig.add_axes([0.02, 0.0, 0.62, 0.98])
    zero = np.zeros((DIM, DIM))
    draw_system(ax, zero, np.zeros(DIM), zero, np.zeros(DIM), tint=tint)
    names = [(1.5, 1.5, r"$3\times3$", "pp"), (4.5, 4.5, r"$3\times3$", "pp"), (7.5, 7.5, r"$3\times3$", "pp"),
             (10.0, 10.0, r"$2\times2$", "ll"), (12.0, 12.0, r"$2\times2$", "ll"),
             (1.5, 10.0, r"$2\times3$", "pl"), (10.0, 1.5, r"$3\times2$", "pl")]
    for x, y, t, _ in names:
        ax.text(x, y, t, color=CELL_TEXT, fontsize=13, ha="center", va="center", fontweight="bold", zorder=6)
    xi_x = DIM + 1.2
    for y, t in [(1.5, r"$3\times1$"), (4.5, r"$3\times1$"), (7.5, r"$3\times1$"), (10.0, r"$2\times1$"),
                 (12.0, r"$2\times1$")]:
        ax.text(xi_x + 0.85, y, t, color=CELL_TEXT, fontsize=12, ha="center", va="center", fontweight="bold",
                zorder=6)

    leg = fig.add_axes([0.66, 0.15, 0.33, 0.7])
    leg.set_facecolor(BG)
    leg.axis("off")
    leg.set_xlim(0, 1)
    leg.set_ylim(0, 1)
    rows = [
        (colors["pp"], "pose - pose", r"$3\times3$ blocks"),
        (colors["pl"], "pose - landmark", r"$3\times2$ and $2\times3$ blocks"),
        (colors["ll"], "landmark - landmark", r"$2\times2$ blocks"),
    ]
    for k, (c, t1, t2) in enumerate(rows):
        y = 0.82 - k * 0.2
        leg.add_patch(Rectangle((0.0, y - 0.05), 0.09, 0.1, facecolor=c, edgecolor=GRID, transform=leg.transData))
        leg.text(0.13, y + 0.02, t1, color=TEXT, fontsize=16, va="center")
        leg.text(0.13, y - 0.045, t2, color=GRID, fontsize=13, va="center")
    leg.text(0.0, 0.15, "In 1D, every variable was one scalar,\nso every block was $1\\times1$.",
             color=TEXT, fontsize=14, va="center")
    fig.savefig(os.path.join(OUT_DIR, LAYOUT_FILE), dpi=140, facecolor=BG)
    plt.close(fig)


def make_solution_figure(mu0, mu):
    fig = plt.figure(figsize=(9, 7.2), facecolor=BG)
    ax = fig.add_axes([0.07, 0.1, 0.9, 0.88])
    style_map_axes(ax)
    for i in range(N_POSES):
        draw_pose(ax, mu0[pose_slice(i)], "", (0, 0), style=(0, (3, 2)), color=GRID)
    for k in range(N_LM):
        draw_landmark(ax, mu0[lm_slice(k)], "", (0, 0), edge=GRID)
    for i in range(N_POSES - 1):
        a, b = mu[pose_slice(i)][:2], mu[pose_slice(i + 1)][:2]
        ax.plot([a[0], b[0]], [a[1], b[1]], color=ODOM, linewidth=3.2, zorder=1)
    for i, k in [(0, 0), (1, 0), (2, 1)]:
        a, b = mu[pose_slice(i)][:2], mu[lm_slice(k)]
        ax.plot([a[0], b[0]], [a[1], b[1]], color=OBS, linewidth=2.8, linestyle=(0, (4, 3)), zorder=1)
    for i in range(N_POSES):
        draw_pose(ax, mu[pose_slice(i)], POSE_LABEL[i], POSE_OFFSET[i], anchored=(i == 0))
    for k in range(N_LM):
        draw_landmark(ax, mu[lm_slice(k)], LM_LABEL[k], LM_OFFSET[k])
    ax.plot([1.45, 1.85], [-0.58, -0.58], color=GRID, linewidth=2.4, linestyle=(0, (3, 2)))
    ax.text(1.92, -0.58, "initial guess (dead reckoning)", color=TEXT, fontsize=12, va="center")
    ax.plot([1.45, 1.85], [-0.84, -0.84], color=TEXT, linewidth=2.4)
    ax.text(1.92, -0.84, "solved", color=TEXT, fontsize=12, va="center")
    fig.savefig(os.path.join(OUT_DIR, SOLUTION_FILE), dpi=140, facecolor=BG)
    plt.close(fig)


# ------------------------------------------------------------------ LaTeX dump for the Markdown

def latex_cell(v, digits=3):
    v = 0.0 if abs(v) < 5 * 10 ** (-digits - 1) else v
    return f"{v:.{digits}f}".rstrip("0").rstrip(".") if v != int(v) else f"{int(v)}"


def latex_matrix(m, digits=3):
    m = np.atleast_2d(np.asarray(m, dtype=float))
    return ("\\begin{bmatrix}\n" + "\\\\\n".join("&".join(latex_cell(v, digits) for v in row) for row in m)
            + "\n\\end{bmatrix}")


def latex_partitioned(m, row_splits=(), col_splits=(), digits=3):
    """Plain bmatrix. GitHub's math renderer rejects \\hline and \\left[...\\right] arrays, so the
    split arguments are accepted for readability of the callers but intentionally not drawn."""
    return latex_matrix(m, digits)


def stacked(pieces):
    """Joint Jacobian [J_a | J_b], the index list it covers, and the split points between nodes."""
    j = np.hstack([p[1] for p in pieces])
    idx = np.concatenate([np.arange(p[0].start, p[0].stop) for p in pieces])
    splits, acc = [], 0
    for p in pieces[:-1]:
        acc += p[1].shape[1]
        splits.append(acc)
    return j, idx, splits


def dump_latex(mu0):
    np.set_printoptions(precision=4, suppress=True)
    print("mu0 =", mu0)
    for step, indices in enumerate(STEPS, 1):
        print(f"\n===== STEP {step} =====")
        for k in indices:
            m = MEASUREMENTS[k]
            pieces, h, zhat = linearize(m, mu0)
            j, idx, splits = stacked(pieces)
            d_omega = j.T @ m[4] @ j
            d_xi = j.T @ m[4] @ zhat
            print(f"-- measurement {k}: {m[0]}  z={m[3]}  h={h}  zhat={zhat}  nodes at {idx[0]}..")
            print("J =\n" + latex_partitioned(j, col_splits=splits))
            print("dOmega =\n" + latex_partitioned(d_omega, row_splits=splits, col_splits=splits))
            print("dxi =\n" + latex_partitioned(d_xi.reshape(-1, 1), row_splits=splits, digits=2))
        done = [k for s in STEPS[:step] for k in s]
        omega, xi = build_system(done, mu0)
        nonzero = np.flatnonzero(np.abs(omega).sum(axis=0) > 0)
        splits = [n for n in range(1, len(nonzero)) if nonzero[n] in BLOCKS]
        print(f"Omega over rows/cols {list(nonzero)}:\n"
              + latex_partitioned(omega[np.ix_(nonzero, nonzero)], row_splits=splits, col_splits=splits, digits=2))
        print("xi:\n" + latex_partitioned(xi[nonzero].reshape(-1, 1), row_splits=splits, digits=2))
    omega, xi = build_system(range(len(MEASUREMENTS)), mu0)
    print("\nFINAL xi =\n" + latex_partitioned(xi.reshape(-1, 1), row_splits=BLOCKS[1:-1], digits=2))
    hist = solve(mu0)
    for k, h in enumerate(hist):
        print(f"iter {k}: {np.round(h, 4)}")
    sigma = np.array([[0.010, 0.004, 0], [0.004, 0.010, 0], [0, 0, 0.0025]])
    print("\ncorrelated Sigma inverse =\n" + latex_matrix(np.linalg.inv(sigma), 2))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    mu0 = initial_guess()
    if "--latex" in sys.argv:
        dump_latex(mu0)
        return

    previous = (np.zeros((DIM, DIM)), np.zeros(DIM))
    applied = []
    for step, ((world_file, system_file), indices) in enumerate(zip(STEP_FILES, STEPS)):
        applied += indices
        omega, xi = build_system(applied, mu0)

        fig = plt.figure(figsize=(9, 7.2), facecolor=BG)
        draw_map(fig, step, mu0)
        fig.savefig(os.path.join(OUT_DIR, world_file), dpi=140, facecolor=BG)
        plt.close(fig)

        nodes, edges = edges_so_far(step)
        fig = plt.figure(figsize=(16, 7.4), facecolor=BG)
        draw_abstract_graph(fig.add_axes([0.01, 0.02, 0.34, 0.96]), nodes, edges)
        draw_system(fig.add_axes([0.36, 0.0, 0.63, 0.98]), omega, xi, previous[0], previous[1])
        fig.savefig(os.path.join(OUT_DIR, system_file), dpi=140, facecolor=BG)
        plt.close(fig)
        previous = (omega, xi)

    make_layout_figure()

    # Sanity checks on numbers quoted in the Markdown.
    omega, xi = build_system(range(len(MEASUREMENTS)), mu0)
    assert np.linalg.eigvalsh(omega).min() > 0
    assert np.allclose(omega[:3, :3], [[300, 0, -100], [0, 225, 275], [-100, 275, 1525]])
    assert np.allclose(omega[lm_slice(0), lm_slice(0)], np.diag([125, 125]))
    assert np.allclose(omega[pose_slice(2), pose_slice(2)], [[125, 0, -37.5], [0, 200, -150], [-37.5, -150, 681.25]])
    assert np.allclose(xi, [-500, -25, -403.32, -97.39, -107.92, 1085.60, 214.21, -235.62, 610.54, 361.77, -17.08,
                            21.40, 385.62], atol=5e-3)
    sigma = np.array([[0.010, 0.004, 0], [0.004, 0.010, 0], [0, 0, 0.0025]])
    assert np.allclose(np.linalg.inv(sigma), [[119.05, -47.62, 0], [-47.62, 119.05, 0], [0, 0, 400]], atol=5e-3)
    assert np.allclose(W_PRIOR, np.diag([100, 100, 400])) and np.allclose(W_LM, np.diag([100, 25]))

    hist = solve(mu0)
    assert np.abs(hist[-1] - hist[-2]).max() < 1e-9
    first, final = np.round(hist[1], 4), np.round(hist[-1], 4)
    assert np.allclose(first[3:], [2.0151, -0.0234, 1.5612, 2.0295, 1.4766, 1.5612, 2.9849, 1.0936, 0.5439, 2.9910],
                       atol=1e-4)
    assert np.allclose(final[3:], [2.0147, -0.0234, 1.5610, 2.0294, 1.4765, 1.5610, 2.9853, 1.0938, 0.5441, 2.9911],
                       atol=1e-4)
    assert np.abs(np.round(hist[2], 4) - final).max() < 1e-4
    make_solution_figure(mu0, hist[-1])
    print("Generated", 2 * len(STEPS) + 2, "figures in", OUT_DIR, "after", len(hist) - 1, "iterations")


if __name__ == "__main__":
    main()
