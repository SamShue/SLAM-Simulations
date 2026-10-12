"""Generate the figures (and the numbers) for graphslam_loop_closure.md and graphslam_sparse_solvers.md.

Run from anywhere:  python Graph_SLAM/generate_loop_and_sparse_figures.py
Print the numbers quoted in the Markdown:  python Graph_SLAM/generate_loop_and_sparse_figures.py --numbers
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle, Polygon, Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loop_example_model as M  # noqa: E402
from generate_2d_figures import pose_polygon  # noqa: E402
from generate_step_figures import (  # noqa: E402
    BG, CELL_NEW, CELL_ON, CELL_TEXT, CONFLICT, GRID, ODOM, OBS, TEXT, star_marker,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Documents", "images")
TRUTH = "#8fa3ad"


def save(fig, name):
    fig.savefig(os.path.join(OUT_DIR, name), dpi=140, facecolor=BG)
    plt.close(fig)


# ------------------------------------------------------------------ top-down maps

def map_axes(fig, rect=(0.07, 0.07, 0.91, 0.91), lim=(-1.7, 5.4)):
    ax = fig.add_axes(list(rect))
    ax.set_facecolor(BG)
    ax.set_xlim(*lim)
    ax.set_ylim(*lim)
    ax.set_aspect("equal")
    ax.set_xticks(range(-1, 6))
    ax.set_yticks(range(-1, 6))
    ax.grid(color="#33454d", linewidth=1.0)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("x (m)", color=TEXT, fontsize=14)
    ax.set_ylabel("y (m)", color=TEXT, fontsize=14)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    return ax


def draw_triangle(ax, p, s, edge=TEXT, lw=2.2, z=4, face=BG):
    ax.add_patch(Polygon(pose_polygon(np.asarray(p, float), s), closed=True, facecolor=face, edgecolor=edge,
                         linewidth=lw, zorder=z))


def draw_star(ax, xy, edge=TEXT, size=22, filled=False, z=4):
    ax.plot([xy[0]], [xy[1]], marker=star_marker(), markersize=size, markerfacecolor=BG if not filled else edge,
            markeredgecolor=edge, markeredgewidth=2.0, linestyle="none", zorder=z)


def label_at(ax, xy, text, color=TEXT, size=16, ha="center", va="center", z=7):
    ax.text(xy[0], xy[1], text, color=color, fontsize=size, ha=ha, va=va,
            bbox=dict(facecolor=BG, edgecolor="none", pad=1.2), zorder=z)


def pose_label_offset(p, dist=0.5):
    v = np.asarray(p[:2]) - np.array([2.0, 2.0])
    return dist * v / max(np.linalg.norm(v), 1e-9)


def draw_truth(ax):
    path = np.vstack([M.TRUE_POSES[:, :2]])
    ax.plot(path[:, 0], path[:, 1], color=TRUTH, linewidth=2.0, linestyle=(0, (5, 4)), zorder=1)
    for lm in M.TRUE_LM:
        draw_star(ax, lm, edge=TRUTH, size=20, z=2)


def draw_estimate(ax, mu, indices, color=ODOM, tri=0.2, labels=True, obs_color=OBS, face=BG, edge=TEXT, lw=2.6):
    poses = [mu[M.pose_slice(i)] for i in range(M.N_POSES)]
    xy = np.array([p[:2] for p in poses])
    ax.plot(xy[:, 0], xy[:, 1], color=color, linewidth=lw, zorder=2)
    for k in indices:
        kind, i, j, _, _ = M.MEASUREMENTS[k]
        if kind == "lm":
            lm = mu[M.lm_slice(j)]
            ax.plot([poses[i][0], lm[0]], [poses[i][1], lm[1]], color=obs_color, linewidth=1.8,
                    linestyle=(0, (4, 3)), zorder=1)
    for i, p in enumerate(poses):
        draw_triangle(ax, p, tri, edge=edge, face=face)
        if labels:
            label_at(ax, p[:2] + pose_label_offset(p), rf"$x_{i}$", size=14)
    for j in range(M.N_LM):
        draw_star(ax, mu[M.lm_slice(j)], edge=edge)


def legend_map(ax, items, x=2.7, y0=-0.85, dy=0.32):
    for k, (kind, color, text) in enumerate(items):
        y = y0 - k * dy
        if kind == "dash":
            ax.plot([x, x + 0.45], [y, y], color=color, linewidth=2.2, linestyle=(0, (4, 3)))
        else:
            ax.plot([x, x + 0.45], [y, y], color=color, linewidth=2.6)
        ax.text(x + 0.55, y, text, color=TEXT, fontsize=12, va="center")


def make_drift_map(mu0):
    fig = plt.figure(figsize=(9, 9), facecolor=BG)
    ax = map_axes(fig)
    draw_truth(ax)
    draw_estimate(ax, mu0, M.LOCAL)
    e8 = mu0[M.pose_slice(8)][:2]
    ax.plot([0, e8[0]], [0, e8[1]], color=CONFLICT, linewidth=3.0, zorder=3)
    gap = np.linalg.norm(e8)
    label_at(ax, (1.2, -1.0), rf"$x_8$ is {gap:.2f} m from the true start", color=CONFLICT, size=13)
    legend_map(ax, [("dash", TRUTH, "true path"), ("line", ODOM, "dead-reckoned path"),
                    ("dash", OBS, "landmark observation")], x=2.6, y0=-0.7)
    save(fig, "loop_world_drift.png")


def make_closure_map(mu0):
    fig = plt.figure(figsize=(9, 9), facecolor=BG)
    ax = map_axes(fig)
    draw_truth(ax)
    draw_estimate(ax, mu0, M.LOCAL)
    e8 = mu0[M.pose_slice(8)][:2]
    ax.plot([0, e8[0]], [0, e8[1]], color=CONFLICT, linewidth=3.4, linestyle=(0, (3, 2)), zorder=5)
    z = M.MEASUREMENTS[M.POSE_CLOSURE][3]
    label_at(ax, (1.25, -0.9), "estimate: 0.57 m apart", color=CONFLICT, size=13)
    label_at(ax, (1.25, -1.25), rf"closure measures: {np.linalg.norm(z[:2]):.2f} m apart", color=CONFLICT, size=13)
    legend_map(ax, [("dash", CONFLICT, "loop closure between $x_8$ and $x_0$"),
                    ("line", ODOM, "dead-reckoned path"), ("dash", TRUTH, "true path")], x=2.6, y0=-0.7)
    save(fig, "loop_world_closure.png")


def make_solved_map(mu0, mu_solved):
    fig = plt.figure(figsize=(9, 9), facecolor=BG)
    ax = map_axes(fig)
    draw_truth(ax)
    xy0 = np.array([mu0[M.pose_slice(i)][:2] for i in range(M.N_POSES)])
    ax.plot(xy0[:, 0], xy0[:, 1], color=CONFLICT, linewidth=2.2, linestyle=(0, (4, 3)), zorder=2)
    for i in range(M.N_POSES):
        draw_triangle(ax, mu0[M.pose_slice(i)], 0.17, edge=CONFLICT, lw=1.8, z=3)
    draw_estimate(ax, mu_solved, M.WITH_POSE_CLOSURE, color=ODOM, tri=0.2, labels=True)
    legend_map(ax, [("dash", TRUTH, "true path"), ("dash", CONFLICT, "dead reckoning (before)"),
                    ("line", ODOM, "after solving with the loop closure")], x=2.0, y0=-0.7)
    save(fig, "loop_world_solved.png")


def make_landmark_closure_map(mu0):
    fig = plt.figure(figsize=(9, 9), facecolor=BG)
    ax = map_axes(fig)
    lm = mu0[M.lm_slice(0)]
    true8 = M.TRUE_POSES[8]
    est8 = mu0[M.pose_slice(8)]
    radius = np.linalg.norm(true8[:2] - M.TRUE_LM[0])
    ax.add_patch(Circle(M.TRUE_LM[0], radius, facecolor="none", edgecolor=OBS, linewidth=1.8,
                        linestyle=(0, (4, 3)), zorder=1))
    draw_star(ax, M.TRUE_LM[0], edge=TEXT)
    label_at(ax, M.TRUE_LM[0] + np.array([0.0, 0.45]), r"$L_0$")
    draw_triangle(ax, true8, 0.22, edge=TRUTH)
    draw_triangle(ax, est8, 0.22, edge=CONFLICT)
    label_at(ax, true8[:2] + np.array([-0.5, -0.1]), "true $x_8$", color=TRUTH, size=13)
    label_at(ax, est8[:2] + np.array([0.75, -0.25]), "dead-reckoned $x_8$", color=CONFLICT, size=13)
    for p, c in [(true8, TRUTH), (est8, CONFLICT)]:
        ax.plot([p[0], lm[0]], [p[1], lm[1]], color=c, linewidth=1.8, linestyle=(0, (4, 3)), zorder=1)
    ax.text(2.0, 4.55, "poses on this circle see $L_0$ at the same distance", color=OBS, fontsize=13,
            ha="center", va="center", bbox=dict(facecolor=BG, edgecolor="none", pad=1.5))
    save(fig, "loop_landmark_closure.png")


# ------------------------------------------------------------------ pattern drawings

LOG_CMAP = LinearSegmentedColormap.from_list("mag", [BG, "#2f6f8f", "#7fd1ff", "#ffffff"])


def draw_pattern(ax, mat, new=None, blocks=None, labels=None, mode="pattern", tol=1e-9, label_size=11,
                 lines=2.0, log_range=(-6.0, 0.0)):
    """Matrix pattern. mode 'pattern': non-zero light, `new` cells yellow. mode 'log': log10 magnitude."""
    n = mat.shape[0]
    ax.set_xlim(-0.2, n + 0.2)
    ax.set_ylim(n + 0.2, -0.2)
    ax.set_aspect("equal")
    ax.axis("off")
    scale = np.abs(mat).max() if mode == "log" else 1.0
    for i in range(n):
        for j in range(n):
            v = mat[i, j]
            if mode == "log":
                a = abs(v) / scale
                t = 0.0 if a < 1e-12 else (np.log10(a) - log_range[0]) / (log_range[1] - log_range[0])
                face = LOG_CMAP(min(max(t, 0.0), 1.0))
            elif new is not None and new[i, j]:
                face = CELL_NEW
            else:
                face = CELL_ON if abs(v) > tol else BG
            ax.add_patch(Rectangle((j, i), 1, 1, facecolor=face, edgecolor=GRID, linewidth=0.35))
    for b in blocks or [0, n]:
        ax.plot([b, b], [0, n], color=TEXT, linewidth=lines, zorder=5)
        ax.plot([0, n], [b, b], color=TEXT, linewidth=lines, zorder=5)
    if labels:
        b = blocks
        for k, name in enumerate(labels):
            c = (b[k] + b[k + 1]) / 2
            ax.text(c, -0.5, name, color=TEXT, fontsize=label_size, ha="center", va="bottom")
            ax.text(-0.4, c, name, color=TEXT, fontsize=label_size, ha="right", va="center")


NODE_LABELS = [rf"$x_{i}$" for i in range(M.N_POSES)] + [rf"$L_{k}$" for k in range(M.N_LM)]


def ring_positions(radius=3.0):
    pos = {}
    for k in range(M.N_POSES):
        phi = np.radians(90 - 40 * k)
        pos[f"x{k}"] = (radius * np.cos(phi), radius * np.sin(phi), phi - np.pi / 2)
    pos["L0"] = (0.0, 1.3, 0.0)
    pos["L1"] = (1.3, -1.2, 0.0)
    return pos


def draw_ring_graph(ax, indices, highlight=None):
    ax.set_xlim(-4.4, 4.4)
    ax.set_ylim(-4.5, 4.4)
    ax.set_aspect("equal")
    ax.axis("off")
    pos = ring_positions()
    for k in indices:
        kind, i, j, _, _ = M.MEASUREMENTS[k]
        if kind == "odom" and i == M.N_POSES - 1 and j == 0:
            continue
        if kind == "odom":
            a, b = pos[f"x{i}"], pos[f"x{j}"]
            ax.plot([a[0], b[0]], [a[1], b[1]], color=ODOM, linewidth=3, zorder=1)
        elif kind == "lm":
            a, b = pos[f"x{i}"], pos[f"L{j}"]
            ax.plot([a[0], b[0]], [a[1], b[1]], color=OBS, linewidth=2.2, linestyle=(0, (4, 3)), zorder=1)
    if highlight is not None:
        kind, i, j, _, _ = M.MEASUREMENTS[highlight]
        a = pos[f"x{i}"]
        b = pos[f"x{j}"] if kind == "odom" else pos[f"L{j}"]
        ax.plot([a[0], b[0]], [a[1], b[1]], color=CONFLICT, linewidth=4.2, linestyle=(0, (3, 2)), zorder=6)
    for name, (x, y, th) in pos.items():
        if name.startswith("x"):
            ax.add_patch(Polygon(pose_polygon(np.array([x, y, th]), 0.36), closed=True, facecolor=BG,
                                 edgecolor=TEXT, linewidth=2.2, zorder=4))
            r = np.array([x, y]) / max(np.hypot(x, y), 1e-9)
            ax.text(x + 0.72 * r[0], y + 0.72 * r[1], rf"$x_{name[1:]}$", color=TEXT, fontsize=15, ha="center",
                    va="center")
        else:
            ax.plot([x], [y], marker=star_marker(), markersize=26, markerfacecolor=BG, markeredgecolor=TEXT,
                    markeredgewidth=2.2, linestyle="none", zorder=4)
            ax.text(x, y + 0.65, rf"$L_{name[1:]}$", color=TEXT, fontsize=15, ha="center", va="center")


def pattern_legend(ax, n, items):
    x = 0.0
    for color, text, w in items:
        ax.add_patch(Rectangle((x, n + 1.2), 1.0, 1.0, facecolor=color, edgecolor=GRID))
        ax.text(x + 1.5, n + 1.7, text, color=TEXT, fontsize=12, va="center")
        x += w
    ax.set_ylim(n + 2.6, -0.2)


def make_system_figure(indices, prev_indices, mu0, name, highlight=None):
    omega, _ = M.build_system(indices, mu0)
    prev, _ = M.build_system(prev_indices, mu0)
    new = (np.abs(omega - prev) > 1e-9)
    fig = plt.figure(figsize=(16, 7.4), facecolor=BG)
    draw_ring_graph(fig.add_axes([0.01, 0.02, 0.36, 0.96]), indices, highlight)
    ax = fig.add_axes([0.40, 0.0, 0.59, 0.94])
    draw_pattern(ax, omega, new=new, blocks=M.NODE_START, labels=NODE_LABELS)
    pattern_legend(ax, M.DIM, [(CELL_NEW, "updated in this step", 11.0), (CELL_ON, "non-zero", 8.0)])
    save(fig, name)


def make_error_chart(series, filename):
    """Position and heading error against pose for each (label, color, estimate, marker). The y ranges are
    fixed so that the two error charts in the loop-closure document can be compared directly."""
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5.6), facecolor=BG)
    for ax, which, ylabel, scale, top in [(a, 0, "position error (m)", 1.0, 0.62),
                                          (b, 1, "heading error (degrees)", 180 / np.pi, 13.0)]:
        ax.set_facecolor(BG)
        for label, color, mu, mk in series:
            err = M.pose_errors(mu)[which] * scale
            ax.plot(range(M.N_POSES), err, color=color, marker=mk, linewidth=2.6, markersize=8, label=label)
        ax.set_ylim(-0.03 * top, top)
        ax.set_xticks(range(M.N_POSES))
        ax.set_xticklabels([rf"$x_{i}$" for i in range(M.N_POSES)], color=TEXT, fontsize=13)
        ax.tick_params(colors=TEXT, labelsize=13)
        ax.set_ylabel(ylabel, color=TEXT, fontsize=14)
        ax.grid(color="#33454d", linewidth=1.0)
        for spine in ax.spines.values():
            spine.set_color(GRID)
    a.legend(facecolor=BG, edgecolor=GRID, labelcolor=TEXT, fontsize=12, loc="upper left")
    fig.tight_layout()
    save(fig, filename)


# ------------------------------------------------------------------ sparse-solver figures

def caption(fig, x, text, y=0.04, size=14, color=TEXT):
    fig.text(x, y, text, color=color, fontsize=size, ha="center", va="center")


def make_omega_inverse(omega):
    inv = np.linalg.inv(omega)
    fig = plt.figure(figsize=(16, 7.6), facecolor=BG)
    a1 = fig.add_axes([0.04, 0.12, 0.44, 0.78])
    a2 = fig.add_axes([0.52, 0.12, 0.44, 0.78])
    draw_pattern(a1, omega, blocks=M.NODE_START, labels=NODE_LABELS, label_size=10)
    draw_pattern(a2, inv, blocks=M.NODE_START, labels=NODE_LABELS, mode="log", label_size=10)
    dens = lambda m: 100 * (np.abs(m) > 1e-9).mean()
    caption(fig, 0.26, rf"$\Omega$: {dens(omega):.0f}% of entries are non-zero")
    caption(fig, 0.74, rf"$\Omega^{{-1}}$: {dens(inv):.0f}% non-zero (brighter = larger)")
    save(fig, "sparse_omega_inverse.png")


def star_example():
    omega = np.array([[4.0, 1, 1, 1], [1, 4, 0, 0], [1, 0, 4, 0], [1, 0, 0, 4]])
    xi = np.array([1.0, 2.0, 3.0, 4.0])
    return omega, xi


def make_star_figure():
    omega, _ = star_example()
    orders = [([0, 1, 2, 3], "hub first"), ([1, 2, 3, 0], "hub last")]
    fig = plt.figure(figsize=(16, 8.4), facecolor=BG)
    for col, (order, title) in enumerate(orders):
        x0 = 0.03 + col * 0.49
        gax = fig.add_axes([x0, 0.50, 0.44, 0.44])
        gax.set_xlim(-2.4, 2.4)
        gax.set_ylim(-2.2, 2.2)
        gax.set_aspect("equal")
        gax.axis("off")
        names = {0: "hub", 1: "a", 2: "b", 3: "c"}
        pos = {0: (0, 0), 1: (-1.7, 1.3), 2: (1.7, 1.3), 3: (0, -1.7)}
        for leaf in (1, 2, 3):
            gax.plot([0, pos[leaf][0]], [0, pos[leaf][1]], color=ODOM, linewidth=3, zorder=1)
        if col == 0:
            for a, b in [(1, 2), (2, 3), (1, 3)]:
                gax.plot([pos[a][0], pos[b][0]], [pos[a][1], pos[b][1]], color=CONFLICT, linewidth=3,
                         linestyle=(0, (4, 3)), zorder=1)
        for v, (x, y) in pos.items():
            gax.add_patch(Circle((x, y), 0.36, facecolor=BG, edgecolor=TEXT, linewidth=2.4, zorder=3))
            gax.text(x, y, "h" if v == 0 else names[v], color=TEXT, fontsize=18, ha="center", va="center", zorder=4)
        l = np.linalg.cholesky(omega[np.ix_(order, order)])
        pattern = np.abs(l) > 1e-9
        orig = np.abs(np.tril(omega[np.ix_(order, order)])) > 1e-9
        ax = fig.add_axes([x0 + 0.07, 0.07, 0.30, 0.38])
        ax.set_xlim(-1.2, 4.2)
        ax.set_ylim(4.2, -0.9)
        ax.set_aspect("equal")
        ax.axis("off")
        labels = ["h" if o == 0 else names[o] for o in order]
        for i in range(4):
            ax.text(-0.15, i + 0.5, labels[i], color=TEXT, fontsize=15, ha="right", va="center")
            ax.text(i + 0.5, -0.2, labels[i], color=TEXT, fontsize=15, ha="center", va="bottom")
            for j in range(4):
                if j > i:
                    face = BG
                elif pattern[i, j] and not orig[i, j]:
                    face = CELL_NEW
                elif pattern[i, j]:
                    face = CELL_ON
                else:
                    face = BG
                ax.add_patch(Rectangle((j, i), 1, 1, facecolor=face, edgecolor=GRID, linewidth=0.8))
                if j <= i and pattern[i, j]:
                    ax.text(j + 0.5, i + 0.5, f"{l[i, j]:.2f}", color=CELL_TEXT, fontsize=13, ha="center",
                            va="center", fontweight="bold")
        fig.text(x0 + 0.22, 0.965, title, color=TEXT, fontsize=20, ha="center", va="center")
        fig.text(x0 + 0.22, 0.025, f"{int(pattern.sum())} non-zeros in $L$"
                 + ("  (yellow = fill-in)" if col == 0 else "  (no fill-in)"), color=TEXT, fontsize=14,
                 ha="center", va="center")
    save(fig, "sparse_fill_star.png")


def make_cholesky_orders(mu0):
    om, _ = M.build_system(M.WITH_POSE_CLOSURE, mu0)
    adj = M.scalar_adjacency(om)
    panels = [
        ("state-vector order\n(all poses, then the landmarks)", list(range(M.DIM))),
        ("order of first appearance\n(each landmark after the pose that first sees it)", M.appearance_order(M.WITH_POSE_CLOSURE)),
        ("minimum-degree order\n(a fill-reducing heuristic)", M.min_degree_order(adj)),
    ]
    fig = plt.figure(figsize=(18, 7.4), facecolor=BG)
    counts = []
    for k, (text, order) in enumerate(panels):
        ax = fig.add_axes([0.01 + k * 0.33, 0.20, 0.31, 0.78])
        l = np.abs(M.cholesky_pattern(om, order)).astype(float)
        orig = np.abs(np.tril(om[np.ix_(order, order)])) > 1e-9
        fill = (l > 0) & ~orig
        draw_pattern(ax, l, new=fill, blocks=None, lines=1.2)
        counts.append(int((l > 0).sum()))
        caption(fig, 0.01 + k * 0.33 + 0.155, f"{text}\n{counts[-1]} non-zeros in $L$ ({int(fill.sum())} of them fill-in)",
                y=0.09, size=13)
    save(fig, "sparse_cholesky_orders.png")
    return counts


def schur_blocks(omega, xi):
    p = slice(0, 3 * M.N_POSES)
    l = slice(3 * M.N_POSES, M.DIM)
    opp, opl, olp, oll = omega[p, p], omega[p, l], omega[l, p], omega[l, l]
    inv_ll = np.linalg.inv(oll)
    s = opp - opl @ inv_ll @ olp
    rhs = xi[p] - opl @ inv_ll @ xi[l]
    return s, rhs, opl, inv_ll, olp, oll


def new_block_mask(s, omega_pp, tol=1e-9):
    """Cells of S that sit in a 3x3 pose block which was entirely zero in the original matrix."""
    n = s.shape[0]
    mask = np.zeros((n, n), bool)
    count = 0
    for a in range(n // 3):
        for b in range(n // 3):
            blk_s = s[3 * a:3 * a + 3, 3 * b:3 * b + 3]
            blk_o = omega_pp[3 * a:3 * a + 3, 3 * b:3 * b + 3]
            if np.abs(blk_s).max() > tol and np.abs(blk_o).max() <= tol:
                mask[3 * a:3 * a + 3, 3 * b:3 * b + 3] = np.abs(blk_s) > tol
                count += 1
    return mask, count


def make_schur_figure(mu0):
    om, xi = M.build_system(M.WITH_LANDMARK_CLOSURE, mu0)
    s, _, _, _, _, _ = schur_blocks(om, xi)
    n_p = 3 * M.N_POSES
    new, n_new = new_block_mask(s, om[:n_p, :n_p])
    fig = plt.figure(figsize=(16, 7.6), facecolor=BG)
    a1 = fig.add_axes([0.04, 0.12, 0.44, 0.78])
    a2 = fig.add_axes([0.52, 0.12, 0.44, 0.78])
    draw_pattern(a1, om, blocks=M.NODE_START, labels=NODE_LABELS, label_size=10)
    a1.plot([n_p, n_p], [0, M.DIM], color=OBS, linewidth=3.2, zorder=6)
    a1.plot([0, M.DIM], [n_p, n_p], color=OBS, linewidth=3.2, zorder=6)
    draw_pattern(a2, s, new=new, blocks=M.NODE_START[: M.N_POSES + 1], labels=NODE_LABELS[: M.N_POSES], label_size=10)
    caption(fig, 0.26, rf"$\Omega$ when $x_8$ re-observes $L_0$: {M.DIM} variables")
    caption(fig, 0.74, f"after eliminating the landmarks: {n_p} variables, {n_new} new pose-pose blocks (yellow)")
    save(fig, "sparse_schur.png")
    return n_new


# ------------------------------------------------------------------ numbers for the Markdown

def fmt(m, d=2):
    m = np.atleast_2d(m)
    def c(v):
        v = 0.0 if abs(v) < 5 * 10 ** (-d - 1) else v
        return f"{v:.{d}f}".rstrip("0").rstrip(".")
    return "\\begin{bmatrix}\n" + "\\\\\n".join("&".join(c(v) for v in r) for r in m) + "\n\\end{bmatrix}"


def print_numbers(mu0):
    np.set_printoptions(precision=3, suppress=True, linewidth=170)
    print("true-vs-dead-reckoned pose errors", np.round(M.pose_errors(mu0)[0], 3), np.round(np.degrees(M.pose_errors(mu0)[1]), 1))
    for name, idx in [("local", M.LOCAL), ("pose closure", M.WITH_POSE_CLOSURE), ("landmark closure", M.WITH_LANDMARK_CLOSURE)]:
        h = M.solve(mu0, idx)
        e, hd = M.pose_errors(h[-1])
        print(name, "iters", len(h) - 1, "max move", round(float(np.abs(h[-1] - mu0).max()), 3), "pos", np.round(e, 3),
              "head(deg)", np.round(np.degrees(hd), 1))
    h = M.solve(mu0, M.WITH_POSE_CLOSURE)[-1]
    moved = [np.linalg.norm(h[M.pose_slice(i)][:2] - mu0[M.pose_slice(i)][:2]) for i in range(M.N_POSES)]
    print("poses moved by the solve", np.round(moved, 3))
    z = M.MEASUREMENTS[M.POSE_CLOSURE][3]
    print("closure z", z, "norm", np.linalg.norm(z[:2]))
    _, h_closure, _ = M.linearize(M.MEASUREMENTS[M.POSE_CLOSURE], mu0)
    print("closure predicted h(mu0)", h_closure, " residual z-h", z - h_closure, "(heading deg", np.degrees((z - h_closure)[2]), ")")
    print("x8 dead-reckoned", mu0[M.pose_slice(8)])
    om, xi = M.build_system(M.WITH_POSE_CLOSURE, mu0)
    n = M.DIM
    print("n", n, "nnz Omega (full)", int((np.abs(om) > 1e-9).sum()), "density", 100 * (np.abs(om) > 1e-9).mean())
    inv = np.linalg.inv(om)
    print("inverse density", 100 * (np.abs(inv) > 1e-9).mean())
    print("local density", 100 * (np.abs(M.build_system(M.LOCAL, mu0)[0]) > 1e-9).mean())
    print("Omega[x0,x8] block (closure):\n" + fmt(om[M.pose_slice(0), M.pose_slice(8)], 1))
    adj = M.scalar_adjacency(om)
    for oname, o in [("state order", list(range(n))), ("first appearance", M.appearance_order(M.WITH_POSE_CLOSURE)),
                     ("landmarks first", list(range(27, 31)) + list(range(27))),
                     ("min degree", M.min_degree_order(adj))]:
        print(oname, "nnz(L) with closure", int(M.cholesky_pattern(om, o).sum()))
    om_local, _ = M.build_system(M.LOCAL, mu0)
    print("first appearance, no closure nnz(L)", int(M.cholesky_pattern(om_local, M.appearance_order(M.LOCAL)).sum()),
          " state order, no closure", int(M.cholesky_pattern(om_local, list(range(n))).sum()))
    omega_l, xi_l = M.build_system(M.WITH_LANDMARK_CLOSURE, mu0)
    s, rhs, opl, inv_ll, olp, oll = schur_blocks(omega_l, xi_l)
    print("L0 block of Omega (landmark closure):\n" + fmt(oll[:2, :2], 1))
    cross = -opl[M.pose_slice(0), 0:2] @ inv_ll[:2, :2] @ olp[0:2, M.pose_slice(8)]
    print("Schur term x0-x8:\n" + fmt(cross, 1), "\nS[x0,x8]:\n" + fmt(s[M.pose_slice(0), M.pose_slice(8)], 1))
    print("new pose-pose blocks after elimination:", new_block_mask(s, omega_l[:27, :27])[1])
    print("Schur max abs diff of solution:",
          np.abs(np.linalg.solve(omega_l, xi_l)[:27] - np.linalg.solve(s, rhs)).max())
    omega_s, xi_s = star_example()
    for order in ([0, 1, 2, 3], [1, 2, 3, 0]):
        l = np.linalg.cholesky(omega_s[np.ix_(order, order)])
        print("star order", order, "\n" + fmt(l, 3))
        y = np.linalg.solve(l, xi_s[order])
        mu = np.linalg.solve(l.T, y)
        print("  y", np.round(y, 3), "mu", np.round(mu, 3), "check", np.allclose(omega_s[np.ix_(order, order)] @ mu, xi_s[order]))
    for r in M.scaling_rows():
        print(r)


def assert_bearing_side_note(mu0):
    """Numbers quoted in the 'would measuring bearing help?' side note of graphslam_loop_closure.md."""
    base = len(M.MEASUREMENTS)

    def x8_error(extra, reweight=1.0):
        saved = list(M.MEASUREMENTS)
        try:
            k = M.LANDMARK_CLOSURE
            kind, i, j, z, w = M.MEASUREMENTS[k]
            M.MEASUREMENTS[k] = (kind, i, j, z, w * reweight)
            M.MEASUREMENTS.extend(extra)
            ids = M.LOCAL + [k] + list(range(base, base + len(extra)))
            return M.pose_errors(M.solve(mu0, ids)[-1])[0][8]
        finally:
            M.MEASUREMENTS[:] = saved

    z = M.MEASUREMENTS[M.LANDMARK_CLOSURE][3]
    assert abs(np.hypot(*z) - 2.50) < 0.005 and abs(np.degrees(np.arctan2(z[1], z[0])) - 37.7) < 0.05
    assert abs(x8_error([]) - 0.51) < 0.005
    assert abs(x8_error([], reweight=100.0) - 0.52) < 0.005
    z1 = M.rot(M.TRUE_POSES[8][2]).T @ (M.TRUE_LM[1] - M.TRUE_POSES[8][:2]) + np.array([0.02, -0.02])
    assert abs(x8_error([("lm", 8, 1, z1, M.W_LM)]) - 0.45) < 0.005
    compass = ("prior", 8, None, np.array([0.0, 0.0, 0.01]), np.diag([0.0, 0.0, 1 / 0.05 ** 2]))
    assert abs(x8_error([compass]) - 0.15) < 0.005


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    mu0 = M.initial_guess()
    if "--numbers" in sys.argv:
        print_numbers(mu0)
        return

    solved_local = M.solve(mu0, M.LOCAL)[-1]
    solved_pose = M.solve(mu0, M.WITH_POSE_CLOSURE)[-1]

    make_drift_map(mu0)
    make_closure_map(mu0)
    make_solved_map(mu0, solved_pose)
    make_landmark_closure_map(mu0)
    solved_landmark = M.solve(mu0, M.WITH_LANDMARK_CLOSURE)[-1]
    make_error_chart([("dead reckoning", CONFLICT, mu0, "o"), ("solved, no loop closure", TRUTH, solved_local, "s"),
                      ("solved, with loop closure", ODOM, solved_pose, "D")], "loop_error.png")
    make_error_chart([("dead reckoning", CONFLICT, mu0, "o"), ("solved, no loop closure", TRUTH, solved_local, "s"),
                      ("solved, $x_8$ re-observes $L_0$", OBS, solved_landmark, "^"),
                      ("solved, place-recognition closure", ODOM, solved_pose, "D")], "loop_error_landmark.png")
    make_system_figure(M.LOCAL, [0], mu0, "loop_system_local.png")
    make_system_figure(M.WITH_POSE_CLOSURE, M.LOCAL, mu0, "loop_system_closure.png", highlight=M.POSE_CLOSURE)

    omega, _ = M.build_system(M.WITH_POSE_CLOSURE, mu0)
    make_omega_inverse(omega)
    make_star_figure()
    counts = make_cholesky_orders(mu0)
    n_new_blocks = make_schur_figure(mu0)

    # Numbers quoted in the Markdown.
    dr_pos = M.pose_errors(mu0)[0]
    assert abs(dr_pos[8] - 0.573) < 5e-4
    assert M.pose_errors(solved_pose)[0].max() < 0.07 and abs(M.pose_errors(solved_pose)[0][8] - 0.022) < 5e-4
    assert abs(M.pose_errors(solved_local)[0][8] - 0.540) < 5e-4
    assert counts == [262, 204, 194], counts
    moved = [np.linalg.norm(solved_pose[M.pose_slice(i)][:2] - mu0[M.pose_slice(i)][:2]) for i in range(M.N_POSES)]
    assert list(np.round(M.pose_errors(mu0)[0], 2)) == [0, 0.03, 0.07, 0.12, 0.25, 0.29, 0.46, 0.4, 0.57]
    assert list(np.round(np.degrees(M.pose_errors(mu0)[1]), 1)) == [0, 1.7, 2.6, 4.6, 6.3, 7.4, 9.5, 10.6, 12.0]
    assert list(np.round(M.pose_errors(solved_local)[0], 2)) == [0, 0.05, 0.06, 0.09, 0.22, 0.25, 0.41, 0.37, 0.54]
    assert list(np.round(M.pose_errors(solved_pose)[0], 2)) == [0, 0.04, 0.03, 0.05, 0.07, 0.05, 0.06, 0.04, 0.02]
    assert list(np.round(moved, 2)) == [0, 0.01, 0.06, 0.11, 0.25, 0.28, 0.48, 0.42, 0.59]
    assert np.degrees(M.pose_errors(solved_pose)[1]).max() < 1.0
    assert max(np.linalg.norm(solved_local[M.pose_slice(i)][:2] - mu0[M.pose_slice(i)][:2]) for i in range(9)) < 0.05
    solved_landmark = M.solve(mu0, M.WITH_LANDMARK_CLOSURE)[-1]
    assert abs(M.pose_errors(solved_landmark)[0][8] - 0.51) < 0.005
    assert list(np.round(M.pose_errors(solved_landmark)[0], 2)) == [0, 0.05, 0.06, 0.09, 0.24, 0.29, 0.46, 0.39, 0.51]
    assert M.pose_errors(mu0)[0].max() < 0.62 and np.degrees(M.pose_errors(mu0)[1]).max() < 13.0
    assert len(M.solve(mu0, M.WITH_POSE_CLOSURE)) - 1 == 6
    assert_bearing_side_note(mu0)
    assert n_new_blocks == 4, n_new_blocks
    om_local, _ = M.build_system(M.LOCAL, mu0)
    assert int(M.cholesky_pattern(om_local, M.appearance_order(M.LOCAL)).sum()) == 128
    assert abs(100 * (np.abs(omega) > 1e-9).mean() - 22.0) < 0.05
    assert abs(100 * (np.abs(np.linalg.inv(omega)) > 1e-9).mean() - 91.9) < 0.05
    rows = {r["poses"]: r for r in M.scaling_rows()}
    assert rows[200]["hub_nnz_l_first"] == 181503 and rows[200]["hub_nnz_l_last"] == 4194
    assert rows[200]["chain_nnz_l"] == 2991 and rows[200]["ring_nnz_l_natural"] == 4773
    print("Generated loop-closure and sparse-solver figures in", OUT_DIR)


if __name__ == "__main__":
    main()
