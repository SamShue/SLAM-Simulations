"""Generate the figures used by least_squares_linear_algebra.md.

Run from anywhere:  python Least_Squares/Documents/images/generate_linear_algebra_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3D projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Same dark palette used throughout the repository's other figures.
BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
POINT = "#ffd166"
COL_A1 = "#7fd1ff"
COL_A2 = "#c792ea"
VEC_B = "#ffd166"
VEC_PROJ = "#7ee787"
VEC_ERR = "#ff8a80"
PLANE = "#4f7fa3"
LINE_BEST = "#7fd1ff"

# The deliberately inconsistent 3-point dataset used throughout the document.
# Using x = -1, 0, 1 makes the two columns of A exactly perpendicular (their dot
# product is the sum of the x-values, which is zero), so the column-space "plane"
# renders as a clean rectangle instead of a narrow sliver, and the normal equations
# decouple into two one-line divisions.
X = np.array([-1.0, 0.0, 1.0])
Y = np.array([1.0, 3.0, 6.0])

A1 = X.copy()                     # first column of A: the x-values
A2 = np.array([1.0, 1.0, 1.0])    # second column of A: all ones
B = Y.copy()                      # right-hand side

AtA = np.column_stack([A1, A2]).T @ np.column_stack([A1, A2])
Atb = np.column_stack([A1, A2]).T @ B
M_HAT, C_HAT = np.linalg.solve(AtA, Atb)
PHAT = M_HAT * A1 + C_HAT * A2
E = B - PHAT


def plane_grid(s_range, t_range, n=2):
    s = np.linspace(*s_range, n)
    t = np.linspace(*t_range, n)
    ss, tt = np.meshgrid(s, t)
    xs = ss * A1[0] + tt * A2[0]
    ys = ss * A1[1] + tt * A2[1]
    zs = ss * A1[2] + tt * A2[2]
    return xs, ys, zs


def new_3d_axes(figsize=(9, 8.5), elev=20, azim=-50):
    fig = plt.figure(figsize=figsize, facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    ax.set_facecolor(BG)
    ax.xaxis.set_pane_color((0, 0, 0, 0))
    ax.yaxis.set_pane_color((0, 0, 0, 0))
    ax.zaxis.set_pane_color((0, 0, 0, 0))
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"]["color"] = (0.5, 0.6, 0.65, 0.25)
    ax.tick_params(colors=TEXT, labelsize=9)
    ax.set_xlabel("measurement 1", color=TEXT, fontsize=11, labelpad=8)
    ax.set_ylabel("measurement 2", color=TEXT, fontsize=11, labelpad=8)
    ax.set_zlabel("measurement 3", color=TEXT, fontsize=11, labelpad=8)
    # A single shared bounding box for every figure, large enough to hold a1, a2, b,
    # the projection, and the plane patch drawn below, with no part of it clipped.
    # All three axes share the same span so that angles render undistorted.
    ax.set_xlim(-4, 6)
    ax.set_ylim(-2, 8)
    ax.set_zlim(-2, 8)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    return fig, ax


def draw_arrow(ax, vec, color, label=None, lw=3.2, zorder=5, label_offset=1.08):
    ax.quiver(0, 0, 0, *vec, color=color, linewidth=lw, arrow_length_ratio=0.08, zorder=zorder)
    if label:
        ax.text(*(np.array(vec) * label_offset), label, color=color, fontsize=13,
                fontweight="bold", zorder=zorder + 1)


def draw_surface(ax, s_range, t_range):
    xs, ys, zs = plane_grid(s_range, t_range, n=4)
    ax.plot_surface(xs, ys, zs, color=PLANE, alpha=0.72, shade=False, linewidth=0.6,
                     edgecolor=(1, 1, 1, 0.3), rstride=1, cstride=1, zorder=1,
                     antialiased=False)


def vectors_as_columns():
    fig, ax = new_3d_axes()
    combo = A1 + A2
    draw_arrow(ax, A1, COL_A1, r"$a_1=(-1,0,1)$")
    draw_arrow(ax, A2, COL_A2, r"$a_2=(1,1,1)$", label_offset=1.3)
    # Tip-to-tail construction: slide a copy of a2 to the tip of a1.
    ax.plot([A1[0], combo[0]], [A1[1], combo[1]], [A1[2], combo[2]],
            color=COL_A2, linestyle=(0, (3, 2)), linewidth=2.4, zorder=4)
    draw_arrow(ax, combo, TEXT, r"$a_1+a_2$", lw=3.0, zorder=6)
    ax.set_title("A matrix is just a list of column vectors", color=TEXT, fontsize=14, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "vectors_as_columns.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def column_space_plane():
    fig, ax = new_3d_axes(elev=22, azim=-40)
    draw_surface(ax, (-0.3, 2.9), (-0.3, 3.7))
    draw_arrow(ax, A1, COL_A1, r"$a_1$", zorder=5)
    draw_arrow(ax, A2, COL_A2, r"$a_2$", zorder=5, label_offset=1.3)
    draw_arrow(ax, B, VEC_B, r"$b=(1,3,6)$", zorder=7)
    ax.scatter(*PHAT, color=VEC_PROJ, s=90, zorder=7, edgecolor=BG, linewidth=1.2)
    ax.text(PHAT[0], PHAT[1], PHAT[2] - 0.6, "closest reachable point", color=VEC_PROJ,
            fontsize=11, zorder=7, ha="center")
    ax.plot([B[0], PHAT[0]], [B[1], PHAT[1]], [B[2], PHAT[2]], color=VEC_ERR,
            linewidth=2.8, zorder=6)
    ax.set_title("b does not lie on the plane spanned by a\u2081 and a\u2082", color=TEXT,
                 fontsize=13.5, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "column_space_plane.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def projection_figure():
    fig, ax = new_3d_axes(elev=22, azim=-40)
    draw_surface(ax, (-0.3, 2.9), (-0.3, 3.7))
    draw_arrow(ax, A1, COL_A1, r"$a_1$", zorder=5)
    draw_arrow(ax, A2, COL_A2, r"$a_2$", zorder=5, label_offset=1.3)
    draw_arrow(ax, B, VEC_B, zorder=6)
    draw_arrow(ax, PHAT, VEC_PROJ, zorder=6)
    ax.text(*(B + np.array([0.3, 0.3, 0.4])), r"$b$", color=VEC_B, fontsize=14,
            fontweight="bold", zorder=7)
    ax.text(*(PHAT + np.array([0.3, 0.3, -0.7])), r"$\hat b=A\hat x$", color=VEC_PROJ,
            fontsize=14, fontweight="bold", zorder=7)
    ax.plot([B[0], PHAT[0]], [B[1], PHAT[1]], [B[2], PHAT[2]], color=VEC_ERR,
            linewidth=2.8, zorder=6)
    mid = (np.array(B) + np.array(PHAT)) / 2
    ax.text(*(mid + np.array([0.3, 0.3, 0])), "e", color=VEC_ERR, fontsize=14,
            fontweight="bold", zorder=7)

    # A small right-angle mark at the foot of the projection, to show e is perpendicular
    # to the plane: one short leg lying in the plane, one short leg along e.
    u = A1 / np.linalg.norm(A1)
    e_hat = E / np.linalg.norm(E)
    eps = 0.35
    p0 = PHAT
    p1 = PHAT + eps * u
    p2 = p1 + eps * e_hat
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color=TEXT, linewidth=1.6, zorder=7)
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=TEXT, linewidth=1.6, zorder=7)

    ax.set_title("The error e is perpendicular to the whole plane", color=TEXT, fontsize=14, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "projection.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def best_fit_linear_algebra():
    fig = plt.figure(figsize=(9.2, 6.5), facecolor=BG)
    ax = fig.add_axes([0.11, 0.11, 0.86, 0.82])
    ax.set_facecolor(BG)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("x", color=TEXT, fontsize=15)
    ax.set_ylabel("y", color=TEXT, fontsize=15)
    ax.grid(True, color=GRID, alpha=0.25, linewidth=0.8)
    ax.set_xlim(-2, 2)
    ax.set_ylim(0, 7)

    xs = np.array([-2, 2])
    ax.plot(xs, M_HAT * xs + C_HAT, color=LINE_BEST, linewidth=3,
            label=rf"$\hat y=2.5x+\frac{{10}}{{3}}$", zorder=2)
    ax.plot(X, Y, "o", color=POINT, markersize=13, markeredgecolor=BG, markeredgewidth=1.3, zorder=4)
    for xi, yi in zip(X, Y):
        pi = M_HAT * xi + C_HAT
        ax.plot([xi, xi], [pi, yi], color=GRID, linestyle=(0, (4, 3)), linewidth=2, zorder=1)
    ax.set_title("The same best-fit line, found through projection", color=TEXT, fontsize=15, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=14)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "best_fit_linear_algebra.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Sanity-check the numbers quoted in the document.
    assert abs(np.dot(A1, A2)) < 1e-9  # a1 and a2 are exactly perpendicular by design
    assert np.allclose([M_HAT, C_HAT], [2.5, 10.0 / 3.0])
    assert np.allclose(PHAT, [5.0 / 6.0, 10.0 / 3.0, 35.0 / 6.0])
    assert np.allclose(E, [1.0 / 6.0, -1.0 / 3.0, 1.0 / 6.0])
    assert abs(np.dot(A1, E)) < 1e-9
    assert abs(np.dot(A2, E)) < 1e-9
    assert abs(np.sum(E ** 2) - 1.0 / 6.0) < 1e-9

    vectors_as_columns()
    column_space_plane()
    projection_figure()
    best_fit_linear_algebra()

    print("Generated 4 figures in", OUT_DIR)


if __name__ == "__main__":
    main()




