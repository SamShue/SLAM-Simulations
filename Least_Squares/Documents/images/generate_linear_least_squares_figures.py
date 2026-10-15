"""Generate the figures used by linear_least_squares.md.

Run from anywhere:  python Least_Squares/Documents/images/generate_linear_least_squares_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3D projection)

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Same dark palette used throughout the repository's other figures.
BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
COL_A1 = "#7fd1ff"
COL_A2 = "#c792ea"
POINT = "#ffd166"
PLANE = "#4f7fa3"

A1 = np.array([1.0, 1.0, 1.0])
A2 = np.array([0.0, 1.0, 2.0])
B = np.array([1.0, 2.0, 3.0])

# The example (x1, x2) combinations used in the document.
EXAMPLES = [(2.0, 3.0), (1.0, 1.0), (0.0, 2.0), (3.0, 0.0)]

# The original system's exact solution, reinterpreted as a line y = m t + c.
X1_SOL, X2_SOL = 1.0, 1.0
C_SOL, M_SOL = X1_SOL, X2_SOL


def plane_grid(s_range, t_range, n=4):
    s = np.linspace(*s_range, n)
    t = np.linspace(*t_range, n)
    ss, tt = np.meshgrid(s, t)
    xs = ss * A1[0] + tt * A2[0]
    ys = ss * A1[1] + tt * A2[1]
    zs = ss * A1[2] + tt * A2[2]
    return xs, ys, zs


def draw_arrow(ax, vec, color, label=None, lw=3.2, zorder=5, label_offset=1.12):
    ax.quiver(0, 0, 0, *vec, color=color, linewidth=lw, arrow_length_ratio=0.06, zorder=zorder)
    if label:
        ax.text(*(np.array(vec) * label_offset), label, color=color, fontsize=14,
                fontweight="bold", zorder=zorder + 1)


def column_space_figure():
    fig = plt.figure(figsize=(9.5, 8.5), facecolor=BG)
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    ax.set_facecolor(BG)
    ax.xaxis.set_pane_color((0, 0, 0, 0))
    ax.yaxis.set_pane_color((0, 0, 0, 0))
    ax.zaxis.set_pane_color((0, 0, 0, 0))
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis._axinfo["grid"]["color"] = (0.5, 0.6, 0.65, 0.25)
    ax.tick_params(colors=TEXT, labelsize=9)
    ax.set_xlabel("equation 1", color=TEXT, fontsize=11, labelpad=8)
    ax.set_ylabel("equation 2", color=TEXT, fontsize=11, labelpad=8)
    ax.set_zlabel("equation 3", color=TEXT, fontsize=11, labelpad=8)

    # A single shared, equal-span bounding box so angles render undistorted.
    ax.set_xlim(-2, 11)
    ax.set_ylim(-2, 11)
    ax.set_zlim(-2, 11)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=18, azim=-55)

    xs, ys, zs = plane_grid((-0.3, 3.3), (-0.3, 3.3))
    ax.plot_surface(xs, ys, zs, color=PLANE, alpha=0.55, shade=False, linewidth=0.5,
                     edgecolor=(1, 1, 1, 0.25), rstride=1, cstride=1, zorder=1,
                     antialiased=False)

    draw_arrow(ax, A1, COL_A1, r"$\mathbf{a}_1$")
    draw_arrow(ax, A2, COL_A2, r"$\mathbf{a}_2$")

    for x1, x2 in EXAMPLES:
        v = x1 * A1 + x2 * A2
        ax.scatter(*v, color=POINT, s=70, zorder=6, edgecolor=BG, linewidth=1.0)
        ax.text(v[0] + 0.2, v[1] + 0.2, v[2] + 0.2, f"({v[0]:.0f},{v[1]:.0f},{v[2]:.0f})",
                color=POINT, fontsize=10, zorder=7)

    ax.set_title("Every A x lands on the same plane through the origin", color=TEXT,
                 fontsize=14, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "column_space_plane.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def new_2d_axes():
    fig = plt.figure(figsize=(8, 6.2), facecolor=BG)
    ax = fig.add_axes([0.11, 0.12, 0.85, 0.82])
    ax.set_facecolor(BG)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("t", color=TEXT, fontsize=15)
    ax.set_ylabel("y", color=TEXT, fontsize=15)
    ax.grid(True, color=GRID, alpha=0.25, linewidth=0.8)
    ax.set_xlim(-0.5, 2.5)
    ax.set_ylim(0, 4)
    return fig, ax


def data_points_figure():
    fig, ax = new_2d_axes()
    ax.plot(A2, B, "o", color=POINT, markersize=13, markeredgecolor=BG, markeredgewidth=1.3,
            zorder=4)
    for t, y in zip(A2, B):
        ax.annotate(f"({t:.0f}, {y:.0f})", xy=(t, y), xytext=(t + 0.08, y + 0.15), color=TEXT,
                    fontsize=12)
    ax.set_title("The system's rows, reread as data points", color=TEXT, fontsize=15, pad=12)
    fig.savefig(os.path.join(OUT_DIR, "data_points.png"), dpi=70, facecolor=BG)
    plt.close(fig)


def line_fit_figure():
    fig, ax = new_2d_axes()
    ts = np.array([-0.5, 2.5])
    slope_term = "t" if M_SOL == 1 else f"{M_SOL:.0f}t"
    ax.plot(ts, M_SOL * ts + C_SOL, color=COL_A1, linewidth=3,
            label=rf"$y={slope_term}+{C_SOL:.0f}$", zorder=2)
    ax.plot(A2, B, "o", color=POINT, markersize=13, markeredgecolor=BG, markeredgewidth=1.3,
            zorder=4)
    ax.set_title("The solution is the line through every point", color=TEXT, fontsize=15, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=14)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "line_fit.png"), dpi=70, facecolor=BG)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Sanity-check the numbers quoted in the document.
    expected = {
        (2.0, 3.0): (2.0, 5.0, 8.0),
        (1.0, 1.0): (1.0, 2.0, 3.0),
        (0.0, 2.0): (0.0, 2.0, 4.0),
        (3.0, 0.0): (3.0, 3.0, 3.0),
    }
    for (x1, x2), expected_v in expected.items():
        v = x1 * A1 + x2 * A2
        assert np.allclose(v, expected_v), (x1, x2, v)

    # The solution, reread as a line y = m t + c, must pass through every (t, y) point.
    assert np.allclose(M_SOL * A2 + C_SOL, B)

    column_space_figure()
    data_points_figure()
    line_fit_figure()
    print("Generated 3 figures in", OUT_DIR)


if __name__ == "__main__":
    main()
