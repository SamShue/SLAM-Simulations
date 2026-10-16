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
VEC_ERR = "#ff8a80"

A1 = np.array([1.0, 1.0, 1.0])
A2 = np.array([0.0, 1.0, 2.0])
B = np.array([1.0, 2.0, 3.0])

# A second, inconsistent right-hand side: only the third measurement changes.
B_NO_SOLUTION = np.array([1.0, 2.0, 6.0])

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


def new_3d_plane_axes(elev=18, azim=-55, span=(-2, 11), plane_range=((-0.3, 3.3), (-0.3, 3.3))):
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
    ax.set_xlim(*span)
    ax.set_ylim(*span)
    ax.set_zlim(*span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)

    xs, ys, zs = plane_grid(*plane_range)
    ax.plot_surface(xs, ys, zs, color=PLANE, alpha=0.55, shade=False, linewidth=0.5,
                     edgecolor=(1, 1, 1, 0.25), rstride=1, cstride=1, zorder=1,
                     antialiased=False)
    return fig, ax


def column_space_figure():
    fig, ax = new_3d_plane_axes()

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


def no_solution_figure():
    fig, ax = new_3d_plane_axes(elev=12, azim=-35, span=(-1, 7.5),
                                 plane_range=((-0.3, 1.5), (-0.3, 2.8)))

    draw_arrow(ax, A1, COL_A1, r"$\mathbf{a}_1$")
    draw_arrow(ax, A2, COL_A2, r"$\mathbf{a}_2$")

    # The closest point the plane actually contains, and the gap from there up to b.
    AtA = np.column_stack([A1, A2]).T @ np.column_stack([A1, A2])
    Atb = np.column_stack([A1, A2]).T @ B_NO_SOLUTION
    xhat = np.linalg.solve(AtA, Atb)
    foot = xhat[0] * A1 + xhat[1] * A2
    gap = B_NO_SOLUTION - foot

    ax.scatter(*foot, color=VEC_ERR, s=70, zorder=6, edgecolor=BG, linewidth=1.0)
    ax.quiver(*foot, *gap, color=VEC_ERR, linewidth=4.2, arrow_length_ratio=0.18, zorder=7)
    draw_arrow(ax, B_NO_SOLUTION, POINT, r"$\mathbf{b}=(1,2,6)$", lw=3.8, zorder=8,
               label_offset=1.05)

    ax.set_title("b does not lie on the plane: this system has no solution", color=TEXT,
                 fontsize=13.5, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "no_solution.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def projection_figure():
    fig, ax = new_3d_plane_axes(elev=12, azim=-35, span=(-1, 7.5),
                                 plane_range=((-0.3, 1.5), (-0.3, 2.8)))

    draw_arrow(ax, A1, COL_A1, r"$\mathbf{a}_1$")
    draw_arrow(ax, A2, COL_A2, r"$\mathbf{a}_2$")

    AtA = np.column_stack([A1, A2]).T @ np.column_stack([A1, A2])
    Atb = np.column_stack([A1, A2]).T @ B_NO_SOLUTION
    xhat = np.linalg.solve(AtA, Atb)
    p = xhat[0] * A1 + xhat[1] * A2
    e = B_NO_SOLUTION - p

    draw_arrow(ax, p, POINT, r"$\mathbf{p}=A\hat{\mathbf{x}}$", lw=3.6, zorder=6,
               label_offset=1.18)
    ax.quiver(*p, *e, color=VEC_ERR, linewidth=4.2, arrow_length_ratio=0.18, zorder=7)
    ax.text(*(p + e * 0.55 + np.array([0.3, 0.3, 0.3])), r"$\mathbf{e}$", color=VEC_ERR,
            fontsize=14, fontweight="bold", zorder=8)
    draw_arrow(ax, B_NO_SOLUTION, "#ffffff", r"$\mathbf{b}$", lw=2.4, zorder=5,
               label_offset=1.05)

    ax.set_title(r"$\mathbf{p}$ is the closest point in Col(A); $\mathbf{e}$ is orthogonal to it",
                 color=TEXT, fontsize=13, pad=0)
    fig.savefig(os.path.join(OUT_DIR, "projection.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def new_2d_axes(ylim=(0, 4)):
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
    ax.set_ylim(*ylim)
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


def data_points_no_fit_figure():
    fig, ax = new_2d_axes(ylim=(0, 7))
    ax.plot(A2, B_NO_SOLUTION, "o", color=POINT, markersize=13, markeredgecolor=BG,
            markeredgewidth=1.3, zorder=4)
    for t, y in zip(A2, B_NO_SOLUTION):
        ax.annotate(f"({t:.0f}, {y:.0f})", xy=(t, y), xytext=(t + 0.08, y + 0.15), color=TEXT,
                    fontsize=12)
    ax.set_title("Three points that don't lie on a single line", color=TEXT, fontsize=15, pad=12)
    fig.savefig(os.path.join(OUT_DIR, "data_points_no_fit.png"), dpi=70, facecolor=BG)
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


def least_squares_fit_figure():
    AtA = np.column_stack([A1, A2]).T @ np.column_stack([A1, A2])
    Atb = np.column_stack([A1, A2]).T @ B_NO_SOLUTION
    xhat = np.linalg.solve(AtA, Atb)
    c_hat, m_hat = xhat[0], xhat[1]

    fig, ax = new_2d_axes(ylim=(0, 7))
    ts = np.array([-0.5, 2.5])
    ax.plot(ts, m_hat * ts + c_hat, color=COL_A1, linewidth=3,
            label=rf"$y={m_hat:.1f}t+{c_hat:.1f}$", zorder=2)

    for t, y in zip(A2, B_NO_SOLUTION):
        y_hat = m_hat * t + c_hat
        ax.plot([t, t], [y, y_hat], color=VEC_ERR, linewidth=2.2, linestyle="--", zorder=3)

    ax.plot(A2, B_NO_SOLUTION, "o", color=POINT, markersize=13, markeredgecolor=BG,
            markeredgewidth=1.3, zorder=4, label="data")
    ax.set_title("The least squares line minimizes the total residual gap", color=TEXT,
                 fontsize=14.5, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=13)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "least_squares_fit.png"), dpi=70, facecolor=BG)
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

    # Check that the first two rows of the inconsistent system agree with the original
    # solution, but the third row does not -- confirming there is no exact solution.
    x_partial = np.linalg.solve(np.column_stack([A1, A2])[:2], B_NO_SOLUTION[:2])
    assert np.allclose(x_partial, [1.0, 1.0])
    predicted_third_row = np.array([A1[2], A2[2]]) @ x_partial
    assert np.isclose(predicted_third_row, 3.0)
    assert not np.isclose(predicted_third_row, B_NO_SOLUTION[2])

    # Check the normal-equations solution quoted in the document for b = (1, 2, 6).
    AtA = np.column_stack([A1, A2]).T @ np.column_stack([A1, A2])
    Atb = np.column_stack([A1, A2]).T @ B_NO_SOLUTION
    assert np.allclose(AtA, [[3, 3], [3, 5]])
    assert np.allclose(Atb, [9, 14])
    xhat = np.linalg.solve(AtA, Atb)
    assert np.allclose(xhat, [0.5, 2.5])
    p = xhat[0] * A1 + xhat[1] * A2
    e = B_NO_SOLUTION - p
    assert np.allclose(p, [0.5, 3.0, 5.5])
    assert np.allclose(e, [0.5, -1.0, 0.5])
    # The error must be orthogonal to every column of A.
    assert np.isclose(e @ A1, 0.0)
    assert np.isclose(e @ A2, 0.0)

    column_space_figure()
    no_solution_figure()
    data_points_figure()
    data_points_no_fit_figure()
    line_fit_figure()
    projection_figure()
    least_squares_fit_figure()
    print("Generated 7 figures in", OUT_DIR)


if __name__ == "__main__":
    main()
