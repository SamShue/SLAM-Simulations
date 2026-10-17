"""Generate the figures used by ekf_slam_introduction.md.

Run from anywhere:  python EKF_SLAM/Documents/images/generate_ekf_slam_introduction_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Same dark palette used throughout the repository's other figures.
BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
CELL_TEXT = "#0b1216"

PRIOR = "#7fd1ff"
MEAS = "#ffd166"
POST = "#c792ea"

ROBOT = "#7fd1ff"
CROSS = "#c792ea"
LANDMARK = "#ffd166"
LL_CROSS = "#4f7fa3"
UNCHANGED = "#2d3a42"


def gaussian(x, mu, var):
    return np.exp(-0.5 * (x - mu) ** 2 / var) / np.sqrt(2 * np.pi * var)


# ---------------------------------------------------------------------------
# Figure 0: the core SLAM intuition -- observe, drift, correct
# ---------------------------------------------------------------------------

TRUE_LINE = "#ffffff"
ODOM_LINE = "#9fb3bf"
CORRECTED_LINE = "#c792ea"
SENSOR_RAY = "#ffd166"
EXPECTED_RAY = "#c792ea"

L0 = (4.0, 1.6)
L1 = (3.6, -1.8)
POSE0 = (0.0, 0.0, 0.0)
POSE1_TRUE = (2.6, 1.0, np.radians(25))
POSE1_ODOM = (3.4, -0.7, np.radians(-5))
POSE1_CORRECTED = (2.9, 0.45, np.radians(15))

INTUITION_XLIM = (-1.3, 5.3)
INTUITION_YLIM = (-2.6, 2.6)


def _rotate(px, py, cx, cy, angle):
    dx, dy = px - cx, py - cy
    ca, sa = np.cos(angle), np.sin(angle)
    return cx + dx * ca - dy * sa, cy + dx * sa + dy * ca


def robot_triangle(ax, pose, color, linestyle="-", size=0.4, zorder=5):
    cx, cy, heading = pose
    s = size
    pts = [(cx - s * 0.8, cy + s), (cx - s * 0.8, cy - s), (cx + s * 1.1, cy)]
    pts = [_rotate(px, py, cx, cy, heading) for px, py in pts]
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=color, linewidth=2.6,
                          linestyle=linestyle, zorder=zorder))


def star(ax, cx, cy, color=TEXT, r_out=0.22, r_in=0.09, zorder=4):
    pts = []
    for k in range(8):
        ang = np.pi / 2 + k * np.pi / 4
        r = r_out if k % 2 == 0 else r_in
        pts.append((cx + r * np.cos(ang), cy + r * np.sin(ang)))
    ax.add_patch(Polygon(pts, closed=True, facecolor=BG, edgecolor=color, linewidth=2.0, zorder=zorder))


def _new_intuition_axes(ax, title):
    ax.set_xlim(*INTUITION_XLIM)
    ax.set_ylim(*INTUITION_YLIM)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, color=TEXT, fontsize=13.5, pad=10)


def _draw_landmarks(ax):
    star(ax, *L0)
    ax.text(L0[0], L0[1] + 0.34, r"$L_0$", color=TEXT, fontsize=11, ha="center")
    star(ax, *L1)
    ax.text(L1[0], L1[1] - 0.4, r"$L_1$", color=TEXT, fontsize=11, ha="center")


def slam_intuition_figure():
    fig = plt.figure(figsize=(15.0, 5.3), facecolor=BG)

    # Panel 1: observe the landmarks from the starting pose.
    ax1 = fig.add_axes([0.02, 0.06, 0.30, 0.82])
    _new_intuition_axes(ax1, "1. Observe landmarks")
    robot_triangle(ax1, POSE0, TRUE_LINE)
    _draw_landmarks(ax1)
    for lm in (L0, L1):
        ax1.plot([POSE0[0], lm[0]], [POSE0[1], lm[1]], color=SENSOR_RAY, linewidth=2.0,
                  linestyle=(0, (4, 3)), zorder=2)

    # Panel 2: drive using odometry alone, then re-observe -- nothing lines up.
    ax2 = fig.add_axes([0.35, 0.06, 0.30, 0.82])
    _new_intuition_axes(ax2, "2. Drive, then re-observe")
    ax2.plot([POSE0[0], POSE1_ODOM[0]], [POSE0[1], POSE1_ODOM[1]], color=ODOM_LINE,
              linewidth=1.8, linestyle=(0, (2, 2)), zorder=1)
    robot_triangle(ax2, POSE1_ODOM, ODOM_LINE, linestyle="--")
    robot_triangle(ax2, POSE1_TRUE, TRUE_LINE)
    _draw_landmarks(ax2)
    for lm in (L0, L1):
        ax2.plot([POSE1_TRUE[0], lm[0]], [POSE1_TRUE[1], lm[1]], color=SENSOR_RAY, linewidth=2.0,
                  linestyle=(0, (4, 3)), zorder=2)
        ax2.plot([POSE1_ODOM[0], lm[0]], [POSE1_ODOM[1], lm[1]], color=EXPECTED_RAY, linewidth=1.6,
                  linestyle=(0, (1, 2)), zorder=2)

    # Panel 3: the correction sits between the odometry-only guess and the truth.
    ax3 = fig.add_axes([0.68, 0.06, 0.30, 0.82])
    _new_intuition_axes(ax3, "3. Correct using the mismatch")
    robot_triangle(ax3, POSE1_ODOM, ODOM_LINE, linestyle="--")
    robot_triangle(ax3, POSE1_CORRECTED, CORRECTED_LINE, linestyle=(0, (1, 1)))
    robot_triangle(ax3, POSE1_TRUE, TRUE_LINE)
    _draw_landmarks(ax3)

    legend_items = [("true pose", TRUE_LINE, "-"), ("odometry-only estimate", ODOM_LINE, "--"),
                    ("corrected (EKF) estimate", CORRECTED_LINE, (0, (1, 1)))]
    ly = -1.7
    for label, color, ls in legend_items:
        ax3.plot([-1.25, -0.55], [ly, ly], color=color, linestyle=ls, linewidth=3.0, zorder=6)
        ax3.text(-0.4, ly, label, color=TEXT, fontsize=9.5, va="center", zorder=6)
        ly -= 0.36

    fig.savefig(os.path.join(OUT_DIR, "slam_process_intuition.png"), dpi=130, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 1: fusing a predicted belief with a measurement
# ---------------------------------------------------------------------------

MU_PRED, VAR_PRED = 0.0, 5.0
MU_MEAS, VAR_MEAS = 3.0, 2.0


def kalman_gain(var_pred, var_meas):
    return var_pred / (var_pred + var_meas)


def fuse(mu_pred, var_pred, mu_meas, var_meas):
    K = kalman_gain(var_pred, var_meas)
    mu_post = mu_pred + K * (mu_meas - mu_pred)
    var_post = (1 - K) * var_pred
    return mu_post, var_post


def kalman_filter_fusion_figure():
    mu_post, var_post = fuse(MU_PRED, VAR_PRED, MU_MEAS, VAR_MEAS)

    fig = plt.figure(figsize=(8.5, 6), facecolor=BG)
    ax = fig.add_axes([0.09, 0.12, 0.88, 0.8])
    ax.set_facecolor(BG)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=12)
    ax.set_yticks([])

    xs = np.linspace(-6, 9, 600)
    ax.plot(xs, gaussian(xs, MU_PRED, VAR_PRED), color=PRIOR, linewidth=3,
            label=rf"predicted belief ($\mu={MU_PRED:.0f}$, $\sigma^2={VAR_PRED:.0f}$)")
    ax.plot(xs, gaussian(xs, MU_MEAS, VAR_MEAS), color=MEAS, linewidth=3,
            label=rf"measurement ($\mu={MU_MEAS:.0f}$, $\sigma^2={VAR_MEAS:.0f}$)")
    ax.plot(xs, gaussian(xs, mu_post, var_post), color=POST, linewidth=3.4,
            label=rf"posterior belief ($\mu={mu_post:.2f}$, $\sigma^2={var_post:.2f}$)")
    ax.fill_between(xs, gaussian(xs, mu_post, var_post), color=POST, alpha=0.18)

    ax.set_xlabel("position", color=TEXT, fontsize=13)
    ax.set_title("Fusing a predicted belief with a measurement", color=TEXT, fontsize=15, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=11.5)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "kalman_filter_fusion.png"), dpi=130, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 2: the block structure of the EKF SLAM covariance matrix
# ---------------------------------------------------------------------------

STATE_LABELS = [r"$x$", r"$y$", r"$\theta$",
                r"$L_{0x}$", r"$L_{0y}$", r"$L_{1x}$", r"$L_{1y}$", r"$L_{2x}$", r"$L_{2y}$"]
N_STATE = len(STATE_LABELS)
ROBOT_IDX = (0, 3)       # [0, 3)
LANDMARK_BOUNDS = [(3, 5), (5, 7), (7, 9)]  # L0, L1, L2


def cell(ax, j, i, color):
    ax.add_patch(Rectangle((j, i), 1, 1, facecolor=color, edgecolor=GRID, linewidth=1.0, zorder=2))


def _legend_item_width(label):
    return 0.4 + 0.13 * len(label) + 0.85


def _layout_legend(legend_items, max_row_width):
    positions = []
    lx, row = 0.0, 0
    for color, label in legend_items:
        w = _legend_item_width(label)
        if lx > 0 and lx + w > max_row_width:
            row += 1
            lx = 0.0
        positions.append((lx, row, color, label))
        lx += w
    n_rows = positions[-1][1] + 1 if positions else 1
    return positions, n_rows


def draw_block_matrix(ax, title, block_colors, legend_items, max_legend_row_width=None):
    n = N_STATE
    top_gutter = 0.9
    max_row_width = max_legend_row_width if max_legend_row_width is not None else n + 1.5
    positions, n_rows = _layout_legend(legend_items, max_row_width)
    bottom_gutter = 1.3 + 0.55 * n_rows + 0.3

    ax.set_xlim(-2.0, max(n + 0.3, max_row_width))
    ax.set_ylim(n + bottom_gutter, -top_gutter)
    ax.set_aspect("equal")
    ax.axis("off")

    for j in range(n):
        ax.text(j + 0.5, -0.25, STATE_LABELS[j], color=TEXT, fontsize=11.5, ha="center", va="bottom")
    for i in range(n):
        ax.text(-0.25, i + 0.5, STATE_LABELS[i], color=TEXT, fontsize=11.5, ha="right", va="center")

    for i in range(n):
        for j in range(n):
            cell(ax, j, i, BG)

    for (j0, j1, i0, i1), color in block_colors:
        for i in range(i0, i1):
            for j in range(j0, j1):
                cell(ax, j, i, color)

    ax.text(n / 2, n + 0.65, title, color=TEXT, fontsize=14, ha="center", va="center")

    legend_top = n + 1.3
    for lx, row, color, label in positions:
        ly = legend_top + row * 0.55
        ax.add_patch(Rectangle((lx, ly), 0.35, 0.35, facecolor=color, edgecolor=GRID))
        ax.text(lx + 0.48, ly + 0.17, label, color=TEXT, fontsize=10.5, va="center")


def covariance_blocks_figure():
    rr = ROBOT_IDX
    blocks = [
        ((rr[0], rr[1], rr[0], rr[1]), ROBOT),
    ]
    # robot-landmark cross blocks (both P_rl and P_lr)
    for (a, b) in LANDMARK_BOUNDS:
        blocks.append(((a, b, rr[0], rr[1]), CROSS))
        blocks.append(((rr[0], rr[1], a, b), CROSS))
    # landmark-landmark blocks: self blocks (landmark variance) vs. cross blocks
    # (correlation between two different landmarks)
    for idx_a, (a0, a1) in enumerate(LANDMARK_BOUNDS):
        for idx_b, (b0, b1) in enumerate(LANDMARK_BOUNDS):
            color = LANDMARK if idx_a == idx_b else LL_CROSS
            blocks.append(((b0, b1, a0, a1), color))

    fig = plt.figure(figsize=(8.6, 7.8), facecolor=BG)
    ax = fig.add_axes([0.14, 0.04, 0.82, 0.92])
    draw_block_matrix(ax, r"Block structure of the EKF SLAM covariance matrix $P$", blocks, [
        (ROBOT, "robot-robot, " + r"$P_{rr}$"),
        (CROSS, "robot-landmark, " + r"$P_{rl}=P_{lr}^{\mathsf{T}}$"),
        (LANDMARK, "landmark variance"),
        (LL_CROSS, "landmark-landmark correlation"),
    ])
    fig.savefig(os.path.join(OUT_DIR, "ekf_slam_covariance_blocks.png"), dpi=140, facecolor=BG)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Figure 3: which blocks change during predict vs. correct
# ---------------------------------------------------------------------------

def predict_correct_figure():
    fig = plt.figure(figsize=(15.0, 8.5), facecolor=BG)

    # Predict step: only the robot block and the robot/landmark cross blocks
    # are touched; the landmark-landmark block is left completely unchanged.
    predict_blocks = [((ROBOT_IDX[0], ROBOT_IDX[1], ROBOT_IDX[0], ROBOT_IDX[1]), ROBOT)]
    for (a, b) in LANDMARK_BOUNDS:
        predict_blocks.append(((a, b, ROBOT_IDX[0], ROBOT_IDX[1]), CROSS))
        predict_blocks.append(((ROBOT_IDX[0], ROBOT_IDX[1], a, b), CROSS))
    for idx_a, (a0, a1) in enumerate(LANDMARK_BOUNDS):
        for idx_b, (b0, b1) in enumerate(LANDMARK_BOUNDS):
            predict_blocks.append(((b0, b1, a0, a1), UNCHANGED))

    # Correct step observing L1: H only directly reads the robot block and
    # the L1 block, but K = P*H'*S^-1 multiplies through every column of P,
    # so every block that is correlated with the robot or L1 is also
    # updated -- in this example, that ends up being the entire matrix.
    l1_a, l1_b = LANDMARK_BOUNDS[1]
    correct_blocks = [((ROBOT_IDX[0], ROBOT_IDX[1], ROBOT_IDX[0], ROBOT_IDX[1]), ROBOT),
                      ((l1_a, l1_b, l1_a, l1_b), LANDMARK)]
    for (a, b) in LANDMARK_BOUNDS:
        correct_blocks.append(((a, b, ROBOT_IDX[0], ROBOT_IDX[1]), CROSS))
        correct_blocks.append(((ROBOT_IDX[0], ROBOT_IDX[1], a, b), CROSS))
    for idx_a, (a0, a1) in enumerate(LANDMARK_BOUNDS):
        for idx_b, (b0, b1) in enumerate(LANDMARK_BOUNDS):
            if idx_a == idx_b:
                if idx_a != 1:
                    correct_blocks.append(((b0, b1, a0, a1), LL_CROSS))
            else:
                correct_blocks.append(((b0, b1, a0, a1), LL_CROSS))

    ax1 = fig.add_axes([0.05, 0.05, 0.43, 0.88])
    draw_block_matrix(ax1, "Predict: only the robot block and its\ncross-terms change", predict_blocks, [
        (ROBOT, "robot block"),
        (CROSS, "robot-landmark cross terms"),
        (UNCHANGED, "unchanged"),
    ])

    ax2 = fig.add_axes([0.55, 0.05, 0.43, 0.88])
    draw_block_matrix(ax2, r"Correct (observing $L_1$): the whole" + "\nmatrix shifts through correlation",
                       correct_blocks, [
        (ROBOT, "robot (observed)"),
        (LANDMARK, r"$L_1$ (observed)"),
        (LL_CROSS, "updated via correlation"),
    ])

    fig.savefig(os.path.join(OUT_DIR, "ekf_slam_predict_correct.png"), dpi=130, facecolor=BG)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Sanity-check the numbers quoted in the document.
    K = kalman_gain(VAR_PRED, VAR_MEAS)
    assert np.isclose(K, 5.0 / 7.0)
    mu_post, var_post = fuse(MU_PRED, VAR_PRED, MU_MEAS, VAR_MEAS)
    assert np.isclose(mu_post, 15.0 / 7.0)
    assert np.isclose(var_post, 10.0 / 7.0)
    # Equivalent precision-weighted-average form must agree.
    mu_check = var_post * (MU_PRED / VAR_PRED + MU_MEAS / VAR_MEAS)
    var_check = 1.0 / (1.0 / VAR_PRED + 1.0 / VAR_MEAS)
    assert np.isclose(mu_check, mu_post)
    assert np.isclose(var_check, var_post)

    slam_intuition_figure()
    kalman_filter_fusion_figure()
    covariance_blocks_figure()
    predict_correct_figure()
    print("Generated 4 figures in", OUT_DIR)


if __name__ == "__main__":
    main()
