"""Generate the figures used by least_squares_introduction.md.

Run from anywhere:  python Least_Squares/Documents/images/generate_intro_figures.py
"""
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Same dark palette used throughout the repository's other figures.
BG = "#1b262c"
TEXT = "#ffffff"
GRID = "#9fb3bf"
POINT = "#ffd166"
LINE_A = "#7fd1ff"
LINE_B = "#c792ea"
LINE_BEST = "#7fd1ff"
POS_RESID = "#7fd1ff"
NEG_RESID = "#ff8a80"

# The small worked dataset used throughout the document: a plant's height (cm)
# measured once a day for six days.
DAY = np.array([1, 2, 3, 4, 5, 6], dtype=float)
HEIGHT = np.array([2.1, 3.9, 6.2, 7.8, 10.1, 11.9])

DAY_BAR = DAY.mean()
HEIGHT_BAR = HEIGHT.mean()
NUM = np.sum((DAY - DAY_BAR) * (HEIGHT - HEIGHT_BAR))
DEN = np.sum((DAY - DAY_BAR) ** 2)
M_BEST = NUM / DEN
B_BEST = HEIGHT_BAR - M_BEST * DAY_BAR


def new_axes(figsize=(9.5, 6.5), xlim=(0, 7), ylim=(0, 14)):
    fig = plt.figure(figsize=figsize, facecolor=BG)
    ax = fig.add_axes([0.11, 0.1, 0.86, 0.84])
    ax.set_facecolor(BG)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("Day", color=TEXT, fontsize=15)
    ax.set_ylabel("Height (cm)", color=TEXT, fontsize=15)
    ax.grid(True, color=GRID, alpha=0.25, linewidth=0.8)
    return fig, ax


def draw_points(ax, label=None):
    ax.plot(DAY, HEIGHT, "o", color=POINT, markersize=11, markeredgecolor=BG,
            markeredgewidth=1.2, zorder=4, label=label)


def scatter_no_line():
    fig, ax = new_axes()
    draw_points(ax)
    ax.set_title("Six measurements of a plant's height", color=TEXT, fontsize=16, pad=12)
    fig.savefig(os.path.join(OUT_DIR, "scatter_no_line.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def candidate_lines():
    fig, ax = new_axes()
    xs = np.array([0, 7])
    m_a, b_a = 1.0, 2.0
    m_b, b_b = 3.0, 0.0
    ax.plot(xs, m_a * xs + b_a, color=LINE_A, linewidth=3, label="Line A", zorder=2)
    ax.plot(xs, m_b * xs + b_b, color=LINE_B, linewidth=3, label="Line B", zorder=2)
    draw_points(ax)
    ax.set_title("Two guesses: which line fits better?", color=TEXT, fontsize=16, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=13)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "candidate_lines.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def residuals_figure():
    fig, ax = new_axes()
    m, b = 1.0, 2.0
    xs = np.array([0, 7])
    ax.plot(xs, m * xs + b, color=LINE_A, linewidth=3, label="Line A", zorder=2)
    pred = m * DAY + b
    for x, y, p in zip(DAY, HEIGHT, pred):
        ax.plot([x, x], [p, y], color=GRID, linestyle=(0, (4, 3)), linewidth=2, zorder=1)
    draw_points(ax)
    # Label the error on two representative points.
    ax.annotate("error", xy=(2, (HEIGHT[1] + pred[1]) / 2), xytext=(2.5, 2.3),
                color=TEXT, fontsize=13,
                arrowprops=dict(arrowstyle="-", color=TEXT, lw=1.2))
    ax.annotate("error", xy=(6, (HEIGHT[5] + pred[5]) / 2), xytext=(6.1, 10.6),
                color=TEXT, fontsize=13,
                arrowprops=dict(arrowstyle="-", color=TEXT, lw=1.2))
    ax.set_title("The error at each point: gap between the dot and the line", color=TEXT,
                 fontsize=15.5, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=13)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "residuals.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def cancellation_trap():
    fig, ax = new_axes()
    flat = HEIGHT_BAR
    ax.plot([0, 7], [flat, flat], color="#f4a742", linewidth=3,
            label=f"Flat line at the average ({flat:.1f} cm)", zorder=2)
    for x, y in zip(DAY, HEIGHT):
        color = POS_RESID if y >= flat else NEG_RESID
        ax.plot([x, x], [flat, y], color=color, linewidth=2.6, zorder=1)
    draw_points(ax)
    ax.set_title("A flat line: errors above and below cancel to exactly zero", color=TEXT,
                 fontsize=15, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=12.5)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "cancellation_trap.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def valley_plot():
    fig = plt.figure(figsize=(9.5, 6.5), facecolor=BG)
    ax = fig.add_axes([0.12, 0.1, 0.85, 0.84])
    ax.set_facecolor(BG)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=TEXT, labelsize=13)
    ax.set_xlabel("Slope being tried", color=TEXT, fontsize=15)
    ax.set_ylabel("Total squared error", color=TEXT, fontsize=15)
    ax.grid(True, color=GRID, alpha=0.25, linewidth=0.8)

    slopes = np.linspace(0.0, 4.0, 400)
    ssr = []
    for m in slopes:
        b = HEIGHT_BAR - m * DAY_BAR  # the best intercept for this slope
        pred = m * DAY + b
        ssr.append(np.sum((HEIGHT - pred) ** 2))
    ssr = np.array(ssr)

    ax.plot(slopes, ssr, color=LINE_A, linewidth=3, zorder=2)
    ax.plot([M_BEST], [np.sum((HEIGHT - (M_BEST * DAY + B_BEST)) ** 2)], "o", color=POINT,
            markersize=13, markeredgecolor=BG, markeredgewidth=1.4, zorder=3)
    ax.annotate(f"lowest point\nslope \u2248 {M_BEST:.2f}", xy=(M_BEST, 1.0),
                xytext=(M_BEST + 0.5, 25), color=TEXT, fontsize=13,
                arrowprops=dict(arrowstyle="-", color=TEXT, lw=1.2))
    ax.set_title("Trying every slope: the errors form a valley", color=TEXT, fontsize=16, pad=12)
    fig.savefig(os.path.join(OUT_DIR, "valley_plot.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def best_fit_figure():
    fig, ax = new_axes()
    xs = np.array([0, 7])
    ax.plot(xs, M_BEST * xs + B_BEST, color=LINE_BEST, linewidth=3,
            label=f"Best-fit line: y = {M_BEST:.2f}x + {B_BEST:.2f}", zorder=2)
    draw_points(ax)
    ax.set_title("The least-squares best-fit line", color=TEXT, fontsize=16, pad=12)
    leg = ax.legend(loc="upper left", facecolor=BG, edgecolor=GRID, fontsize=12.5)
    for text in leg.get_texts():
        text.set_color(TEXT)
    fig.savefig(os.path.join(OUT_DIR, "best_fit.png"), dpi=140, facecolor=BG)
    plt.close(fig)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Sanity-check the numbers quoted in the document.
    assert abs(DAY_BAR - 3.5) < 1e-9
    assert abs(HEIGHT_BAR - 7.0) < 1e-9
    assert abs(M_BEST - 1.9771428571428569) < 1e-9
    assert abs(B_BEST - 0.08000000000000096) < 1e-9
    pred_best = M_BEST * DAY + B_BEST
    ssr_best = np.sum((HEIGHT - pred_best) ** 2)
    assert abs(ssr_best - 0.11085714285714321) < 1e-9

    flat_resid = HEIGHT - HEIGHT_BAR
    assert abs(np.sum(flat_resid)) < 1e-9
    assert abs(np.sum(flat_resid ** 2) - 68.52000000000001) < 1e-6

    scatter_no_line()
    candidate_lines()
    residuals_figure()
    cancellation_trap()
    valley_plot()
    best_fit_figure()

    print("Generated 6 figures in", OUT_DIR)


if __name__ == "__main__":
    main()
