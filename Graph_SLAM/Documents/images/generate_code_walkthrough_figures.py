"""Draw the figures for graphslam_code_walkthrough.md from the data exported by MATLAB.

Step 1 (needs MATLAB; runs the real SIMULATION_GraphSLAM.m with a fixed random seed):
    matlab -batch "run('Graph_SLAM/Documents/images/generate_code_walkthrough_figures.m')"
Step 2 (this script):
    python Graph_SLAM/Documents/images/generate_code_walkthrough_figures.py
    python Graph_SLAM/Documents/images/generate_code_walkthrough_figures.py --numbers   # print quoted numbers

The assertions at the end check every number quoted in the Markdown against the exported data.
"""
import json
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from generate_step_figures import BG, CELL_NEW, CELL_ON, CONFLICT, GRID, ODOM, OBS, TEXT  # noqa: E402

TRUTH = "#8fa3ad"
GREEN = "#7bd88f"


def load():
    with open(os.path.join(HERE, "code_walkthrough_run.json")) as f:
        return json.load(f)


def save(fig, name):
    fig.savefig(os.path.join(HERE, name), dpi=140, facecolor=BG)
    plt.close(fig)


def style(ax):
    ax.set_facecolor(BG)
    ax.tick_params(colors=TEXT, labelsize=12)
    ax.grid(color="#33454d", linewidth=1.0)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def pos_error(est, truth):
    est, truth = np.asarray(est), np.asarray(truth)
    return np.hypot(*(est[:, :2] - truth[:, :2]).T)


def figure_observations(d):
    m = np.array(d["measurementLog"])
    fig, (ax, bar) = plt.subplots(1, 2, figsize=(14, 4.8), facecolor=BG, gridspec_kw={"width_ratios": [4, 1]})
    style(ax)
    style(bar)
    ax.scatter(m[:, 0], m[:, 1], s=34, color=OBS, zorder=3)
    ax.set_yticks(range(1, len(d["landmarks"]) + 1))
    ax.set_yticklabels([rf"$L_{k}$" for k in range(1, len(d["landmarks"]) + 1)], fontsize=14)
    ax.set_xlabel("pose index $t$ (one observation is logged for every pose after the first)", color=TEXT, fontsize=13)
    ax.set_ylabel("landmark observed", color=TEXT, fontsize=13)
    ax.set_ylim(0.4, len(d["landmarks"]) + 0.6)
    counts = np.bincount(m[:, 1].astype(int))[1:]
    bar.barh(range(1, len(counts) + 1), counts, color=ODOM, height=0.6)
    for k, c in enumerate(counts, 1):
        bar.text(c + 0.5, k, str(c), color=TEXT, va="center", fontsize=12)
    bar.set_ylim(0.4, len(counts) + 0.6)
    bar.set_yticks(range(1, len(counts) + 1))
    bar.set_yticklabels([])
    bar.set_xlim(0, counts.max() * 1.25)
    bar.set_xlabel("observations", color=TEXT, fontsize=13)
    fig.tight_layout()
    save(fig, "code_observation_log.png")
    return counts


def figure_pattern(d):
    n, pose_dims = d["n"], d["poseDims"]
    pat = np.zeros((n, n))
    pat[np.array(d["patternRows"]) - 1, np.array(d["patternCols"]) - 1] = 1
    cmap = ListedColormap([BG, CELL_ON])
    fig = plt.figure(figsize=(14, 7.2), facecolor=BG)
    a1 = fig.add_axes([0.04, 0.10, 0.44, 0.80])
    a2 = fig.add_axes([0.56, 0.10, 0.40, 0.80])
    a1.imshow(pat, cmap=cmap, interpolation="nearest", vmin=0, vmax=1)
    a1.axvline(pose_dims - 0.5, color=OBS, linewidth=2.2)
    a1.axhline(pose_dims - 0.5, color=OBS, linewidth=2.2)
    a1.set_title("")
    a1.text(pose_dims / 2, -14, f"{d['numPoses']} poses (3 variables each)", color=TEXT, ha="center", fontsize=12)
    a1.text(pose_dims + (n - pose_dims) / 2, -14, "6 landmarks", color=OBS, ha="center", fontsize=12)
    a1.text(n / 2, n + 26, f"all {n} unknowns: {100 * pat.mean():.1f}% of the entries are non-zero", color=TEXT,
            ha="center", fontsize=14)
    k = 18
    a2.pcolormesh(np.arange(k + 1), np.arange(k + 1), pat[:k, :k], cmap=cmap, edgecolors=GRID, linewidth=0.5, vmin=0, vmax=1)
    a2.set_ylim(k, 0)
    a2.set_aspect("equal")
    for b in range(0, k + 1, 3):
        a2.axvline(b, color=TEXT, linewidth=1.8)
        a2.axhline(b, color=TEXT, linewidth=1.8)
    a2.set_xticks([3 * i + 1.5 for i in range(k // 3)])
    a2.set_xticklabels([rf"pose {i + 1}" for i in range(k // 3)], color=TEXT, fontsize=11)
    a2.set_yticks([3 * i + 1.5 for i in range(k // 3)])
    a2.set_yticklabels([rf"pose {i + 1}" for i in range(k // 3)], color=TEXT, fontsize=11)
    a2.xaxis.tick_top()
    a2.tick_params(length=0)
    a2.set_title("zoom: the first six poses, in 3x3 blocks", color=TEXT, fontsize=14, pad=34)
    for spine in a2.spines.values():
        spine.set_visible(False)
    a1.axis("off")
    save(fig, "code_system_pattern.png")
    return pat


def figure_convergence(d):
    cost = list(d["costBefore"]) + [d["costAfter"]]
    step = d["maxStep"]
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5.2), facecolor=BG)
    style(a)
    style(b)
    a.semilogy(range(len(cost)), cost, color=ODOM, marker="o", markersize=9, linewidth=2.8)
    for i, c in enumerate(cost):
        a.annotate(f"{c:,.1f}", (i, c), textcoords="offset points", xytext=(10, 8), color=TEXT, fontsize=12)
    a.set_xticks(range(len(cost)))
    a.set_xticklabels(["initial\nguess"] + [f"after\niteration {i}" for i in range(1, len(cost))], color=TEXT, fontsize=11)
    a.set_ylabel(r"total weighted error $\sum e^\top\Omega\,e$", color=TEXT, fontsize=13)
    b.semilogy(range(1, len(step) + 1), step, color=OBS, marker="s", markersize=9, linewidth=2.8)
    for i, s in enumerate(step, 1):
        b.annotate(f"{s:.2g}", (i, s), textcoords="offset points", xytext=(10, 8), color=TEXT, fontsize=12)
    b.set_xticks(range(1, len(step) + 1))
    b.set_xlabel("Gauss-Newton iteration", color=TEXT, fontsize=13)
    b.set_ylabel(r"largest change to any variable, $\max|dx|$", color=TEXT, fontsize=13)
    a.set_xlim(-0.3, len(cost) - 0.7)
    fig.tight_layout()
    save(fig, "code_convergence.png")


def figure_errors(d):
    tr = np.array(d["truePoses"])
    e_dr = pos_error(d["deadReckoned"], tr)
    e_op = pos_error(d["optimized"], tr)
    lm = np.array(d["landmarks"])[np.array(d["observedIds"]) - 1]
    l_naive = np.hypot(*(np.array(d["landmarkEstimates"]) - lm).T)
    l_opt = np.hypot(*(np.array(d["optimizedLandmarks"]) - lm).T)
    fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5.2), facecolor=BG, gridspec_kw={"width_ratios": [3, 2]})
    style(a)
    style(b)
    t = np.arange(1, len(e_dr) + 1)
    a.plot(t, e_dr, color=CONFLICT, linewidth=2.6, label="dead reckoning")
    a.plot(t, e_op, color=GREEN, linewidth=2.6, label="GraphSLAM")
    a.set_xlabel("pose index $t$", color=TEXT, fontsize=13)
    a.set_ylabel("position error (m)", color=TEXT, fontsize=13)
    a.set_ylim(bottom=0)
    a.legend(facecolor=BG, edgecolor=GRID, labelcolor=TEXT, fontsize=12, loc="upper left")
    x = np.arange(len(l_naive))
    b.bar(x - 0.19, l_naive, width=0.38, color=TRUTH, label="first sighting (initial guess)")
    b.bar(x + 0.19, l_opt, width=0.38, color=GREEN, label="GraphSLAM")
    b.set_xticks(x)
    b.set_xticklabels([rf"$L_{k}$" for k in range(1, len(x) + 1)], color=TEXT, fontsize=13)
    b.set_ylabel("landmark position error (m)", color=TEXT, fontsize=13)
    b.legend(facecolor=BG, edgecolor=GRID, labelcolor=TEXT, fontsize=11, loc="upper left")
    b.set_ylim(0, max(l_naive.max(), l_opt.max()) * 1.45)
    fig.tight_layout()
    save(fig, "code_errors.png")
    return e_dr, e_op, l_naive, l_opt


def main():
    d = load()
    counts = figure_observations(d)
    pat = figure_pattern(d)
    figure_convergence(d)
    e_dr, e_op, l_naive, l_opt = figure_errors(d)

    n = d["n"]
    stats = {
        "seed": d["seed"], "poses": d["numPoses"], "meas": d["numMeasurements"], "n": n,
        "mean_dr": e_dr.mean(), "mean_opt": e_op.mean(), "final_dr": e_dr[-1], "final_opt": e_op[-1],
        "max_dr": e_dr.max(), "max_opt": e_op.max(),
        "lm_naive": np.round(l_naive, 3).tolist(), "lm_opt": np.round(l_opt, 3).tolist(),
        "counts": counts.tolist(), "nnzH": int(pat.sum()), "density_pct": 100 * pat.mean(),
        "lower_nnz": int(np.tril(pat).sum()), "dense_lower": n * (n + 1) // 2,
        "chol_natural": d["nnzCholNatural"], "chol_symamd": d["nnzCholSymamd"],
        "chol_lm_first": d["nnzCholLandmarksFirst"],
        "cost": [round(c, 1) for c in list(d["costBefore"]) + [d["costAfter"]]],
        "max_step": [float(f"{s:.3g}") for s in d["maxStep"]],
        "omega_motion": [round(w, 1) for w in d["omegaMotion"]], "omega_meas": [round(w, 1) for w in d["omegaMeas"]],
    }
    if "--numbers" in sys.argv:
        for k, v in stats.items():
            print(f"{k}: {v}")
        return

    assert stats["seed"] == 4 and stats["poses"] == 151 and stats["meas"] == 150 and n == 465
    assert np.array_equal(pat, pat.T)
    assert [round(stats[k], 3) for k in ("mean_dr", "mean_opt", "final_dr", "final_opt", "max_dr", "max_opt")] == \
        [0.099, 0.046, 0.202, 0.031, 0.237, 0.089], stats
    assert stats["lm_naive"] == [0.05, 0.073, 0.074, 0.118, 0.028, 0.063]
    assert stats["lm_opt"] == [0.037, 0.052, 0.049, 0.044, 0.038, 0.055]
    assert stats["counts"] == [30, 26, 21, 23, 21, 29]
    assert (stats["nnzH"], stats["lower_nnz"], stats["dense_lower"]) == (4677, 2571, 108345)
    assert round(stats["density_pct"], 2) == 2.16
    assert (stats["chol_natural"], stats["chol_symamd"], stats["chol_lm_first"]) == (7106, 5342, 96086)
    assert stats["cost"] == [8470.7, 638.1, 294.5, 258.6, 258.5, 258.5]
    assert stats["max_step"] == [0.278, 0.091, 0.0142, 0.00161, 0.000167]
    assert stats["omega_motion"] == [4444.4, 4444.4, 328280.6] and stats["omega_meas"] == [400.0, 820.7]
    first_moved = np.abs(np.array(d["optimized"])[0] - np.array(d["deadReckoned"])[0]).max()
    assert first_moved < 1e-5
    assert abs(first_moved - 1.4e-6) < 0.05e-6
    # facts quoted in graphslam_code_walkthrough.md
    assert int(pat[d["poseDims"]].sum()) == 92              # row of the first landmark's x coordinate
    assert d["costMotionFirst"] == 0.0                        # motion constraints are exact at the initial guess
    costs = list(d["costBefore"]) + [d["costAfter"]]
    assert round(costs[3], 2) == 258.64 and round(costs[3] - costs[4], 2) == 0.18 and round(costs[4] - costs[5], 4) == 0.0001
    assert round(costs[0] / costs[1]) == 13
    ratios = [a / b for a, b in zip(d["maxStep"][:-1], d["maxStep"][1:])]
    assert 3 <= min(ratios) and max(ratios) < 10, ratios
    assert round(1 / np.sqrt(d["omegaMotion"][2]) / d["dt"], 4) == round(d["sigma_w_odom"], 4)
    assert abs(1 / np.sqrt(1e6) - 0.001) < 1e-12
    print("Generated 4 figures in", HERE)


if __name__ == "__main__":
    main()
