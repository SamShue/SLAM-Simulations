"""Shared model for the loop-closure and sparse-solver examples.

A robot drives once around a 4 m x 4 m square (9 poses, x0..x8, ending where it started) and sees
two landmarks. The odometry is biased (it slightly over-reports distance and turn angle), so dead
reckoning drifts. The last pose x8 sees landmark L0 again, which closes the loop.

State layout: [x0 y0 th0 | ... | x8 y8 th8 | L0x L0y | L1x L1y]  (31 variables)
Measurements and the linearization are exactly those of graphslam_2d_example.md.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_2d_figures import W_LM, W_ODOM, W_PRIOR, compose, d_rot_t, rot, wrap  # noqa: E402

N_POSES, N_LM = 9, 2
DIM = 3 * N_POSES + 2 * N_LM
NODE_START = [3 * i for i in range(N_POSES)] + [3 * N_POSES + 2 * k for k in range(N_LM)] + [DIM]
NODE_NAMES = [f"x{i}" for i in range(N_POSES)] + [f"L{k}" for k in range(N_LM)]

TRUE_POSES = np.array([
    [0, 0, 0], [2, 0, 0], [4, 0, np.pi / 2], [4, 2, np.pi / 2], [4, 4, np.pi],
    [2, 4, np.pi], [0, 4, -np.pi / 2], [0, 2, -np.pi / 2], [0, 0, 0],
], dtype=float)
TRUE_LM = np.array([[2.0, 1.5], [3.0, 3.0]])

# Hand-picked, deterministic measurement errors (no random numbers, so results never change).
ODOM_SCALE, ODOM_TURN_BIAS = 1.01, 0.025
ODOM_ERR = np.array([
    [0.010, 0.010, 0.005], [-0.020, -0.010, -0.010], [0.015, 0.020, 0.010], [0.000, -0.015, 0.005],
    [0.020, 0.010, -0.005], [-0.010, 0.000, 0.010], [0.010, -0.020, -0.005], [-0.015, 0.010, 0.000],
])
# (pose index, landmark index, error added to the true observation)
OBS_SPEC = [(0, 0, (0.02, -0.03)), (1, 0, (-0.03, 0.02)), (3, 1, (0.02, 0.01)), (4, 1, (-0.02, 0.03)),
            (8, 0, (-0.02, 0.03))]


def pose_slice(i):
    return slice(3 * i, 3 * i + 3)


def lm_slice(k):
    s = 3 * N_POSES + 2 * k
    return slice(s, s + 2)


def node_slice(n):
    return slice(NODE_START[n], NODE_START[n + 1])


def relative_pose(p, q):
    return np.array([*(rot(p[2]).T @ (q[:2] - p[:2])), wrap(q[2] - p[2])])


def build_measurements():
    """(kind, i, j, z, W): prior, 8 odometry, 4 local observations, then the loop-closing observation."""
    ms = [("prior", 0, None, np.zeros(3), W_PRIOR)]
    for k in range(N_POSES - 1):
        z = relative_pose(TRUE_POSES[k], TRUE_POSES[k + 1])
        z[0] *= ODOM_SCALE
        z = z + ODOM_ERR[k]
        z[2] = wrap(z[2] + ODOM_TURN_BIAS)
        ms.append(("odom", k, k + 1, z, W_ODOM))
    for i, lm, err in OBS_SPEC:
        z = rot(TRUE_POSES[i][2]).T @ (TRUE_LM[lm] - TRUE_POSES[i][:2]) + np.array(err)
        ms.append(("lm", i, lm, z, W_LM))
    return ms


MEASUREMENTS = build_measurements()
# Pose closure: x8 recognises it is back at x0 and measures x0's pose relative to itself (like odometry,
# but between non-consecutive poses). Landmark closure: x8 sees L0 again (constrains only 2 of 3 DoF).
POSE_CLOSURE_ERR = np.array([0.02, -0.015, 0.01])
MEASUREMENTS.append(("odom", N_POSES - 1, 0, relative_pose(TRUE_POSES[-1], TRUE_POSES[0]) + POSE_CLOSURE_ERR,
                     W_ODOM))
LANDMARK_CLOSURE = len(MEASUREMENTS) - 2   # the x8 -> L0 observation built by build_measurements
POSE_CLOSURE = len(MEASUREMENTS) - 1
LOCAL = list(range(LANDMARK_CLOSURE))
WITH_POSE_CLOSURE = LOCAL + [POSE_CLOSURE]
WITH_LANDMARK_CLOSURE = LOCAL + [LANDMARK_CLOSURE]


def initial_guess():
    """Dead reckoning for poses; first observation for each landmark."""
    mu = np.zeros(DIM)
    for k in range(N_POSES - 1):
        mu[pose_slice(k + 1)] = compose(mu[pose_slice(k)], MEASUREMENTS[1 + k][3])
    seen = set()
    for kind, i, j, z, _ in MEASUREMENTS:
        if kind == "lm" and j not in seen:
            seen.add(j)
            p = mu[pose_slice(i)]
            mu[lm_slice(j)] = p[:2] + rot(p[2]) @ z
    return mu


def linearize(m, mu):
    """[(slice, J)] pieces, h(mu) and z_hat = z - h + J mu for one measurement."""
    kind, i, j, z, _ = m
    if kind == "prior":
        s = pose_slice(i)
        zhat = z.copy()
        zhat[:2] = z[:2] - mu[s][:2] + mu[s][:2]
        zhat[2] = wrap(z[2] - mu[s][2]) + mu[s][2]
        return [(s, np.eye(3))], mu[s].copy(), zhat
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
    return [(si, ji), (sl, jl)], h, z - h + ji @ pi + jl @ lm


def build_system(indices, mu):
    omega, xi = np.zeros((DIM, DIM)), np.zeros(DIM)
    for k in indices:
        m = MEASUREMENTS[k]
        pieces, _, zhat = linearize(m, mu)
        for a, ja in pieces:
            xi[a] += ja.T @ m[4] @ zhat
            for b, jb in pieces:
                omega[a, b] += ja.T @ m[4] @ jb
    return omega, xi


def solve(mu0, indices, iterations=30):
    """Solve, relinearize, repeat. Returns every iterate."""
    history = [mu0.copy()]
    mu = mu0.copy()
    for _ in range(iterations):
        omega, xi = build_system(indices, mu)
        new = np.linalg.solve(omega, xi)
        for i in range(N_POSES):
            new[3 * i + 2] = wrap(new[3 * i + 2])
        history.append(new.copy())
        done = np.abs(new - mu).max() < 1e-10
        mu = new
        if done:
            break
    return history


def pose_errors(mu):
    """Position error (m) and heading error (rad) of every pose against the true square."""
    pos = np.array([np.linalg.norm(mu[pose_slice(i)][:2] - TRUE_POSES[i][:2]) for i in range(N_POSES)])
    head = np.array([abs(wrap(mu[pose_slice(i)][2] - TRUE_POSES[i][2])) for i in range(N_POSES)])
    return pos, head


# ----------------------------------------------------------------------- sparsity analysis

def scalar_adjacency(omega, tol=1e-9):
    n = omega.shape[0]
    return [set(np.flatnonzero(np.abs(omega[i]) > tol).tolist()) - {i} for i in range(n)]


def symbolic_nnz(adj, order):
    """Number of non-zeros (lower triangle, diagonal included) of the Cholesky factor for an ordering."""
    adj = [set(a) for a in adj]
    done = set()
    total = 0
    for v in order:
        nbrs = [u for u in adj[v] if u not in done]
        total += 1 + len(nbrs)
        for a in nbrs:
            adj[a].update(nbrs)
            adj[a].discard(a)
        done.add(v)
    return total


def min_degree_order(adj):
    """Greedy minimum-degree ordering (ties broken by index)."""
    adj = [set(a) for a in adj]
    remaining = set(range(len(adj)))
    order = []
    while remaining:
        v = min(remaining, key=lambda u: (len(adj[u] & remaining), u))
        nbrs = adj[v] & remaining - {v}
        for a in nbrs:
            adj[a].update(nbrs)
            adj[a].discard(a)
        order.append(v)
        remaining.discard(v)
    return order


def cholesky_pattern(omega, order, tol=1e-9):
    l = np.linalg.cholesky(omega[np.ix_(order, order)])
    return np.abs(l) > tol


def lower_nnz(omega, tol=1e-9):
    return int((np.abs(np.tril(omega)) > tol).sum())


def appearance_order(indices):
    """Scalar variable order in which a SLAM system would add its nodes: each pose when it is created,
    each landmark right after the first pose that sees it."""
    first_seen = {}
    for k in indices:
        kind, i, j, _, _ = MEASUREMENTS[k]
        if kind == "lm":
            first_seen.setdefault(j, i)
    order = []
    for pose in range(N_POSES):
        order += list(range(3 * pose, 3 * pose + 3))
        for lm, at in sorted(first_seen.items()):
            if at == pose:
                s = lm_slice(lm)
                order += list(range(s.start, s.stop))
    return order


def block_graph(sizes, edges):
    """Scalar adjacency of a block graph: each node is a clique, each edge a complete bipartite link."""
    starts = np.concatenate([[0], np.cumsum(sizes)])
    n = int(starts[-1])
    adj = [set() for _ in range(n)]
    for a in range(len(sizes)):
        idx = range(starts[a], starts[a + 1])
        for i in idx:
            adj[i].update(j for j in idx if j != i)
    for a, b in edges:
        for i in range(starts[a], starts[a + 1]):
            for j in range(starts[b], starts[b + 1]):
                adj[i].add(j)
                adj[j].add(i)
    return adj


def scaling_rows(sizes_list=(10, 50, 100, 200)):
    """Fill-in for chains, rings (loop closure) and a landmark seen from every pose."""
    rows = []
    for n in sizes_list:
        chain = [(i, i + 1) for i in range(n - 1)]
        pose_sizes = [3] * n
        row = {"poses": n, "n": 3 * n}
        row["dense"] = (3 * n) * (3 * n + 1) // 2
        adj = block_graph(pose_sizes, chain)
        row["chain_nnz_omega"] = (sum(len(a) for a in adj) + 3 * n) // 2
        row["chain_nnz_l"] = symbolic_nnz(adj, range(3 * n))
        ring = block_graph(pose_sizes, chain + [(0, n - 1)])
        row["ring_nnz_l_natural"] = symbolic_nnz(ring, range(3 * n))
        row["ring_nnz_l_mindeg"] = symbolic_nnz(ring, min_degree_order(ring))
        # one landmark (2 variables, node index n) seen from every pose
        hub = block_graph(pose_sizes + [2], chain + [(i, n) for i in range(n)])
        last = list(range(3 * n)) + [3 * n, 3 * n + 1]
        first = [3 * n, 3 * n + 1] + list(range(3 * n))
        row["hub_nnz_l_last"] = symbolic_nnz(hub, last)
        row["hub_nnz_l_first"] = symbolic_nnz(hub, first)
        rows.append(row)
    return rows


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True, linewidth=160)
    mu0 = initial_guess()
    err0, head0 = pose_errors(mu0)
    print("dead-reckoned position error per pose:", np.round(err0, 3))
    print("dead-reckoned heading error per pose :", np.round(head0, 3))
    for name, idx in [("no closure", LOCAL), ("pose closure", WITH_POSE_CLOSURE),
                      ("landmark closure", WITH_LANDMARK_CLOSURE)]:
        hist = solve(mu0, idx)
        e, hd = pose_errors(hist[-1])
        print(f"{name}: iters {len(hist) - 1}, max change {np.abs(hist[-1] - mu0).max():.4f}")
        print("   pos err ", np.round(e, 3))
        print("   head err", np.round(hd, 3))
