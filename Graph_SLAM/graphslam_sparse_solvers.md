# Solving the System Efficiently

This document continues [Loop Closure in GraphSLAM](graphslam_loop_closure.md) and uses the same 31-variable system. It assumes you have read the [2D GraphSLAM Example](graphslam_2d_example.md).

## Introduction

Every GraphSLAM example so far ended with the same step: solve $\Omega\mu=\xi$. With 7 or 13 or even 31 unknowns, any method works and the choice does not matter. Real maps are much larger. A robot that records a pose every few tenths of a metre can easily collect thousands of poses, and each pose adds three unknowns.

A dense solver treats $\Omega$ as an arbitrary matrix with $n$ unknowns. It stores $n^2$ numbers and needs about $n^3/3$ operations. For a map with $10{,}000$ poses, $n=30{,}000$, and that means about $7.2\text{ GB}$ just to hold $\Omega$ and about $9\times10^{12}$ operations for a single solve. We have to repeat that solve every time we relinearize.

The key observation is that $\Omega$ is **sparse**. A pose is only directly connected to a few neighbours, so almost all entries are zero. A solver that knows this can be thousands of times cheaper. This document shows how, and what the loop closure from the previous document does to it.

---

## $\Omega$ Is Sparse, but $\Omega^{-1}$ Is Not

In the loop-closure example, $22\%$ of the entries of the $31\times31$ matrix $\Omega$ are non-zero. As the map grows the fraction drops quickly, because each new pose adds only a few non-zero blocks, so the number of non-zeros grows in proportion to the number of poses, not their square:

| Poses | Unknowns | Dense entries | Non-zeros in $\Omega$ (chain) |
|---|---|---|---|
| 10 | 30 | 465 | 126 |
| 50 | 150 | 11,325 | 666 |
| 100 | 300 | 45,150 | 1,341 |
| 200 | 600 | 180,300 | 2,691 |

Counts are for the lower triangle, including the diagonal, of a chain of poses where each pose is linked to the next.

It is tempting to solve the system by computing $\mu=\Omega^{-1}\xi$, as we mentioned in the 1D example. For anything beyond a small example this is a poor idea, because **the inverse of a sparse matrix is generally dense**:

![Two matrix patterns for the loop-closure system. Omega, on the left, has non-zero entries only in a band near the diagonal plus a few extra blocks. Its inverse, on the right, has non-zero entries almost everywhere, with brighter cells where the entries are larger.](images/sparse_omega_inverse.png)

$\Omega$ records the direct constraints between variables. $\Omega^{-1}$ is the covariance, which describes how errors in different variables are correlated. Every pose is correlated with every other pose through the chain of odometry between them, so almost every entry of $\Omega^{-1}$ is non-zero. In this example, $92\%$ are, even though only $22\%$ of $\Omega$ is.

So computing the inverse would throw the sparsity away. We want to solve the equations without ever forming $\Omega^{-1}$.

---

## Cholesky Factorization

The information matrix $\Omega$ is symmetric and positive definite, as long as the prior has anchored the problem. For these matrices there is an efficient factorization, the **Cholesky factorization**:

```math
\Omega=LL^\top,
```

where $L$ is lower triangular. With the factor in hand, the solve takes two cheap steps:

1. **Forward substitution:** solve $Ly=\xi$ for $y$, starting from the first row.
2. **Back substitution:** solve $L^\top\mu=y$ for $\mu$, starting from the last row.

Each substitution only touches the non-zero entries of $L$, so the cost depends on how many non-zeros $L$ has.

Here is a small worked example. Four variables $a,\ b,\ c,\ h$, where $h$ is a "hub" linked to the other three. In SLAM terms, $h$ could be a landmark seen from three poses:

```math
\Omega=
\begin{bmatrix}
4&0&0&1\\
0&4&0&1\\
0&0&4&1\\
1&1&1&4
\end{bmatrix},
\qquad
\xi=
\begin{bmatrix}
2\\
3\\
4\\
1
\end{bmatrix}.
```

Its Cholesky factor is

```math
L=
\begin{bmatrix}
2&0&0&0\\
0&2&0&0\\
0&0&2&0\\
0.5&0.5&0.5&1.803
\end{bmatrix}.
```

Forward substitution solves $Ly=\xi$ one row at a time:

```math
y_1=\frac{2}{2}=1,\quad
y_2=\frac{3}{2}=1.5,\quad
y_3=\frac{4}{2}=2,\quad
y_4=\frac{1-0.5(1+1.5+2)}{1.803}=-0.693.
```

Back substitution solves $L^\top\mu=y$ from the last row upward:

```math
\mu_h=\frac{-0.693}{1.803}=-0.385,\quad
\mu_a=\frac{1-0.5\,\mu_h}{2}=0.596,\quad
\mu_b=0.846,\quad
\mu_c=1.096.
```

You can check that $\Omega\mu=\xi$. We never needed $\Omega^{-1}$.

---

## Fill-In and the Order of Elimination

Notice that the factor $L$ above has non-zeros in exactly the same places as the lower triangle of $\Omega$: no new non-zeros appeared. That was lucky. In general, factorization can create new non-zeros where $\Omega$ had zeros, which is called **fill-in**. Whether it does depends on the order in which the variables are eliminated.

The same matrix, with the hub eliminated **first** instead of last:

![Two ways of eliminating the same four-variable system. When the hub h is eliminated first, its three neighbours become connected to each other, which is shown as three red dashed edges, and the Cholesky factor has 10 non-zeros, three of which are fill-in. When the hub is eliminated last, no new edges appear, and the factor has only 7 non-zeros.](images/sparse_fill_star.png)

The graph picture makes it easy to see why:

> Eliminating a variable connects all of its neighbours to each other.

When the hub goes first, $a,\ b,\ c$ all lose their only connection to the rest of the graph, and they become connected to each other. These are the new edges and the new non-zeros. When the hub goes last, its neighbours have already been eliminated, so there is nothing left to connect.

The order of elimination does not change the solution. It only changes the cost.

### Why this matters in SLAM

The hub in the example is a landmark seen from many poses. The effect gets very large as the map grows. This table compares a landmark seen from **every** pose, with the landmark eliminated last or first:

| Poses | Dense entries | $L$, landmark last | $L$, landmark first |
|---|---|---|---|
| 10 | 528 | 204 | 528 |
| 50 | 11,628 | 1,044 | 11,628 |
| 100 | 45,753 | 2,094 | 45,753 |
| 200 | 181,503 | 4,194 | 181,503 |

This setup has the poses plus one landmark, so there are $3N+2$ unknowns, and "dense entries" is the size of the lower triangle of the full matrix. Eliminated first, the landmark makes $L$ completely dense. Eliminated last, $L$ stays sparse and grows in proportion to the number of poses. Both orders produce exactly the same answer.

---

## Ordering in the Loop-Closure Example

The loop-closure system has 31 unknowns, and a dense factor would have 496 non-zeros in its lower triangle. Here is how many non-zeros $L$ actually has for three different orders of the same variables, with the loop closure included. Fill-in is shown in yellow:

![Three Cholesky factors of the loop-closure system, drawn as lower-triangular patterns. With the state-vector order, where all poses come before the landmarks, the factor has 262 non-zeros and a large block of fill-in along the bottom rows. With the order of first appearance, where each landmark follows the pose that first sees it, it has 204. With a minimum-degree order it has 194.](images/sparse_cholesky_orders.png)

- **State-vector order** (all poses, then the landmarks): $262$ non-zeros. The landmarks come last, but $L_0$ is tied to the first two poses. Eliminating the poses in order drags that connection along the whole trajectory, which produces the large band of fill-in at the bottom.
- **Order of first appearance** (each landmark right after the first pose that sees it): $204$ non-zeros. This is the order in which a SLAM system naturally adds nodes to the graph.
- **Minimum degree** ($194$ non-zeros): a general-purpose heuristic that does not need to know about poses or landmarks. At each step it eliminates the variable with the fewest remaining neighbours.

### What the loop closure costs

Without the loop closure, the first-appearance order gives a factor with only $128$ non-zeros. With it, $204$. The closure ties $\mathbf{x}_8$ to $\mathbf{x}_0$, so eliminating each pose in turn passes a link to $\mathbf{x}_8$ along the chain, and the last block row of $L$ fills.

It costs more, but not much more. The cost of a loop closure is proportional to the length of the loop, not its square. Here is the same comparison for longer trajectories:

| Poses | $L$, chain | $L$, with a loop closure |
|---|---|---|
| 10 | 141 | 213 |
| 50 | 741 | 1,173 |
| 100 | 1,491 | 2,373 |
| 200 | 2,991 | 4,773 |

These counts are for a chain or ring of poses only, eliminated in chronological order.

Closing the loop adds about nine non-zeros per pose. Even with a loop closure over $200$ poses, the factor has $4{,}773$ non-zeros instead of the $180{,}300$ of a dense matrix.

Real solvers do not require you to choose the order by hand. They run a **fill-reducing ordering** algorithm on the sparsity pattern first. The common ones are minimum degree and its approximate versions (AMD and COLAMD), and **nested dissection**, which works better for very large maps.

---

## Eliminating the Landmarks: the Schur Complement

There is a second way to take advantage of the structure. Landmarks never connect directly to each other. They only connect to the poses that saw them. If we split the unknowns into poses $p$ and landmarks $l$, the information matrix has blocks

```math
\begin{bmatrix}
\Omega_{pp}&\Omega_{pl}\\
\Omega_{lp}&\Omega_{ll}
\end{bmatrix}
\begin{bmatrix}
\mu_p\\
\mu_l
\end{bmatrix}
=
\begin{bmatrix}
\xi_p\\
\xi_l
\end{bmatrix},
```

and $\Omega_{ll}$ is **block diagonal**, with one tiny $2\times2$ block per landmark. Inverting it is trivial.

That makes it cheap to eliminate all the landmarks first. We substitute $\mu_l$ out of the first row, which leaves a smaller system for the poses alone, called the **Schur complement**:

```math
\left(\Omega_{pp}-\Omega_{pl}\Omega_{ll}^{-1}\Omega_{lp}\right)\mu_p
=
\xi_p-\Omega_{pl}\Omega_{ll}^{-1}\xi_l.
```

After solving for the poses, the landmarks follow by back-substitution:

```math
\mu_l=\Omega_{ll}^{-1}\left(\xi_l-\Omega_{lp}\mu_p\right).
```

The result is exactly the same as solving the full system. In the example below, the two answers agree to about $10^{-13}$. This reduction is the one used in the original GraphSLAM algorithm.

### A landmark closure becomes a pose-pose closure

The previous document looked at the case where $\mathbf{x}_8$ re-observes $L_0$ instead of recognizing a place. Let us use that system here. The landmark $L_0$ is seen by $\mathbf{x}_0$, $\mathbf{x}_1$, and $\mathbf{x}_8$. Its own block is

```math
\Omega_{L_0L_0}=
\begin{bmatrix}
296.7&17.5\\
17.5&78.3
\end{bmatrix}.
```

Eliminating $L_0$ connects every pair of poses that saw it. For example, there was no direct link between $\mathbf{x}_0$ and $\mathbf{x}_8$ before. Now there is, with the block

```math
-\Omega_{x_0L_0}\,\Omega_{L_0L_0}^{-1}\,\Omega_{L_0x_8}=
\begin{bmatrix}
-31.9&-3.1&53.6\\
-3.1&-8.8&-8.3\\
40.6&-13.4&-95.5
\end{bmatrix}.
```

![Left: the information matrix of the system in which x8 re-observes L0, with the landmark rows and columns outlined in yellow. Right: the 27 by 27 pose-only matrix after eliminating the landmarks, with four new pose-pose blocks, between x0 and x8 and between x1 and x8, highlighted in yellow.](images/sparse_schur.png)

This is the same fill-in rule as before, seen from the landmark's side: eliminating a landmark that was seen by several poses couples those poses. It also shows what a landmark closure really is. After the landmark is eliminated, re-observing $L_0$ from $\mathbf{x}_8$ is equivalent to a loop closure between poses $\mathbf{x}_8$, $\mathbf{x}_0$, and $\mathbf{x}_1$.

---

## In Practice

- **Gauss-Newton with sparse solves.** Each iteration rebuilds $\Omega$ and $\xi$ at the current estimate and solves a sparse system. The pattern of non-zeros does not change between iterations, only the numbers do. So the expensive ordering and symbolic analysis can be done once and reused.
- **Libraries.** You rarely write this yourself. CHOLMOD (part of SuiteSparse) and Eigen's sparse Cholesky solvers include fill-reducing orderings. MATLAB's `chol` and backslash operator on a sparse symmetric positive definite matrix use a sparse Cholesky factorization with a fill-reducing ordering. In Python, SciPy has sparse solvers such as `spsolve` and `splu`, which use sparse LU instead of Cholesky. The `scikit-sparse` package wraps CHOLMOD. Pose-graph and SLAM libraries such as g2o, GTSAM, and Ceres are built on top of sparse solvers like these.
- **Iterative alternatives.** Conjugate gradient solves $\Omega\mu=\xi$ using only products with $\Omega$, which suits very large problems, but it needs a good preconditioner to converge quickly.
- **Incremental solvers.** When new measurements arrive one at a time, methods such as iSAM update the existing factorization instead of recomputing it from scratch. This lets GraphSLAM run online, which this offline batch walkthrough has not needed.

---

## Summary

- $\Omega$ is sparse because each pose connects only to a few neighbours. $\Omega^{-1}$ is dense, so we never compute it.
- Cholesky factorization $\Omega=LL^\top$ followed by forward and back substitution solves the system using only the non-zeros of $L$.
- Factorization creates fill-in, and the amount depends on the elimination order. Eliminating a variable connects all of its neighbours.
- A landmark seen from many poses is a hub. Ordering it badly makes $L$ dense, and ordering it well keeps $L$ sparse.
- A loop closure adds off-diagonal blocks, but its fill-in grows only in proportion to the length of the loop.
- Eliminating the landmarks with the Schur complement gives a smaller, pose-only system with the same solution.
