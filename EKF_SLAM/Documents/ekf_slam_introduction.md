# Introduction to EKF SLAM

This is the first of a series of documents about EKF SLAM. It assumes no prior experience with Kalman filters: it builds up from a single noisy number, to a full filter that estimates a robot's pose and a map of landmarks together. Further documents will work through a complete 2D example by hand and walk through this repository's MATLAB implementation, the way the [GraphSLAM series](../../Graph_SLAM/Documents/graphslam_introduction.md) does for its own method.

## What Is SLAM?

A robot that moves through an unknown place needs two things at once: a **map** of the place, and its own **pose** in that map. Each one is needed to compute the other:

- To add an observation to the map, the robot must know where it was when it made it.
- To work out where it is, the robot must compare what it sees against a map.

**Simultaneous localization and mapping (SLAM)** solves both together. The robot has two kinds of information to work with: **odometry**, which measures how the robot moved but drifts over time, and **observations**, which measure where a landmark sits relative to the robot but are also noisy. Neither can be trusted exactly, and SLAM is the problem of combining them into the single most consistent estimate.

**EKF SLAM** is one way to solve this problem, built around the **extended Kalman filter (EKF)**. If you haven't met a Kalman filter before, the next two sections build one up from scratch, starting with a single noisy number and nothing else.

---

## A Single Noisy Number

Suppose you're trying to estimate some quantity — a position along a hallway, say — and all you have is a noisy sensor. A single reading is rarely something you should trust completely, so rather than keeping just one number, a Kalman filter keeps a **belief**: a best guess $\mu$ (the mean), and a **variance** $\sigma^2$ that says how sure it is of that guess. Together they describe a bell-shaped curve — a Gaussian — centered at $\mu$, and as wide as $\sigma^2$ says it should be.

### Combining Two Beliefs

Suppose you already have a belief about this quantity, and a brand new, independent measurement arrives. You now have two separate guesses about the same thing — your prior belief, and the measurement — each with its own mean and variance. Intuitively, the right way to combine them is some kind of average, but not a plain 50/50 average: an estimate with a *small* variance is more trustworthy, so it should pull the combined answer harder toward itself.

That intuition turns out to be exactly correct. If you have two independent Gaussian estimates of the same quantity, with means $\mu_1,\mu_2$ and variances $\sigma_1^2,\sigma_2^2$, the single best combined estimate is a **variance-weighted average**:

```math
\mu_{\text{post}}=\frac{\sigma_2^2\mu_1+\sigma_1^2\mu_2}{\sigma_1^2+\sigma_2^2},
\qquad
\sigma_{\text{post}}^2=\frac{\sigma_1^2\sigma_2^2}{\sigma_1^2+\sigma_2^2}.
```

It's usually written a different, more convenient way, by first defining the **Kalman gain**,

```math
K=\frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2},
```

which rewrites the combination as

```math
\boxed{\mu_{\text{post}}=\mu_1+K(\mu_2-\mu_1),\qquad \sigma_{\text{post}}^2=(1-K)\sigma_1^2.}
```

$K$ is always between $0$ and $1$. The posterior mean starts at $\mu_1$ and moves toward $\mu_2$ by a fraction $K$ of the distance between them — so a large $K$ pulls the estimate strongly toward the new measurement, and a small $K$ barely moves it at all. Notice what makes $K$ large: it grows whenever $\sigma_1^2$ (the prior's variance) is large relative to $\sigma_2^2$ (the measurement's variance). In other words, the estimate always shifts toward whichever of the two is *more confident* — exactly the behavior we wanted.

As a concrete example, say the prior belief is $\mu_1=0$ with variance $\sigma_1^2=5$, and a measurement arrives of $\mu_2=3$ with variance $\sigma_2^2=2$ — a more confident reading, since its variance is smaller. Then

```math
K=\frac{5}{5+2}\approx0.714,
\qquad
\mu_{\text{post}}=0+0.714(3-0)\approx2.14,
\qquad
\sigma_{\text{post}}^2=(1-0.714)(5)\approx1.43.
```

The posterior sits closer to the measurement than to the prior, because the measurement was more confident, and its variance is smaller than *either* input — combining two independent estimates always leaves you more certain than either one alone.

![Three bell curves on the same axis: a wide blue curve centered at 0 labeled the predicted belief, a narrower gold curve centered at 3 labeled the measurement, and an even narrower purple curve between them around 2.14 labeled the posterior belief, with the area under the purple curve shaded.](images/kalman_filter_fusion.png)

### Predict, Then Correct

The example above fused two beliefs that already existed, but a real system is also *moving* or *changing* between measurements, and that has to be accounted for too. Every Kalman filter repeats a two-step cycle:

- **Predict.** Before a new measurement arrives, advance the belief forward using whatever you know about how the quantity changes over time (a **motion model**, or more generally a **process model**). Since the motion itself is never perfectly known, this step also *grows* the variance — you become a little less sure, simply because time has passed.
- **Correct.** When a measurement arrives, fuse it into the predicted belief using the weighted average above. This *shrinks* the variance again, because a new, independent piece of information has arrived.

Uncertainty grows during predict and shrinks during correct, and the filter's belief tracks the true quantity by repeating the cycle indefinitely: predict, correct, predict, correct, one time step at a time.

---

## From One Number to Many

A robot's state is rarely just one number — a 2D pose has an $x$, a $y$, and a heading $\theta$, all at once. Generalizing the filter to several numbers means replacing the mean $\mu$ with a **state vector** $\mathbf{x}$, and replacing the variance $\sigma^2$ with a **covariance matrix** $P$: a square matrix whose diagonal holds the variance of each individual state variable, and whose off-diagonal entries capture how pairs of variables are correlated — how learning something about one tells you something about another.

The predict and correct steps generalize the same way:

```math
\textbf{Predict:}\quad
\mathbf{x}'=f(\mathbf{x},\mathbf{u}),
\qquad
P'=FPF^{\mathsf{T}}+R
```

```math
\textbf{Correct:}\quad
\mathbf{y}=\mathbf{z}-h(\mathbf{x}'),
\qquad
S=HP'H^{\mathsf{T}}+Q,
\qquad
K=P'H^{\mathsf{T}}S^{-1},
\qquad
\mathbf{x}=\mathbf{x}'+K\mathbf{y},
\qquad
P=(I-KH)P'
```

where:

- $f$ is the process model (how the state evolves given a control input $\mathbf{u}$), and $h$ is the measurement model (what a sensor reading should look like, given the state).
- $R$ and $Q$ are the process noise and measurement noise covariances — the multi-variable equivalents of $\sigma_1^2$ and $\sigma_2^2$ from before.
- $\mathbf{y}$, the difference between the actual and the predicted measurement, is called the **innovation** — the multi-variable equivalent of $\mu_2-\mu_1$.
- $K$ again decides how much the innovation should move the state estimate, exactly like before, just expressed with matrices instead of a single ratio.

For many real systems, $f$ and $h$ are **nonlinear** — a heading $\theta$ usually appears inside a $\sin$ or $\cos$, for instance. $F$ and $H$ above are the **Jacobians** of $f$ and $h$: matrices of partial derivatives that locally approximate the nonlinear functions with a linear one, evaluated at the current estimate. Using this local linear approximation instead of the exact nonlinear function is what makes this the **extended** Kalman filter.

---

## EKF SLAM: Augmenting the State with the Map

Ordinary EKF localization only estimates the robot's own pose. EKF SLAM estimates the map as well, by folding every landmark's position directly into the same state vector:

```math
\mathbf{x}=
\begin{bmatrix}
x_r\\y_r\\\theta_r\\L_{0x}\\L_{0y}\\L_{1x}\\L_{1y}\\\vdots\\L_{Nx}\\L_{Ny}
\end{bmatrix}
```

For $N$ landmarks, this is a single vector of length $3+2N$: three entries for the robot pose, and two more for every landmark's $(x,y)$ position. The covariance matrix $P$ grows to match, becoming $(3+2N)\times(3+2N)$ — and because it's symmetric, it's cleanest to understand by splitting it into blocks:

- $P_{rr}$, a $3\times3$ block in the top-left corner, is the robot's own pose covariance — exactly what ordinary EKF localization would track on its own.
- $P_{rl}$ and $P_{lr}=P_{rl}^{\mathsf{T}}$, each $3\times2N$, capture the correlation between the robot's pose and every landmark. This matters because every landmark's estimated position was computed from the robot's pose *at the time it was observed* — if that pose estimate later turns out to be off, the landmark's estimate should move too, and this block is what remembers that relationship.
- $P_{ll}$, the remaining $2N\times2N$ block, holds each landmark's own variance along its diagonal 2×2 blocks, and the correlation *between pairs of landmarks* everywhere else. These cross-landmark terms start at zero — two landmarks that have never been seen from the same uncertain pose have no reason to be correlated yet — but they fill in over time, which turns out to be the key to why SLAM works better than mapping each landmark independently.

![A 9 by 9 grid representing the covariance matrix for a robot pose and three landmarks, labeled x, y, theta, L0x, L0y, L1x, L1y, L2x, L2y along both axes. The top-left 3 by 3 block is colored blue for the robot-robot block. The row and column band for the robot against every landmark is colored purple for the robot-landmark cross blocks. Along the landmark-landmark region, the 2 by 2 blocks on the diagonal for each individual landmark are colored gold, and every remaining off-diagonal 2 by 2 block between two different landmarks is colored a muted blue-gray, representing correlation between landmarks.](images/ekf_slam_covariance_blocks.png)

---

## What Changes During Predict and Correct

Splitting $P$ into these blocks makes it possible to see exactly what each step of the filter touches, rather than treating $P$ as one large opaque matrix.

### Predict

Only the robot moves — landmarks stay put between one time step and the next. This shows up in the Jacobian $F$: it's the identity matrix everywhere except in the $3\times3$ robot block, where it holds the Jacobian of the motion model, call it $F_x$. Multiplying this structured $F$ through $P'=FPF^{\mathsf{T}}+R$ shows exactly where the effect lands:

```math
P_{rr}'=F_xP_{rr}F_x^{\mathsf{T}}+R_x,
\qquad
P_{rl}'=F_xP_{rl},
\qquad
P_{ll}'=P_{ll}.
```

The robot block changes the most — it's transformed by $F_x$ on both sides and gains new process noise $R_x$. The cross-correlation blocks change too, but more gently, scaled by $F_x$ on one side only. The landmark-landmark block is completely untouched: landmarks don't move, so nothing new happens to how they relate to each other during a predict step.

### Correct

When the robot observes one landmark, say $L_1$, the measurement model $h$ only directly depends on the robot pose and $L_1$'s position — a selection matrix picks out just those entries, so $H$ is zero everywhere else. You might expect the correction to only touch those same rows and columns of $P$. It doesn't: $K=P'H^{\mathsf{T}}S^{-1}$ multiplies through *every column* of $P'$, not just the ones $H$ reads from directly. Because the robot is already correlated with every landmark through $P_{rl}$, and $L_1$ may already be correlated with other landmarks through $P_{ll}$, the correction ripples outward through those correlations and can adjust the entire map — including landmarks the robot isn't even looking at right now.

![Two side-by-side 9 by 9 grids for the same state as before. The left grid, titled predict, highlights only the robot block in blue and the robot-landmark cross blocks in purple, leaving the landmark-landmark block a flat unchanged dark gray. The right grid, titled correct observing L1, highlights the robot block in blue and the L1 diagonal block in gold as the two directly observed blocks, while every remaining block of the matrix, including landmarks other than L1 and their correlations, is colored blue-gray to show it is still updated indirectly through correlation.](images/ekf_slam_predict_correct.png)

This is the central mechanism that makes EKF SLAM more than just independently tracking each landmark: once two landmarks have both been seen from the same uncertain robot pose, they become linked, and correcting one tends to improve the other, even without observing it directly.

---

## Filtering vs. Batch Estimation

EKF SLAM is a **filter**: it keeps only the current pose and map, folding every past measurement into $P$ and then discarding it. This is the same distinction the [Introduction to GraphSLAM](../../Graph_SLAM/Documents/graphslam_introduction.md#an-offline-batch-solver) draws between filtering and batch, offline solvers like GraphSLAM, which keep every pose ever visited and can revise the whole trajectory at once:

| | Filtering (EKF SLAM) | GraphSLAM |
|---|---|---|
| Estimates | The current pose and the map | Every pose and the map |
| Past poses | Summarized and discarded | Kept and re-estimated |
| New information | Updates the current estimate | Can change the whole trajectory |
| Typical use | Online, as the robot moves | Offline, over a collected data set |

EKF SLAM's cost grows with the *size of the map*: $P$ is $(3+2N)^2$ entries, and updating it at every step is at least that expensive, which is why the number of landmarks a plain EKF SLAM implementation can track is limited in practice. Its advantage is that it runs online, producing an estimate after every single measurement, with no need to store the robot's whole history.

---

## How These Documents Are Organized

This document is the first in a planned series on EKF SLAM. Future documents will work through a full 2D example by hand — building $\mathbf{x}$, $P$, and the Jacobians $F$ and $H$ explicitly — and then walk through how [SIMULATION_EKFSLAM.m](../SIMULATION_EKFSLAM.m), this repository's MATLAB implementation, puts all of it into code.
