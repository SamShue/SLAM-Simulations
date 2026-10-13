# GraphSLAM in Code: A Walkthrough of the MATLAB Simulation

This document pairs the earlier GraphSLAM documents with the code in this repository. It assumes you have read the [Introduction](graphslam_introduction.md), the [1D example](graphslam_1d_example.md), and the [2D example](graphslam_2d_example.md), since it uses their vocabulary: nodes, residuals, weights, Jacobians, and the information matrix $\Omega$ and vector $\xi$.

The code is in [SIMULATION_GraphSLAM.m](../SIMULATION_GraphSLAM.m). It simulates a robot driving through a field of landmarks, and then solves for the whole trajectory and map with GraphSLAM. The simulation helpers are in the [common](../../common) folder.

Every code listing below is copied from that file, and [check_code_walkthrough_snippets.py](images/check_code_walkthrough_snippets.py) checks that each one still matches the source. The numbers and figures come from running the real simulation.

## Running It

Open [SIMULATION_GraphSLAM.m](../SIMULATION_GraphSLAM.m) in MATLAB and run it. A window shows the robot driving a path between five waypoints while it observes landmarks. When the drive ends, the window shows the GraphSLAM result.

The simulation draws random noise every run and does not fix a random seed, so **your numbers and plots will differ from the ones in this document**. The figures here come from one run with the random seed set to $4$, produced by [generate_code_walkthrough_figures.m](images/generate_code_walkthrough_figures.m). That script runs the simulation unchanged, and then re-builds the solver's system once more, with a few extra counters, to record diagnostics such as the total error at each iteration. It checks that its copy reproduces the simulation's own optimized poses and landmarks. [generate_code_walkthrough_figures.py](images/generate_code_walkthrough_figures.py) draws the figures from the exported data.

![The final figure drawn by the simulation. Red stars are the six true landmarks, green stars are the optimized landmark estimates, and each green star is close to a red one. The blue line is the true path, the dashed grey line is the dead-reckoned path, and the green line is the optimized path. The green path follows the blue path more closely than the grey one does.](images/code_result_matlab.png)

*The figure that the simulation itself draws at the end of the run (seed 4). The blue line is where the robot really went. The grey dashed line is what dead reckoning says, and the green line is the GraphSLAM result.*

---

## Overview: Two Phases

The program does exactly what the [Introduction](graphslam_introduction.md) said a batch GraphSLAM does. It is split into two phases:

| Phase | Where in the file | What happens |
|---|---|---|
| **1. Drive and log** | The `while` loop in the main script | The robot moves, dead-reckons, and **records** everything. The estimate is never corrected during the drive. |
| **2. Solve** | The function `graphSLAM` | Builds the graph from the logs and solves for all poses and landmarks at once. |

The main script announces this in a comment before the loop. Four arrays are the only things phase 1 produces for phase 2:

```matlab
poseHistory = [0, 0.2, 0];
controlHistory = zeros(0, 3);
measurementLog = zeros(0, 4);
truePoseHistory = poseHistory;
```

- `poseHistory` holds the dead-reckoned pose at every step. It will be the **initial guess** $\bar\mu$ for every pose node.
- `controlHistory` holds the noisy odometry reading $[v,\ w,\ \Delta t]$ for each step. These become the **motion edges**.
- `measurementLog` holds every landmark observation as `[pose index, landmark id, range, bearing]`. These become the **observation edges**.
- `truePoseHistory` is the robot's real pose. It is used only to draw plots and to measure the error. The solver never sees it.

---

## Phase 1: Driving and Logging

### Dead reckoning

Each time through the loop, the helper `getControlAndMeasurement` steers the robot with a pure-pursuit controller, moves its **true** pose with noise, and returns two things: a noisy control `u` and a noisy range-bearing measurement `z` of one landmark.

```matlab
    [u, z, truePose, correspondence] = getControlAndMeasurement(estimatedPose, v_mps, lastGoal, currentGoal, simParams);
```

The robot cannot see the true motion. It only has its own odometry reading of it, which has additional noise (`sigma_v_odom` and `sigma_w_odom`). The robot integrates that reading to get its next pose estimate, and logs both the new pose and the control:

```matlab
    v = u(1) + simParams.sigma_v_odom*randn;
    w = u(2) + simParams.sigma_w_odom*randn;
    dt = u(3); theta = estimatedPose(3);
    newPose = [estimatedPose(1) + v*cos(theta)*dt, ...
               estimatedPose(2) + v*sin(theta)*dt, ...
               wrapToPi(theta + w*dt)];
    poseHistory(end+1, :) = newPose;
    controlHistory(end+1, :) = [v, w, dt];
    truePoseHistory(end+1, :) = truePose;
```

This is the dead-reckoned path from the earlier documents: a chain of poses, each computed from the previous one. It drifts, because the noise in `v` and `w` is never corrected. The grey dashed line in the result figure above is this path.

### Logging observations

Next, the observation of the landmark is logged. The simulator tells the code which landmark was seen, in `correspondence`, so there is no data-association problem to solve here.

```matlab
    j = correspondence;
    if(~observedLandmarks(j))
        % First sighting: seed a naive landmark estimate from the current pose/measurement
        landmarkEstimates(j, :) = newPose(1:2) + z(1)*[cos(z(2) + newPose(3)), sin(z(2) + newPose(3))];
        observedLandmarks(j) = true;
    end
    measurementLog(end+1, :) = [size(poseHistory,1), j, z(1), z(2)];
```

Two things happen:

- **A landmark is initialized the first time it is seen.** Its position is computed by walking `range` metres from the current dead-reckoned pose in the direction `bearing`. This is the "dead-reckoned estimate" that the 2D example used to initialize its landmarks. The guess inherits whatever drift the pose had at that moment.
- **Every observation is logged**, tagged with the index of the pose it was made from (`size(poseHistory,1)`, the pose that was just added).

In this run the robot took $150$ steps, so there are $151$ poses and $150$ observations, one per step. The observations were of six landmarks:

![A scatter plot with the pose index on the horizontal axis and landmark number on the vertical axis. Each of the six landmarks is observed many times, spread across the whole trajectory. A bar chart on the right shows the counts: 30, 26, 21, 23, 21 and 29 observations.](images/code_observation_log.png)

Each landmark is observed repeatedly, from poses that are spread over the whole drive. In the language of the [loop closure document](graphslam_loop_closure.md), every re-sighting of a landmark after a long gap is a loop closure through that landmark. The simulation gets them for free, because it already knows which landmark it is looking at.

---

## Phase 2: Solving the Graph

After the drive, the main script makes a single call. The function receives only the logs and two noise matrices, `Q` and `R`. It does not receive the simulation's parameter struct, so it does not depend on the simulator, and it could be used with logs from any source.

```matlab
[optimizedPoses, optimizedLandmarks] = graphSLAM(poseHistory, controlHistory, measurementLog, landmarkEstimates, observedLandmarks, Q, R);
```

The rest of this document is about what `graphSLAM` does with those logs.

### Nodes and the state vector

Every pose gets a node. Only landmarks that were actually observed get one, because a landmark with no constraints would leave the system singular.

```matlab
    observedIds = find(observedLandmarks)';
    numObserved = numel(observedIds);
    landmarkNodeOf = zeros(numLandmarks, 1);
    landmarkNodeOf(observedIds) = 1:numObserved;

    poseDims = 3*numPoses;
    n = poseDims + 2*numObserved;
```

All the nodes are stacked into one state vector, exactly as in the [2D example](graphslam_2d_example.md), with the poses first and the landmarks after them:

```math
\mu=
\begin{bmatrix}
x_1\\
y_1\\
\theta_1\\
\vdots\\
x_T\\
y_T\\
\theta_T\\
L_{1,x}\\
L_{1,y}\\
\vdots
\end{bmatrix}.
```

The initial guess $\bar\mu$ is built from the logs: the dead-reckoned poses, and the first-sighting landmark positions.

```matlab
    mu = zeros(n, 1);
    mu(1:poseDims) = reshape(poseHistory', [], 1);
    mu(poseDims+1:end) = reshape(landmarkEstimates(observedIds,:)', [], 1);
```

In this run, $T=151$ poses and $6$ landmarks give $n=3\cdot151+2\cdot6=465$ unknowns. Pose $t$ occupies entries `3*(t-1)+1` to `3*(t-1)+3`, and each landmark has two entries after `poseDims`. The code computes these ranges as `idx_t`, `idx_pose` and `idx_lm`, and they are how it finds a node's block inside $\Omega$ and $\xi$.

### The loop: relinearize, build, solve

The measurement functions are nonlinear, so, as in the 2D example, the solver builds a linear system around the current guess, solves it, and repeats. The code runs a fixed five rounds of this, which is a Gauss-Newton iteration:

```matlab
    omega_meas = inv(Q);
    numIterations = 5;
    for iter = 1:numIterations
        H = zeros(n);
        b = zeros(n, 1);
```

At the start of every round the system is reset to zero, and then every constraint adds its contribution. `omega_meas` is the weight of a landmark observation, $W=\Sigma^{-1}$, with $\Sigma$ built from the range and bearing noise in `Q`, which the main script passes in. In this simulation $W=\mathrm{diag}(400,\ 820.7)$.

#### How the code's variables map to the documents

The code uses different names, and one different way of writing the system:

| In the code | In the documents |
|---|---|
| `H` | The information matrix $\Omega$ |
| `mu` | The linearization point $\bar\mu$, which is updated every round |
| `e` | The residual: for an observation, $z-h(\bar\mu)$ |
| `A`, `B` | The Jacobian blocks for the two nodes of a constraint, with the sign flipped: they are the derivative of the residual $e$, so $A=-J_{\text{first node}}$ and $B=-J_{\text{second node}}$ |
| `omega_meas`, `omega_motion` | The weights $W$ |
| `b` | The right-hand side, but see below |
| `dx` | The correction $\mu-\bar\mu$ |

The documents solve $\Omega\mu=\xi$ for $\mu$ directly. The code instead solves for the **correction** $\Delta\mu=\mu-\bar\mu$, with $H\,\Delta\mu=-b$, and adds it to the guess. The two are the same system. Because $b=\sum A^\top W e$, and because $\xi=\Omega\bar\mu-b$, solving for $\Delta\mu$ and then adding it to $\bar\mu$ gives exactly the $\mu$ that $\Omega\mu=\xi$ would. The sign flip in $A$ cancels in $A^\top WA=J^\top WJ$, and the only place it shows up is in $b$.

#### The anchor

The first thing added to the system is a prior on the first pose:

```matlab
        H(1:3,1:3) = H(1:3,1:3) + diag([1e6, 1e6, 1e6]);
```

This is the prior from Step 1 of the 2D example. Without it, every constraint is relative, and the whole graph could slide or rotate without changing any residual. A weight of $10^6$ means a standard deviation of $0.001$, much tighter than the $0.1$ used in the documents, so the first pose is held essentially fixed.

It only adds to `H`, and not to `b`, so it acts as a very stiff spring on the first pose's **correction**: pose 1 stays at wherever it started. After five rounds, its heading had moved by $1.4\times10^{-6}$ radians.

### Motion constraints

Every pair of consecutive poses is connected by an edge, built from the odometry reading that was logged for that step. This is the odometry edge from the earlier documents. The code writes it in the form that the simulation used to integrate the motion: the pose that the motion model predicts, compared with the pose that is currently in the state vector.

```matlab
        for t = 1:numPoses-1
            idx_t  = 3*(t-1)+1 : 3*(t-1)+3;
            idx_t1 = 3*t+1     : 3*t+3;

            theta = mu(idx_t(3));
            v = controlHistory(t,1); w = controlHistory(t,2); dt = controlHistory(t,3);

            % error between where pose t+1 actually is and where the
            % motion model predicts it should be, given pose t and control t
            predicted = [mu(idx_t(1)) + v*cos(theta)*dt;
                         mu(idx_t(2)) + v*sin(theta)*dt;
                         wrapToPi(theta + w*dt)];
            e = mu(idx_t1) - predicted;
            e(3) = wrapToPi(e(3));
```

In math, with $\mathbf x_t=(x_t,\ y_t,\ \theta_t)$ and the logged control $(v,\ w)$, the motion model and the residual are

```math
g(\mathbf x_t)=
\begin{bmatrix}
x_t+v\cos\theta_t\,\Delta t\\
y_t+v\sin\theta_t\,\Delta t\\
\theta_t+w\,\Delta t
\end{bmatrix},
\qquad
e=\mathbf x_{t+1}-g(\mathbf x_t).
```

There are two differences from the odometry measurement in the 2D example. The residual here is "state minus prediction", rather than "measurement minus prediction". And it is expressed as a pose in the **world** frame, rather than as a relative pose in the robot's frame. Both constrain consecutive poses in the same way, so the structure of $\Omega$ is the same.

The residual has two nodes in it, $\mathbf x_t$ and $\mathbf x_{t+1}$, so its Jacobian has two blocks:

```matlab
            G = [1, 0, -v*sin(theta)*dt;
                 0, 1,  v*cos(theta)*dt;
                 0, 0,  1];
            A = -G;      % d(e)/d(pose_t)
            B = eye(3);  % d(e)/d(pose_t+1)
```

```math
A=\frac{\partial e}{\partial\mathbf x_t}=-G,
\qquad
G=
\begin{bmatrix}
1&0&-v\sin\theta_t\,\Delta t\\
0&1&v\cos\theta_t\,\Delta t\\
0&0&1
\end{bmatrix},
\qquad
B=\frac{\partial e}{\partial\mathbf x_{t+1}}=I.
```

The third column of $G$ is the "lever arm" from the 2D example: it says how much the predicted position changes if the heading of pose $t$ is wrong.

#### The weight

The noise has to be described in the space of the residual, which here is pose space. The main script builds a diagonal covariance matrix `R` for it, next to the matrix `Q` for the range and bearing:

```matlab
R = diag([simParams.sigma_v_odom^2, simParams.sigma_v_odom^2, simParams.sigma_w_odom^2]);
```

`R` is expressed **per second**: its entries are the variances of the odometry reading's noise (in $\text{m}^2/\text{s}^2$ and $\text{rad}^2/\text{s}^2$). Inside the loop, the function scales it by $\Delta t^2$ to get the covariance of one step, and inverts that to get the weight:

```matlab
            % The covariance of this step's odometry noise is R scaled by
            % dt^2, and its information matrix is the inverse of that.
            omega_motion = inv(R*dt^2);
```

That is $W=\mathrm{diag}(4444,\ 4444,\ 328281)$ for this simulation, where $\Delta t=0.1$. The scaling is done inside the function, instead of passing a fixed matrix, because $\Delta t$ is part of each logged control, so steps of different lengths each get the right weight.

The main script's comments explain the choice of a diagonal matrix. The weight is based on the noise of the **odometry reading**, which is what was logged. The noise in the two inputs $v$ and $w$ could in principle be mapped into pose space through the motion model, but this was not done, because $v$ and $w$ span only two of the three dimensions, and that would have made $W$ singular.

#### Adding the contribution

Now the pieces from the [2D example](graphslam_2d_example.md) come together. The Jacobian of this constraint is non-zero only in the columns of the two poses, $J=[A\ \ B]$, so the contribution $J^\top WJ$ and $J^\top We$ breaks into four blocks of $H$ and two of $b$:

```math
J^\top WJ=
\begin{bmatrix}
A^\top WA&A^\top WB\\
B^\top WA&B^\top WB
\end{bmatrix},
\qquad
J^\top We=
\begin{bmatrix}
A^\top We\\
B^\top We
\end{bmatrix}.
```

```matlab
            H(idx_t,  idx_t)  = H(idx_t,  idx_t)  + A'*omega_motion*A;
            H(idx_t,  idx_t1) = H(idx_t,  idx_t1) + A'*omega_motion*B;
            H(idx_t1, idx_t)  = H(idx_t1, idx_t)  + B'*omega_motion*A;
            H(idx_t1, idx_t1) = H(idx_t1, idx_t1) + B'*omega_motion*B;
            b(idx_t)  = b(idx_t)  + A'*omega_motion*e;
            b(idx_t1) = b(idx_t1) + B'*omega_motion*e;
        end
```

These six lines are the whole mechanism. They add $J^\top WJ$ into the blocks where the two nodes meet, and nothing anywhere else, which is why the matrix ends up sparse.

### Observation constraints

The second loop does the same for each logged landmark observation. A row of `measurementLog` says which pose saw which landmark, so the code finds the two blocks that the observation touches:

```matlab
        for k = 1:size(measurementLog, 1)
            t = measurementLog(k,1);
            j = measurementLog(k,2);
            z = measurementLog(k,3:4)';

            idx_pose = 3*(t-1)+1 : 3*(t-1)+3;
            node = landmarkNodeOf(j);
            idx_lm = poseDims + 2*(node-1)+1 : poseDims + 2*(node-1)+2;
```

#### The measurement model

This sensor reports **range and bearing**, so the expected measurement is a different function from the (forward, left) observation in the 2D example. The note in that document said only $h$ and its derivatives change, and here it is:

```matlab
            pose = mu(idx_pose);
            lm   = mu(idx_lm);

            delta = lm - pose(1:2);
            q = delta'*delta;
            zhat = [sqrt(q); wrapToPi(atan2(delta(2), delta(1)) - pose(3))];
            e = z - zhat;
            e(2) = wrapToPi(e(2));
```

With $d_x=L_x-x$, $d_y=L_y-y$, and $q=d_x^2+d_y^2$, the expected measurement and the residual are

```math
h=
\begin{bmatrix}
\sqrt q\\
\mathrm{atan2}(d_y,\ d_x)-\theta
\end{bmatrix},
\qquad
e=z-h.
```

The first component is the distance to the landmark, and the second is the angle to the landmark relative to the robot's heading. This residual has the same form, "measurement minus prediction", as in the 2D example. It is wrapped to $[-\pi,\ \pi]$, so that an angle that goes just past $\pi$ is not mistaken for a large error.

#### The Jacobian

Differentiating $h$ with respect to the five variables $x,\ y,\ \theta,\ L_x,\ L_y$ gives the Jacobian that the code calls `H_low`. Its two rows are the derivatives of the range and of the bearing:

```matlab
            H_low = (1/q)*[-sqrt(q)*delta(1), -sqrt(q)*delta(2), 0,     sqrt(q)*delta(1), sqrt(q)*delta(2); ...
                             delta(2),         -delta(1),        -q,   -delta(2),         delta(1)];
            A = -H_low(:,1:3);  % d(e)/d(pose)
            B = -H_low(:,4:5);  % d(e)/d(landmark)
```

```math
\frac{\partial h}{\partial(x,\ y,\ \theta,\ L_x,\ L_y)}=
\frac{1}{q}
\begin{bmatrix}
-\sqrt q\,d_x&-\sqrt q\,d_y&0&\sqrt q\,d_x&\sqrt q\,d_y\\
d_y&-d_x&-q&-d_y&d_x
\end{bmatrix}.
```

The first row is about the range. The entries for $L_x$ and $L_y$ are the unit vector that points from the robot to the landmark, so the range grows when the landmark moves away from the robot along that line. The entries for $x$ and $y$ are the negative of it, so the range shrinks by the same amount when the robot moves the same way, and the $0$ for $\theta$ says that the range does not depend on the heading. In the second row, the entry for $\theta$ is $-1$ (after the factor $1/q$), because turning the robot by an angle changes the bearing by minus that angle. The first three columns become `A` and the last two become `B`, both with the sign flipped, as before.

#### The weight and the update

The weight is `omega_meas`, from before, and the update is the same six lines as for the motion constraint, with the pose and the landmark as the two nodes:

```matlab
            H(idx_pose, idx_pose) = H(idx_pose, idx_pose) + A'*omega_meas*A;
            H(idx_pose, idx_lm)   = H(idx_pose, idx_lm)   + A'*omega_meas*B;
            H(idx_lm,   idx_pose) = H(idx_lm,   idx_pose) + B'*omega_meas*A;
            H(idx_lm,   idx_lm)   = H(idx_lm,   idx_lm)   + B'*omega_meas*B;
            b(idx_pose) = b(idx_pose) + A'*omega_meas*e;
            b(idx_lm)   = b(idx_lm)   + B'*omega_meas*e;
        end
```

This is a $3\times3$ pose block, two $3\times2$ and $2\times3$ blocks linking the pose and the landmark, and a $2\times2$ landmark block, just like the blocks in the 2D example's [layout figure](graphslam_2d_example.md#the-state-vector).

### What the system looks like

After both loops, `H` is $465\times465$. Here is its pattern of non-zero entries for the first round, with the landmark rows and columns outlined in yellow, and a zoom on the first six poses:

![On the left, the 465 by 465 matrix pattern: a thin diagonal band for the 151 poses, and a vertical strip along the right edge and a horizontal strip along the bottom edge for the 6 landmarks. On the right, a zoom of the first six poses in 3 by 3 blocks, showing non-zero blocks only on the diagonal and next to it.](images/code_system_pattern.png)

It has the same structure as the loop-closure examples:

- **The band along the diagonal** comes from the motion constraints. Each links two consecutive poses, so it only touches the blocks next to the diagonal.
- **The strips along the edges** come from the observations. A landmark is linked to every pose that saw it. The row for the first landmark, for example, has $92$ non-zero entries: $3$ for each of the $30$ poses that observed it, and its own $2$.
- **Few entries are non-zero.** Only $4{,}677$ of the $216{,}225$ entries, or $2.2\%$.

A few entries inside the $3\times3$ blocks are zero too. They depend on the data. For example, the first pose has heading $0$, so the $\sin\theta$ term in $G$ is zero.

### Solving and updating

With the system built, the correction comes from one line, and the guess is updated:

```matlab
        dx = -(H\b);
        mu = mu + dx;
        for t = 1:numPoses
            mu(3*t) = wrapToPi(mu(3*t));
        end
```

The headings are wrapped again after the update, since adding a correction can push an angle past $\pi$. Then the loop goes back to the top, where the next round **relinearizes** around the improved `mu`: the Jacobians and residuals are recomputed from scratch.

#### Does five rounds suffice?

The loop has no convergence test. It always runs five rounds. I recorded the total weighted error that the solver is minimizing, the sum of $e^\top We$ over every constraint, and the size of the largest correction in each round. The simulation does not compute these.

![Two charts. On the left, the total weighted error on a log scale falls from 8,470.7 at the initial guess to 638.1 after the first iteration, 294.5 after the second, and 258.6, 258.5 and 258.5 after the third to fifth. On the right, the largest change to any variable falls from 0.28 in the first iteration to 0.091, 0.014, 0.0016 and 0.00017.](images/code_convergence.png)

- The error is $8{,}470.7$ at the initial guess and drops by a factor of $13$ in the first round. After the third round it is $258.64$. The fourth round lowers it by only $0.18$, to $258.47$, and the fifth changes it by $0.0001$.
- The largest correction to any variable shrinks by a factor of about $3$ to $10$ each round, down to $1.7\times10^{-4}$ in the fifth.

For this run, five is more than enough. At the initial guess, every motion constraint is satisfied exactly, because the poses were produced by integrating the same motion model. I checked this: the motion constraints contribute exactly $0$ to the starting error. So all of it comes from the landmark observations, which the drifting poses and the first-sighting landmarks disagree with.

### Unpacking the result

Finally, the state vector is cut back into poses and landmark positions. Landmarks that were never observed come back as `NaN`:

```matlab
    poses = reshape(mu(1:poseDims), 3, numPoses)';
    landmarks = nan(numLandmarks, 2);
    landmarks(observedIds,:) = reshape(mu(poseDims+1:end), 2, numObserved)';
```

---

## How Well Does It Work?

Compared with the true path, which the solver never saw:

![On the left, position error against pose index. The dead-reckoning error drifts up to about 0.24 metres, while the GraphSLAM error stays below about 0.09 metres. On the right, a bar chart of the position error of each landmark's initial guess and its GraphSLAM estimate. Five of the six landmarks improve.](images/code_errors.png)

| | Dead reckoning | GraphSLAM |
|---|---|---|
| Mean position error | $0.099\text{ m}$ | $0.046\text{ m}$ |
| Position error at the last pose | $0.202\text{ m}$ | $0.031\text{ m}$ |
| Largest position error | $0.237\text{ m}$ | $0.089\text{ m}$ |

The landmark errors, from the first-sighting guess to GraphSLAM, are:

| Landmark | $L_1$ | $L_2$ | $L_3$ | $L_4$ | $L_5$ | $L_6$ |
|---|---|---|---|---|---|---|
| First sighting (m) | 0.050 | 0.073 | 0.074 | 0.118 | 0.028 | 0.063 |
| GraphSLAM (m) | 0.037 | 0.052 | 0.049 | 0.044 | 0.038 | 0.055 |

Five of the six landmarks improved. $L_5$ got slightly worse: its first sighting happened to be very accurate, and the solution is a compromise among all $21$ observations of it and the poses that saw it.

This is a single run. The same program run with other random seeds, as `GRAPHSLAM_SEED=n` and `GRAPHSLAM_EXPORT=0` in [generate_code_walkthrough_figures.m](images/generate_code_walkthrough_figures.m), gives:

| Seed | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| Poses | 154 | 149 | 153 | 151 | 149 | 416 | 148 | 147 | 146 |
| Dead reckoning, mean error (m) | 0.109 | 0.100 | 0.148 | 0.099 | 0.060 | 0.403 | 0.081 | 0.056 | 0.090 |
| GraphSLAM, mean error (m) | 0.077 | 0.040 | 0.026 | 0.046 | 0.019 | 0.031 | 0.051 | 0.028 | 0.022 |

GraphSLAM reduced the mean position error in every one of the nine runs. How much varies. Seed 6 is a much longer drive (the robot took $416$ steps), and dead reckoning is the worst there, while GraphSLAM is still accurate.

---

## Efficiency: What the Code Does and Does Not Exploit

The [sparse solver document](graphslam_sparse_solvers.md) explains how to exploit the sparsity of $\Omega$. This simulation does not. It builds `H` as a full matrix, `zeros(n)`, and solves with a dense `H\b`. For $n=465$ that is instant, and it keeps the code short and easy to follow. For a map with thousands of poses, you would store $\Omega$ in a sparse format and use a sparse Cholesky factorization.

To show what that would buy, I factored this run's first-round $\Omega$ with MATLAB's `chol`, in the generator script (it is not part of the simulation). The dense lower triangle has $108{,}345$ entries:

| Order of the variables | Non-zeros in the Cholesky factor $L$ |
|---|---|
| The code's own order: poses in time order, then landmarks | $7{,}106$ |
| MATLAB's `symamd` minimum-degree order | $5{,}342$ |
| Landmarks first, then poses | $96{,}086$ |

These match the lessons of that document. The code's own order, with all the landmarks last, is already close to a good order. Putting the landmarks first is nearly as bad as a dense matrix, because every landmark is linked to many poses spread over the whole trajectory, and eliminating it first connects all of them, which is the fill-in effect.

---

## Where the Code Differs from the Documents

The code implements the same algorithm, but it makes some different choices:

- **It solves for a correction, not for $\mu$.** The system is $H\,\Delta\mu=-b$ instead of $\Omega\mu=\xi$. They are equivalent, as shown above.
- **It uses range and bearing**, so $h$ and its Jacobian are different from the (forward, left) observations used in the 2D example. Nothing else changes.
- **The motion constraint is in the world frame.** It compares poses with the motion model's prediction, instead of comparing relative poses in the robot's frame, and its noise is a diagonal matrix in pose space.
- **The data association is given.** The simulator reports which landmark it observed, so there is no front-end problem to solve, and every landmark re-sighting is a loop closure for free.
- **The prior is much stiffer.** It uses $10^6$ for the weight, and is added only to `H`.
- **The iteration count is fixed.** There is no convergence test.
- **The matrix is dense.** There is no use of the sparsity.
- **The simulated true motion** uses the `'icr'` kinematic model, but the dead reckoning and the graph both use the simpler Euler step. This small model mismatch is part of what GraphSLAM has to absorb.

---

## Summary

- The simulation drives first and solves later. It logs a dead-reckoned pose and a control for every step, and every landmark observation.
- `graphSLAM` stacks all poses and observed landmarks into one state vector, starts from the dead-reckoned guess, and runs five rounds of: build the system, solve, update.
- Every constraint, a motion edge or an observation, adds $A^\top WA$, $A^\top WB$, $B^\top WA$ and $B^\top WB$ to the four blocks of `H` where its two nodes meet, and $A^\top We$ and $B^\top We$ to `b`.
- The first pose is anchored, so the problem has a unique solution.
- The result is a trajectory and map that agree with all of the measurements at once, according to their weights.
