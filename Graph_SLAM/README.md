# GraphSLAM

This folder contains a MATLAB simulation of **GraphSLAM**, and a set of documents that explain the method, from the idea to the code. GraphSLAM is an offline, batch approach to simultaneous localization and mapping: it records the whole history of motion and landmark measurements, builds a graph from them, and solves for every robot pose and landmark at once.

If you are new to the topic, start with the documents. If you want to see the code run, start with the simulation.

## Contents

| Path | What it is |
|---|---|
| [SIMULATION_GraphSLAM.m](SIMULATION_GraphSLAM.m) | The MATLAB simulation and the GraphSLAM solver |
| [Documents/](Documents) | The documentation, with the figures it uses in [Documents/images/](Documents/images) |

## The Simulation

[SIMULATION_GraphSLAM.m](SIMULATION_GraphSLAM.m) is a single script with two phases:

1. **Drive and log.** A simulated differential-drive robot follows a path through a list of waypoints, steered by a pure-pursuit controller. At each step it dead-reckons from its own noisy odometry, and observes one landmark with a noisy range and bearing. The estimate is never corrected during the drive. The script only records the dead-reckoned poses, the odometry readings, and the landmark observations.
2. **Solve.** The function `graphSLAM` builds the graph from those logs, and solves for all the poses and landmark positions together with Gauss-Newton iterations. It receives only the logs and two noise matrices: `Q` for the range and bearing observations, and `R` for the odometry. It does not depend on the simulator.

When the drive ends, the script plots the true path, the dead-reckoned path and the optimized path, along with the true and optimized landmarks.

### Running it

1. Open MATLAB and run `SIMULATION_GraphSLAM.m`. It adds the [common](../common) folder to the path itself, so you can run it from anywhere.
2. A figure shows the robot driving the path. When it reaches the last waypoint, the figure changes to the GraphSLAM result.

A few things to know:

- The script begins with `clear all` and `close all`, so running it clears your workspace.
- It does not set a random seed, so every run is different. To repeat a run, call `rng(n)` before it, for example `rng(4); run('SIMULATION_GraphSLAM.m')`.
- I ran it in MATLAB R2026b. The script and its helpers use `deg2rad` and `rad2deg`, which I believe are part of base MATLAB from R2021b onwards, and were in a toolbox before that. I did not test older releases. The script defines its own `wrapToPi`.

### What to change

The parameters are at the top of the script:

- `landmarks` and `goalPoints` set the environment and the path.
- `v_mps` is the robot's speed.
- The `simParams` fields set the time step and the noise in the true motion, the odometry reading, and the range and bearing measurements.
- `numIterations` in `graphSLAM` sets how many Gauss-Newton rounds are run. It is fixed, with no convergence test.

### Helpers

The simulation uses these functions from the [common](../common) folder, which are shared with the EKF SLAM code:

| File | Used for |
|---|---|
| [getControlAndMeasurement.m](../common/getControlAndMeasurement.m) | Advances the true robot one step with noise, and returns the noisy control and one noisy range-bearing observation |
| [purePursuit.m](../common/purePursuit.m) | Computes the steering command that follows the path |
| [differentialDriveKinematics.m](../common/differentialDriveKinematics.m) | Moves the true robot pose |
| [drawRobot.m](../common/drawRobot.m) | Draws the robot in the plot |

## Documentation

The documents in [Documents/](Documents) are meant to be read in order. Each one assumes the ones before it.

| Document | What it covers |
|---|---|
| [Introduction to GraphSLAM](Documents/graphslam_introduction.md) | What the method is and why it works |
| [1D GraphSLAM Example](Documents/graphslam_1d_example.md) | The full method on a robot that moves along a line: building the information matrix and vector, solving, and weights |
| [2D GraphSLAM Example](Documents/graphslam_2d_example.md) | Poses with a heading, block matrices, noise matrices, and linearization |
| [Loop Closure in GraphSLAM](Documents/graphslam_loop_closure.md) | How revisiting a place corrects drift across the whole trajectory |
| [Solving the System Efficiently](Documents/graphslam_sparse_solvers.md) | Using the sparsity of the information matrix: Cholesky factorization, fill-in, and eliminating landmarks |
| [GraphSLAM in Code](Documents/graphslam_code_walkthrough.md) | How `SIMULATION_GraphSLAM.m` implements all of the above, with listings and figures from a real run |

If you would rather start from the code, read the last document first, and go back to the earlier ones when you meet an unfamiliar term.

## Regenerating the Figures

The figures in the documents are produced by scripts in [Documents/images/](Documents/images), next to the images themselves. Each script also checks the numbers that its document quotes. Run them from the root of the repository.

| Script | Produces the figures for |
|---|---|
| [generate_step_figures.py](Documents/images/generate_step_figures.py) | The 1D example |
| [generate_2d_figures.py](Documents/images/generate_2d_figures.py) | The 2D example |
| [generate_loop_and_sparse_figures.py](Documents/images/generate_loop_and_sparse_figures.py) | The loop closure and sparse solver documents, using the model in [loop_example_model.py](Documents/images/loop_example_model.py) |
| [generate_code_walkthrough_figures.m](Documents/images/generate_code_walkthrough_figures.m) and [generate_code_walkthrough_figures.py](Documents/images/generate_code_walkthrough_figures.py) | The code walkthrough |

The Python scripts need NumPy and Matplotlib:

```
python Graph_SLAM/Documents/images/generate_step_figures.py
python Graph_SLAM/Documents/images/generate_2d_figures.py
python Graph_SLAM/Documents/images/generate_loop_and_sparse_figures.py
```

The code walkthrough has two steps, because its data comes from running the real simulation. The MATLAB script runs `SIMULATION_GraphSLAM.m` unchanged with a fixed seed, and exports the data. The Python script draws the figures from it:

```
matlab -batch "run('Graph_SLAM/Documents/images/generate_code_walkthrough_figures.m')"
python Graph_SLAM/Documents/images/generate_code_walkthrough_figures.py
```

The code listings in the walkthrough are copied from the simulation. If you change `SIMULATION_GraphSLAM.m`, run [check_code_walkthrough_snippets.py](Documents/images/check_code_walkthrough_snippets.py) to see which listings no longer match:

```
python Graph_SLAM/Documents/images/check_code_walkthrough_snippets.py
```
