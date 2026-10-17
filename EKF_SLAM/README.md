# EKF SLAM

This folder contains a MATLAB simulation of **EKF SLAM**, and a set of documents that explain the method, from the idea to the code. EKF SLAM is an online, filtering approach to simultaneous localization and mapping: it keeps a single running estimate of the robot's pose and the map, and folds in each new measurement as it arrives.

If you are new to the topic, start with the documents. If you want to see the code run, start with the simulation.

## Contents

| Path | What it is |
|---|---|
| [SIMULATION_EKFSLAM.m](SIMULATION_EKFSLAM.m) | The MATLAB simulation and the EKF SLAM filter |
| [SIMULATION_DeadReckoning.m](SIMULATION_DeadReckoning.m) | A simpler simulation showing odometry drift with no correction |
| [Documents/](Documents) | The documentation, with the figures it uses in [Documents/images/](Documents/images) |

## The Simulation

[SIMULATION_EKFSLAM.m](SIMULATION_EKFSLAM.m) drives a simulated differential-drive robot through a list of waypoints with a pure-pursuit controller. At every step it:

1. Predicts the robot's pose forward using its noisy odometry reading.
2. Observes a randomly chosen landmark with a noisy range and bearing, and corrects the state estimate (the robot pose plus every landmark position seen so far) with an EKF update.

Unlike GraphSLAM, nothing about the trajectory is revisited after the fact: every correction updates only the current state and its covariance matrix `P`, which is why the filter can run live, one step at a time, as the robot moves.

### Running it

1. Open MATLAB and run `SIMULATION_EKFSLAM.m`. It adds the [common](../common) folder to the path itself, so you can run it from anywhere.
2. A figure shows the robot driving the path, with its true pose, its estimated pose, the true landmarks, and the estimated landmarks all drawn together.

A few things to know:

- The script begins with `clear all` and `close all`, so running it clears your workspace.
- It does not set a random seed, so every run is different.
- I ran it in MATLAB R2026b. The script and its helpers use `deg2rad` and `rad2deg`, which I believe are part of base MATLAB from R2021b onwards, and were in a toolbox before that. I did not test older releases.

### What to change

The parameters are at the top of the script:

- `landmarks` and `goalPoints` set the environment and the path.
- `v_mps` is the robot's speed.
- `simParams` fields set the time step and the noise in the true motion, the odometry reading, and the range and bearing measurements.
- `largeInitialVariance` sets how uncertain an unobserved landmark starts out before it is first seen.

### Helpers

The simulation uses these functions from the [common](../common) folder, which are shared with the GraphSLAM code:

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
| [Introduction to EKF SLAM](Documents/ekf_slam_introduction.md) | Kalman filters from scratch, extending them to the EKF, and the block structure of the EKF SLAM state and covariance matrix |

More documents will be added to this series, working through the full 2D equations by hand and then through the code in [SIMULATION_EKFSLAM.m](SIMULATION_EKFSLAM.m). See the [GraphSLAM](../Graph_SLAM) folder for the offline, batch alternative to this method.

## Regenerating the Figures

The figures in the documents are produced by scripts in [Documents/images/](Documents/images), next to the images themselves. Each script also checks the numbers that its document quotes. Run it from the root of the repository:

```
python EKF_SLAM/Documents/images/generate_ekf_slam_introduction_figures.py
```

| Script | Produces the figures for |
|---|---|
| [generate_ekf_slam_introduction_figures.py](Documents/images/generate_ekf_slam_introduction_figures.py) | Introduction to EKF SLAM |

The script needs NumPy and Matplotlib.
