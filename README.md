# Two-Wheel Self-Balancing Robot — Controls & ML Simulation

Simulation-based controls and machine-learning project for a two-wheel self-balancing robot using MATLAB, Simulink, and Python.

## Overview

This project progresses from classical feedback control to state estimation and learning-based dynamics modeling.

Current work includes:
- Inverted-pendulum modeling
- PID stabilization
- State-space modeling and LQR control
- Simulated sensor noise
- Complementary filtering
- Kalman state estimation
- Estimated-state LQR feedback
- Physics-informed residual learning in progress

## Current Results

### State Estimation
- Raw simulated pitch measurement RMSE: **0.2061°**
- Complementary-filter pitch RMSE: **0.0506°**
- Kalman pitch RMSE: **0.0899°** under nominal conditions

### Controller Comparison
The project compares PID, perfect-state LQR, and Kalman-estimated-state LQR under initial-angle disturbances, external force disturbances, actuator saturation, and parameter variation.

## Main Files

- `self_balancing_robot_pid_rebuilt.slx` — PID controller
- `self_balancing_robot_lqr_perfect_state.slx` — LQR with perfect state feedback
- `self_balancing_robot_lqr_kalman.slx` — Kalman-estimated-state LQR
- `robot_params.m` — robot, controller, sensor, and estimator parameters
- `compare_pid_lqr.m` — PID vs LQR comparison
- `compare_v3.m` — perfect-state vs Kalman-LQR comparison
- `version3_analysis.m` — complementary-filter analysis
- `version3_kalman_analysis.m` — Kalman estimator RMSE analysis

## ML Extension

The next stage uses trajectories generated from a higher-fidelity nonlinear simulation to train a residual neural dynamics model.

The goal is to compare:

- nominal physics model
- pure ML dynamics model
- hybrid physics + learned residual model

The residual model is intended to learn dynamics not captured by the simplified nominal physics model.

## Status

- [x] PID control
- [x] State-space model
- [x] LQR control
- [x] Sensor noise simulation
- [x] Complementary filtering
- [x] Kalman state estimation
- [x] Estimated-state LQR
- [ ] Nonlinear truth-model dataset generation
- [ ] Residual neural dynamics model
- [ ] Physics vs pure-ML vs hybrid evaluation