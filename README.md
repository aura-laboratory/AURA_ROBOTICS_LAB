# AURA Robotics Lab

This repository contains the **course-layer files only** for three integrated robotics labs. The large simulator/backend is installed separately and is intentionally **not committed to this repository**.

## Labs

1. **Lab 1 — Motion Planning**  
   Compare backend RRT and A*, then implement a student local planner while keeping the common assignment, heading interpolation, and temporal multi-UAV deconfliction pipeline.

2. **Lab 2 — Localization / Estimation / SLAM**  
   Work with dead reckoning, GPS fusion, EKF-style estimation, and landmark-based EKF-SLAM through a common `StudentEstimator` API.

3. **Lab 3 — Decision Making / Reinforcement Learning**  
   Choose which target each UAV should visit next; the motion-planning backend executes the resulting target sequence.

## Start here

Read **[README_INSTALLATION.md](README_INSTALLATION.md)** first.

After the backend is installed, each lab has its own folder, installer, README, and launcher(s):

- `lab1_motion_planning/`
- `lab2_localization_slam/`
- `lab3_decision_making/`

The three labs can coexist on the same backend. Lab 1 and Lab 3 launchers temporarily activate their own planner/config and restore the original backend files when the run exits normally or with Ctrl+C.

## Repository policy

Do not add the simulator image, `.aura_runtime`, virtual environments, result folders, or large generated archives to this Git repository. Only the course-layer patches and documentation belong here.
