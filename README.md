# AURA Robotics Lab

This repository contains the course material and software layer used for the practical laboratory exercises of the Robotics course.

The laboratories use a common robotics simulation backend, which provides the simulated environment, UAV models, visualization, trajectory execution, and supporting infrastructure. The course repository contains only the files required for the laboratory exercises and does not include the full simulator/backend.

## Laboratories

The course includes three laboratory exercises.

### Lab 1 — Motion Planning

Lab 1 focuses on motion planning in a multi-UAV mission.

Students work with an integrated planning pipeline that combines:

- path planning,
- task allocation,
- routing.

The lab includes the comparison of provided A* and RRT planners, the implementation of a shared-viewpoint assignment strategy, and the development of a student local path planner.

The remaining stages of the mission pipeline, such as TSP-based visit ordering, heading interpolation, trajectory generation, smoothing, and temporal multi-UAV collision avoidance, are provided as part of the laboratory infrastructure.

See:

`lab1_motion_planning/README.md`

### Lab 2 — Localization / Estimation / SLAM

Lab 2 focuses on robot localization, state estimation, and SLAM.

Students will work with estimation methods and sensor information in a simulated robotic system.

The detailed structure and exercises for this laboratory will be provided in the Lab 2 material.

See:

`lab2_localization_slam/`

### Lab 3 — Decision Making / Reinforcement Learning

Lab 3 focuses on decision making and reinforcement learning for robotic systems.

Students will study how autonomous robots can make decisions and select actions while interacting with a simulated environment.

The detailed structure and exercises for this laboratory will be provided in the Lab 3 material.

See:

`lab3_decision_making/`

## Installation

The simulator/backend must be installed before running the laboratories.

Start with:

**[README_INSTALLATION.md](README_INSTALLATION.md)**

After the backend installation, each laboratory provides its own files, instructions, and launcher scripts.

