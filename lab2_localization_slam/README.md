# Lab 2 — Localization, State Estimation and SLAM

## Goal

Study **where the UAV is** and, in SLAM mode, **where the UAV and observed landmarks are**.

The infrastructure creates noisy motion/sensor measurements and evaluates the student's estimate against simulator ground truth. Ground truth is used by the evaluator/visualization, not passed to `StudentEstimator` as the answer.

## Install

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab2_localization_slam
./install_lab2.sh ~/Downloads/aura-lab
```

## Run

```bash
cd ~/Downloads/aura-lab

./run_lab2.sh dead_reckoning
./run_lab2.sh gps_fusion
./run_lab2.sh ekf
./run_lab2.sh ekf_slam
```

Additional methods present in the course API may include:

```bash
./run_lab2.sh particle_filter
./run_lab2.sh graph_slam
```

Some of those are intended as student/advanced TODOs.

## Main student file

Edit:

```text
~/Downloads/aura-lab/.aura_runtime/backend/
  mrim_task/mrim_planner/scripts/student_estimator.py
```

Common API:

```python
reset(initial_pose, initial_covariance=None)
predict(motion)
update(measurements)
get_pose()
get_map()
```

The pose state is based on:

```text
[x, y, z, yaw]
```

## Student work / TODO areas

The course file contains the exact TODO markers. The main progression is:

- dead reckoning,
- GPS fusion,
- EKF localization,
- particle-filter extension,
- EKF-SLAM with landmark observations,
- graph-SLAM as an advanced extension.

The current course package includes working reference infrastructure for EKF-SLAM and leaves the designated student/advanced sections in `student_estimator.py` for exercises.

## EKF-SLAM

In `ekf_slam` mode the estimator receives noisy landmark observations containing an ID, range, bearing, and elevation. The student estimator does not receive the true world landmark coordinates.

The RViz configuration includes separate displays for:

- ground-truth trajectory,
- dead-reckoning trajectory,
- estimator/EKF-SLAM trajectory,
- true landmarks for evaluation/visualization,
- estimated landmarks.

## Included backend files

```text
mrim_task/mrim_planner/scripts/student_estimator.py
mrim_task/mrim_planner/scripts/aura_estimator_node.py
mrim_task/mrim_planner/launch/aura_lab2.launch
mrim_task/mrim_manager/config/rviz/offline_lab2.rviz
```

Root launcher installed into the AURA host folder:

```text
run_lab2.sh
```
