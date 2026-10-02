# Lab 1 — Motion Planning

## Goal

Study **how a UAV gets from one viewpoint to the next** in a 3D environment with obstacles, while keeping the rest of the multi-UAV mission pipeline fixed.

All three Lab 1 modes use the same student-facing logic for:

- assignment of shared/purple viewpoints,
- heading interpolation,
- temporal two-UAV collision avoidance.

Only the **local path planner** changes between the available modes.

---

## Install

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab1_motion_planning
./install_lab1.sh ~/Downloads/aura-lab
```

---

## Run

```bash
cd ~/Downloads/aura-lab

./run_lab1_rrt.sh
./run_lab1_astar.sh
./run_lab1_student.sh
```

Equivalent generic launcher:

```bash
./run_lab1.sh rrt
./run_lab1.sh astar
./run_lab1.sh student
```

`rrtstar` is also accepted by the generic launcher when the corresponding backend planner is available/configured.

---

## What each mode does

```text
RRT mode
Student assignment
    ↓
Backend RRT
    ↓
Heading interpolation
    ↓
Trajectory generation
    ↓
Temporal two-UAV deconfliction
```

```text
A* mode
Student assignment
    ↓
Backend A*
    ↓
Heading interpolation
    ↓
Trajectory generation
    ↓
Temporal two-UAV deconfliction
```

```text
STUDENT mode
Student assignment
    ↓
StudentPlanner.plan_path()
    ↓
Heading interpolation
    ↓
Trajectory generation
    ↓
Temporal two-UAV deconfliction
```

---

# Student Work

For the core Lab 1 assignment, the student should modify **ONLY this file**:

**`student_planner_lab1_aura.py`**

After installation, the full path is:

```text
~/Downloads/aura-lab/.aura_runtime/backend/
mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

The student has **two main TODOs** inside this file.

---

## TODO 1 — Shared Viewpoint Assignment

The first TODO is:

**`assign_shared_viewpoints(...)`**

This method decides how the shared/purple viewpoints are distributed between the two UAVs.

A working/reference implementation is already provided.

The student can replace or improve it with another assignment strategy.

Examples:

- nearest-UAV assignment,
- balanced assignment,
- clustering,
- cost-based assignment,
- another custom strategy.

---

## TODO 2 — Student Local Path Planner

The second TODO is:

**`plan_path(...)`**

This method is used when running:

```bash
./run_lab1_student.sh
```

A working educational/reference implementation is already included.

The student should replace or improve it with their own collision-free local path planner.

Possible approaches include:

- RRT variants,
- PRM,
- graph search,
- visibility graph methods,
- another custom 3D path-planning algorithm.

---

## Provided Infrastructure — Do Not Modify for the Core Assignment

The following student-facing methods are already implemented and should normally be left unchanged:

```python
interpolate_headings(...)
resolve_start_delay(...)
```

They provide:

- heading interpolation along the geometric path,
- temporal collision avoidance between the two UAVs.

The remaining backend files are also provided infrastructure for the core exercise.

---

## Included Backend Files

The Lab 1 package contains the files required to reproduce the course behavior:

```text
mrim_task/mrim_planner/config/virtual_lab1.yaml

mrim_task/mrim_planner/scripts/planner_lab1_aura.py

mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py

mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py
mrim_task/mrim_planner/scripts/path_planners/grid_based/grid_3d.py

mrim_task/mrim_planner/scripts/path_planners/sampling_based/rrt.py

mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py

mrim_task/mrim_planner/scripts/trajectory.py
```

For the core assignment, students should **not modify these infrastructure files**.

The only file they are expected to edit is:

**`student_planner_lab1_aura.py`**

with the two main TODOs:

1. **`assign_shared_viewpoints(...)`**
2. **`plan_path(...)`**

---

## Launcher Behavior

The Lab 1 launcher temporarily activates the Lab 1-specific planner and configuration:

```text
planner_lab1_aura.py
        ↓
temporary planner.py

virtual_lab1.yaml
        ↓
temporary virtual.yaml
```

After the run finishes, the original backend files are restored.

This allows Lab 1, Lab 2, and Lab 3 to coexist on the same backend installation.
