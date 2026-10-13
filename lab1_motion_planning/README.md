# Lab 1 — Motion Planning

**Path Planning • Task Allocation • Routing**

## Goal

Lab 1 combines path planning, task allocation, and routing in an integrated multi-UAV mission.

The exercise follows a complete robotics planning pipeline, where the final mission behavior depends on several connected stages rather than on a single algorithm.

During the lab, students will:

- compare two provided path planners, A* and RRT,
- implement a strategy for assigning shared viewpoints between two UAVs,
- study the effect of viewpoint assignment on the resulting routes,
- implement their own collision-free local path planner,
- evaluate how these choices affect the final mission execution.

The remaining parts of the pipeline, including TSP-based visit ordering, heading interpolation, trajectory generation and smoothing, and temporal collision avoidance between the UAVs, are provided.

---

## Installation

The simulation backend must be installed first.

See:

```text
~/Downloads/AURA_ROBOTICS_LAB/README_INSTALLATION.md
```

Then install Lab 1:

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab1_motion_planning

chmod +x install_lab1.sh
./install_lab1.sh ~/Downloads/aura-lab
```

---

## Running Lab 1

Move to:

```bash
cd ~/Downloads/aura-lab
```

The three Lab 1 launchers are:

```bash
./run_lab1_rrt.sh
./run_lab1_astar.sh
./run_lab1_student.sh
```

Use:

- `run_lab1_rrt.sh` for the provided RRT planner,
- `run_lab1_astar.sh` for the provided A* planner,
- `run_lab1_student.sh` for Questions B and C.

---

## Lab 1 Planning Pipeline

The mission is solved through the following stages:

```text
Inspection viewpoints
        ↓
Shared-viewpoint assignment
        ↓
TSP visit ordering
        ↓
Local path planning
        ↓
Heading interpolation
        ↓
Trajectory generation and smoothing
        ↓
Temporal two-UAV collision avoidance
        ↓
Final mission execution
```

The assignment stage determines **which UAV** receives each shared viewpoint.

The TSP stage determines **in which order** each UAV visits its assigned viewpoints.

The local path planner determines **how the UAV moves collision-free** between consecutive viewpoints.

---

# Question A — Provided Path Planners

Question A uses the provided backend implementations of:

- A*
- RRT

No planner implementation is required for this part.

Run:

```bash
./run_lab1_astar.sh
```

and:

```bash
./run_lab1_rrt.sh
```

The two planners are evaluated using the same mission setup and viewpoint assignment.

Students should compare their behavior and the resulting mission performance using the measurements requested in the Lab assignment.

The A* implementation is located at:

```text
mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py
```

The RRT implementation is located at:

```text
mrim_task/mrim_planner/scripts/path_planners/sampling_based/rrt.py
```

These files are provided for inspection and should not be modified for the Lab assignment.

---

# Question B — Shared Viewpoint Assignment

Some viewpoints can only be inspected by one specific UAV. These viewpoints are assigned automatically.

Shared/purple viewpoints can be inspected by either UAV.

For Question B, students must implement their own strategy for deciding which UAV receives each shared viewpoint.

The relevant function is:

```python
assign_shared_viewpoints(self, problem, viewpoints)
```

It is located in:

```text
mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

A deterministic K-Means reference implementation is provided so that the complete Lab pipeline can run before the student implementation is added.

Each shared viewpoint must be assigned to exactly one UAV.

Possible strategies may consider:

- distance from the UAVs,
- balanced workload,
- clustering,
- estimated route cost,
- obstacle information,
- combinations of these criteria.

Run the student configuration with:

```bash
./run_lab1_student.sh
```

The terminal output shows which shared viewpoints were assigned to each UAV.

For example:

```text
[AURA LAB 1] SHARED VIEWPOINT ASSIGNMENT
   UAV ID 1: 2 shared viewpoints -> [5, 8]
   UAV ID 2: 2 shared viewpoints -> [6, 7]
```

These IDs can be used when reporting the resulting assignment.

---

# Question C — Student Path Planner

For Question C, students implement their own collision-free local path planner.

The relevant function is:

```python
plan_path(self, p_from, p_to, problem, planner_config)
```

in:

```text
mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

A small reference RRT implementation is provided so that the student mode can run before the custom planner is implemented.

The student should replace the reference call inside `plan_path(...)` with their own planner.

Possible approaches include:

- RRT*,
- PRM,
- graph-based planning,
- another suitable collision-free 3D path-planning method.

The returned path must:

- start at `p_from`,
- end at `p_to`,
- respect the obstacle clearance,
- contain any required intermediate points.

Intermediate headings do not need to be generated by the student planner. They are handled later by the provided heading-interpolation stage.

Run:

```bash
./run_lab1_student.sh
```

For Question C, modify only the implementation of `plan_path(...)`.

The assignment strategy implemented in Question B can be kept. If the Question B implementation is not working correctly, the provided reference K-Means assignment can be used while testing the path planner.

---

# Student File

For Questions B and C, students should modify only:

```text
student_planner_lab1_aura.py
```

Full path after installation:

```text
~/Downloads/aura-lab/.aura_runtime/backend/
mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

The two student tasks are:

```python
assign_shared_viewpoints(...)
plan_path(...)
```

The rest of the Lab pipeline is provided.

---

# Provided Infrastructure

The following stages are already implemented and should not be modified for the Lab assignment:

- fixed RED/BLUE viewpoint assignment,
- TSP visit ordering,
- heading interpolation,
- trajectory generation,
- trajectory sampling and smoothing,
- velocity and acceleration constraint handling,
- temporal two-UAV collision avoidance.

In `student_planner_lab1_aura.py`, the following methods are provided:

```python
interpolate_headings(...)
resolve_start_delay(...)
```

They should remain unchanged.

---

# Main Lab 1 Files

The main Lab 1 files are:

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

Only `student_planner_lab1_aura.py` is intended to be modified by students for the Lab assignment.

---

# Opening the Project in PyCharm

The backend can be opened inside the Apptainer environment using:

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend

./simulation/pycharm.sh
```

Then navigate to:

```text
mrim_task/mrim_planner/scripts/
```

and open:

```text
student_planner_lab1_aura.py
```

---

# Notes

- Do not modify `virtual_lab1.yaml` to switch between A*, RRT, and STUDENT modes.
- Use the provided launcher scripts instead.
- Do not modify the provided A* or RRT implementations for Question A.
- Do not modify the provided TSP, heading interpolation, trajectory-generation, or collision-avoidance stages.
- Failed planning runs are still useful experimental results and should not simply be discarded.
