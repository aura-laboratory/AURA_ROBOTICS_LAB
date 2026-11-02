# Lab 1 — Motion Planning

**Path Planning • Task Allocation • Routing**  
**Robotics 2026–2027**

## Goal

Lab 1 combines path planning, task allocation, and routing in an integrated multi-UAV inspection mission. Two UAVs, **RED** and **BLUE**, must visit viewpoints in a 3D environment with obstacles. Some viewpoints are assigned to a specific UAV; shared (purple) viewpoints may be assigned to either UAV.

Students will:

- study the provided **A\*** grid-based path planner and identify how it uses a heuristic function;
- explain, at a conceptual level, how A\* could be converted into Dijkstra's algorithm;
- modify the A\* heuristic and experimentally compare the results with the original implementation;
- implement a strategy for assigning shared viewpoints between the two UAVs;
- optionally implement their own collision-free local path planner (**bonus**).

The remaining pipeline stages—TSP-based visit ordering, heading interpolation, trajectory generation and smoothing, and temporal collision avoidance—are provided. They are not implementation tasks in this lab.

**Required:** Questions A and B. **Optional bonus:** Question C.  
For the full question wording and report requirements, refer to the Lab 1 assignment handout.

---

## Installation

The simulation backend must be installed first. Follow:

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

From the Lab directory:

```bash
cd ~/Downloads/aura-lab
```

Use the following launchers for the required tasks:

```bash
./run_lab1_astar.sh    # Question A: original or modified A* heuristic
./run_lab1_student.sh  # Question B: viewpoint assignment; Question C: bonus planner
```

### Open the project in PyCharm

```bash
cd ~/Downloads/aura-lab/.aura_runtime/backend
./simulation/pycharm.sh
```

You can then navigate to the Python files specified in the questions below.

---

## Lab 1 Planning Pipeline

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

- **Assignment:** Which UAV visits each shared viewpoint?
- **Routing / TSP:** In which order are the assigned viewpoints visited?
- **Path planning:** How does each UAV travel collision-free between consecutive viewpoints?
- **Trajectory planning:** How are the paths converted into time-parameterized trajectories that respect the UAVs' motion constraints?

---

## Question A — Study and Modification of the A* Path Planner

The A\* implementation is a grid-based planner that searches for a collision-free route between viewpoints in a discrete 3D environment.

**File to inspect and, for A.3, modify:**

```text
~/Downloads/aura-lab/.aura_runtime/backend/mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py
```

### A.1 — Understanding A*

Read the existing code and answer:

1. Which heuristic function does the provided A\* implementation use? Write its mathematical expression and explain what it computes.
2. What do `g(n)`, `h(n)` and `f(n)` represent in A\*? Identify the corresponding variables in the code and explain how the next node to expand is selected.
3. **Run the original A\*** without modifying the code:

   ```bash
   cd ~/Downloads/aura-lab
   ./run_lab1_astar.sh
   ```

   Record the following baseline results:

   - **Final Time:** Duration of the final mission.
   - **Final Score:** Final score reported by the system.
   - **Remaining Solution time:** Computation time needed to produce a solution, as reported by the system.
   - **Trajectory length:** Length of the trajectory for **each** UAV.
   - **UAV Dynamics:** `OK` or `FAILED` for RED and BLUE; if failed, identify the violated limit.
   - **Screenshot evidence:** At least one screenshot showing the final results.

> **Solution Time is not Final Time.** Solution Time measures how long the system needs to compute the mission solution. Final Time measures how long the planned mission takes to execute. A faster planner does not necessarily produce a shorter mission.

### A.2 — From A* to Dijkstra (theoretical)

How could the existing A\* algorithm be converted into Dijkstra's algorithm? Explain the required changes conceptually.

**No code modification or execution is required for A.2.**

### A.3 — Experimenting with a Different Heuristic

Modify the existing A\* heuristic function, choosing a **different method of estimating the remaining cost to the goal**. Do not replace the entire planner or modify other pipeline stages.

Run the mission again using:

```bash
cd ~/Downloads/aura-lab
./run_lab1_astar.sh
```

Record the **same measurements as in A.1** and compare the new execution with the baseline.

In the report, explain:

1. Which heuristic did you select, and why?
2. What differences did you observe compared with the original A\* implementation?
3. How do you explain the observed differences, **or the absence of differences**?

Differences in Solution Time, trajectory length or Final Time are **not guaranteed**. An unchanged result is still a valid experimental observation.

**Important:** Complete and save the A.1 baseline **before** modifying the heuristic for A.3.

### If a run fails

If planning times out, a collision-free path is not found, or playback is not reached, do not discard the run. Record it as `FAILED`, keep a screenshot or relevant terminal output, and briefly discuss the possible reason.

---

## Question B — Multi-UAV Shared Viewpoint Assignment

Three viewpoint categories exist:

- **RED:** Inspected by the RED UAV only.
- **BLUE:** Inspected by the BLUE UAV only.
- **PURPLE / shared:** Can be inspected by either UAV.

RED and BLUE viewpoints are assigned automatically. For **shared viewpoints**, implement your own assignment strategy, assigning **every shared viewpoint exactly once** to one of the UAVs.

The function to modify is:

```python
assign_shared_viewpoints(self, problem, viewpoints)
```

in:

```text
~/Downloads/aura-lab/.aura_runtime/backend/mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

The provided reference implementation calls:

```python
return self._reference_assign_shared_viewpoints(problem, viewpoints)
```

This is a **K-Means** reference solution for demonstration and must be replaced by your own assignment method for Question B.

Possible criteria include distance, balancing workload, clustering, estimated route cost, obstacle-aware cost, or combinations of these. Your strategy should have a clear cost or grouping criterion; a global optimum is **not required**.

### Provided routing infrastructure

After viewpoint assignment, the existing `TSPSolver3D` builds a distance matrix and uses the provided LKH solver to determine the visit order. The TSP solver **must not be modified**.

```text
mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py
```

For Question B, the student configuration uses a provided reference local path planner with a fixed random seed to reduce variability. Students are only required to implement the viewpoint-assignment strategy.

### Run Question B

```bash
cd ~/Downloads/aura-lab
./run_lab1_student.sh
```

The terminal prints the shared viewpoints assigned to each UAV, for example:

```text
[AURA LAB 1] SHARED VIEWPOINT ASSIGNMENT
   UAV ID 1: 2 shared viewpoints -> [5, 8]
   UAV ID 2: 2 shared viewpoints -> [6, 7]
```

In the report, describe:

1. Your assignment strategy and how it works.
2. The objective or cost criterion it attempts to improve.
3. How many and which shared viewpoint IDs were assigned to RED and BLUE.
4. The effect of your choice on Final Time compared with the reference K-Means solution, and why an assignment need not be optimal for every objective.

---

## Question C — Student Path Planner (Optional Bonus)

Question C is **optional** and has no penalty if not attempted. Implement your own collision-free local path planner by replacing the provided reference call inside:

```python
plan_path(self, p_from, p_to, problem, planner_config)
```

in the **same student file**:

```text
~/Downloads/aura-lab/.aura_runtime/backend/mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
```

The `plan_path(...)` method initially uses a provided reference implementation. For the optional bonus, replace this reference call with your own path-planning method.

You may replace it with a suitable path-planning method. The planner must generate a valid, obstacle-free path between consecutive viewpoints and respect the problem constraints. You may retain the assignment from Question B, or use the provided K-Means reference assignment if needed for testing.

Do **not** modify the provided temporal collision-avoidance stage.

Run the student planner **three times**, without changing the parameters:

```bash
cd ~/Downloads/aura-lab
./run_lab1_student.sh
```

For **each run**, report Final Time, Final Score, Remaining Solution time, trajectory length, UAV Dynamics, and screenshot evidence. Include failed runs in the results. Briefly describe your method and compare its performance with the results from Question A.

---


## Main Lab 1 Files

```text
mrim_task/mrim_planner/config/virtual_lab1.yaml
mrim_task/mrim_planner/scripts/planner_lab1_aura.py
mrim_task/mrim_planner/scripts/student_planner_lab1_aura.py
mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py
mrim_task/mrim_planner/scripts/path_planners/grid_based/grid_3d.py
mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py
mrim_task/mrim_planner/scripts/trajectory.py
```
