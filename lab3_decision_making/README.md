# Lab 3 — Decision Making and Reinforcement Learning

## Goal

Study **which target each UAV should visit next**. Lab 3 is a high-level decision-making exercise; low-level collision-free motion between selected targets is still executed by the motion-planning backend.

Ownership rule:

- RED target -> RED UAV,
- BLUE target -> BLUE UAV,
- PURPLE/shared target -> assigned to RED or BLUE.

## Install

```bash
cd ~/Downloads/AURA_ROBOTICS_LAB/lab3_decision_making
./install_lab3.sh ~/Downloads/aura-lab
```

## Run

```bash
cd ~/Downloads/aura-lab

./run_lab3.sh random
./run_lab3.sh greedy
./run_lab3.sh q_learning
./run_lab3.sh tsp
```

Optional advanced mode:

```bash
./run_lab3.sh dqn
```

## Main student file

Edit:

```text
~/Downloads/aura-lab/.aura_runtime/backend/
  mrim_task/mrim_planner/scripts/student_agent.py
```

Student-agent interface:

```python
reset(initial_state)
select_action(state)
update(state, action, reward, next_state, done)
get_assignment()
```

## Student work / TODO areas

The main learning exercise is the decision policy in `student_agent.py`:

- choose legal next-target actions,
- implement/understand epsilon-greedy Q-learning action selection,
- update Q-values with the Bellman rule,
- compare learned behavior with random/greedy/TSP baselines,
- optionally implement a DQN extension.

The course file may include a working/reference Q-learning implementation so that the lab can run end-to-end; the exact exercise points are marked with TODO comments in `student_agent.py`.

## Relationship to Lab 1

Lab 3 decides **WHERE to go next**. The motion planner decides **HOW to get there**.

```text
StudentAgent / decision method
        -> next target(s)
        -> local A*/RRT motion planning
        -> heading / trajectory generation
        -> temporal multi-UAV deconfliction
```

## Included backend files

```text
mrim_task/mrim_planner/config/virtual_lab3.yaml
mrim_task/mrim_planner/scripts/planner_lab3_aura.py
mrim_task/mrim_planner/scripts/student_agent.py
mrim_task/mrim_planner/scripts/student_planner_lab3_aura.py
mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py
mrim_task/mrim_planner/scripts/path_planners/grid_based/grid_3d.py
mrim_task/mrim_planner/scripts/path_planners/sampling_based/rrt.py
mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py
mrim_task/mrim_planner/scripts/trajectory.py
```

The launcher temporarily activates the Lab 3 planner and `virtual_lab3.yaml`, and restores the original backend planner/config after the run.
