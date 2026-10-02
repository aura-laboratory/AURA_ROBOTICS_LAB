#!/usr/bin/env python3
"""
AURA Lab 3 - Student Decision-Making Agent
==========================================

This is the student-facing high-level decision-making interface.

Mission ownership rules
-----------------------
- RED viewpoints  -> must be visited by RED UAV
- BLUE viewpoints -> must be visited by BLUE UAV
- SHARED/PURPLE viewpoints -> may be assigned to either UAV

The agent does NOT generate trajectories.
It only decides which inspection target each UAV should visit next.

Lab 1 remains responsible for:
    target -> collision-free path -> trajectory

Lab 2 remains responsible for:
    sensor data -> pose estimate

Lab 3 is responsible for:
    state -> next high-level decision

Supported starter policies
--------------------------
- random      : working baseline
- greedy      : working nearest-target baseline
- q_learning  : student implementation
- dqn         : optional advanced extension

Expected state format
---------------------
The infrastructure passes a plain Python dictionary:

state = {
    "uavs": {
        "red": {
            "position": [x, y, z],
            "current_target": None,   # or target id
        },
        "blue": {
            "position": [x, y, z],
            "current_target": None,
        },
    },

    "targets": [
        {
            "id": 0,
            "position": [x, y, z],
            "color": "red",       # "red", "blue", "shared"/"purple"
            "visited": False,
        },
        ...
    ],

    "done": False,
}

Action format
-------------
select_action(state) returns:

{
    "red": target_id_or_None,
    "blue": target_id_or_None,
}

The AURA environment then executes those decisions using the reference
motion planner.

Reward/update
-------------
After execution the environment calls:

    update(state, action, reward, next_state, done)

For Q-learning the action is a JOINT decision:
    (red_target, blue_target)

so the student can learn coordination rather than two unrelated policies.
"""

import math
import random
from collections import defaultdict

import numpy as np


RED = "red"
BLUE = "blue"
SHARED = "shared"


def _normalize_color(color):
    c = str(color).strip().lower()

    if c in ("red", "r"):
        return RED

    if c in ("blue", "b"):
        return BLUE

    if c in ("shared", "purple", "common", "both", "p"):
        return SHARED

    raise ValueError("Unknown target color: {!r}".format(color))


def _distance(a, b):
    a = np.asarray(a, dtype=float).reshape(3)
    b = np.asarray(b, dtype=float).reshape(3)
    return float(np.linalg.norm(a - b))


class StudentAgent:
    """
    Common Lab-3 interface.

    Students may replace the internal decision algorithm while keeping the
    same interface used by the AURA infrastructure.
    """

    SUPPORTED_METHODS = (
        "random",
        "greedy",
        "q_learning",
        "dqn",
    )

    def __init__(
        self,
        method="greedy",
        shared_assignment="balanced_nearest",
        alpha=0.20,
        gamma=0.95,
        epsilon=0.20,
        random_seed=7,
    ):
        if method not in self.SUPPORTED_METHODS:
            raise ValueError(
                "Unknown method {!r}. Supported: {}".format(
                    method,
                    self.SUPPORTED_METHODS,
                )
            )

        self.method = method
        self.shared_assignment = shared_assignment

        # Q-learning parameters.
        self.alpha = float(alpha)
        self.gamma = float(gamma)
        self.epsilon = float(epsilon)

        self.rng = random.Random(random_seed)

        # target_id -> RED or BLUE
        #
        # Fixed-color targets are inserted immediately.
        # Shared targets are assigned during reset().
        self.owner = {}

        # Stable target ordering used by bitmask state encoding.
        self.target_ids = []
        self.target_to_bit = {}

        # Tabular Q-function:
        #   Q[(encoded_state, encoded_action)] = value
        self.q = defaultdict(float)

        self.initialized = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self, initial_state):
        """
        Start a new episode and create a legal target allocation.

        RED targets are always owned by RED.
        BLUE targets are always owned by BLUE.
        SHARED targets are distributed between RED and BLUE.
        """
        targets = self._targets(initial_state)

        self.target_ids = sorted(int(t["id"]) for t in targets)
        self.target_to_bit = {
            target_id: bit
            for bit, target_id in enumerate(self.target_ids)
        }

        self.owner = {}

        # First enforce the hard ownership constraints.
        for target in targets:
            target_id = int(target["id"])
            color = _normalize_color(target["color"])

            if color == RED:
                self.owner[target_id] = RED
            elif color == BLUE:
                self.owner[target_id] = BLUE

        # Then distribute shared/purple targets.
        self.assign_shared_targets(initial_state)

        self.initialized = True

    def select_action(self, state):
        """
        Return next target for BOTH UAVs.

        Output:
            {
                "red": target_id_or_None,
                "blue": target_id_or_None,
            }
        """
        self._require_initialized()

        if state.get("done", False):
            return {
                RED: None,
                BLUE: None,
            }

        if self.method == "random":
            return self._select_random(state)

        if self.method == "greedy":
            return self._select_greedy(state)

        if self.method == "q_learning":
            return self._select_q_learning(state)

        if self.method == "dqn":
            return self._select_dqn(state)

        raise RuntimeError("Unhandled method: {}".format(self.method))

    def update(self, state, action, reward, next_state, done):
        """
        Learn from one transition.

        Random/greedy baselines do not learn.
        Q-learning students implement the Bellman update here.
        """
        self._require_initialized()

        if self.method in ("random", "greedy"):
            return

        if self.method == "q_learning":
            self._update_q_learning(
                state,
                action,
                reward,
                next_state,
                done,
            )
            return

        if self.method == "dqn":
            self._update_dqn(
                state,
                action,
                reward,
                next_state,
                done,
            )
            return

    def get_assignment(self):
        """
        Return the current fixed ownership table.

        Example:
            {
                0: "red",
                1: "blue",
                2: "red",    # was shared
                3: "blue",   # was shared
            }
        """
        self._require_initialized()
        return dict(self.owner)

    # ------------------------------------------------------------------
    # Shared/purple viewpoint assignment
    # ------------------------------------------------------------------

    def assign_shared_targets(self, state):
        """
        Assign every SHARED/PURPLE target to exactly one UAV.

        The default 'balanced_nearest' baseline combines:
          - distance from each UAV to the shared target
          - current number of assigned targets

        This preserves the Lab-1 mission rule while keeping the two UAVs
        reasonably balanced.

        Students may replace this function with another allocation strategy.
        """
        targets = self._targets(state)

        red_position = self._uav_position(state, RED)
        blue_position = self._uav_position(state, BLUE)

        red_load = sum(1 for owner in self.owner.values() if owner == RED)
        blue_load = sum(1 for owner in self.owner.values() if owner == BLUE)

        shared_targets = [
            target
            for target in targets
            if _normalize_color(target["color"]) == SHARED
        ]

        # Deterministic order keeps experiments reproducible.
        shared_targets.sort(key=lambda target: int(target["id"]))

        for target in shared_targets:
            target_id = int(target["id"])
            position = target["position"]

            if self.shared_assignment == "random":
                chosen = self.rng.choice([RED, BLUE])

            elif self.shared_assignment == "balanced_nearest":
                red_distance = _distance(red_position, position)
                blue_distance = _distance(blue_position, position)

                # Small load penalty encourages distribution instead of giving
                # all shared targets to whichever UAV starts slightly closer.
                load_penalty = 2.0

                red_cost = red_distance + load_penalty * red_load
                blue_cost = blue_distance + load_penalty * blue_load

                chosen = RED if red_cost <= blue_cost else BLUE

            else:
                raise ValueError(
                    "Unknown shared assignment method: {}".format(
                        self.shared_assignment
                    )
                )

            self.owner[target_id] = chosen

            if chosen == RED:
                red_load += 1
            else:
                blue_load += 1

    # ------------------------------------------------------------------
    # Legal actions
    # ------------------------------------------------------------------

    def available_targets(self, state, uav):
        """
        Return legal, unvisited targets for one UAV.

        RED can never select a BLUE-owned target and vice versa.
        """
        uav = str(uav).lower()

        if uav not in (RED, BLUE):
            raise ValueError("uav must be 'red' or 'blue'")

        available = []

        for target in self._targets(state):
            target_id = int(target["id"])

            if bool(target.get("visited", False)):
                continue

            if self.owner.get(target_id) == uav:
                available.append(target_id)

        return available

    def valid_joint_actions(self, state):
        """
        Enumerate legal joint decisions.

        None means that a UAV has no remaining assigned target.

        Because shared targets were assigned once in reset(), RED and BLUE can
        never simultaneously choose the same target.
        """
        red_actions = self.available_targets(state, RED)
        blue_actions = self.available_targets(state, BLUE)

        if not red_actions:
            red_actions = [None]

        if not blue_actions:
            blue_actions = [None]

        return [
            (red_target, blue_target)
            for red_target in red_actions
            for blue_target in blue_actions
        ]

    # ------------------------------------------------------------------
    # Working baselines
    # ------------------------------------------------------------------

    def _select_random(self, state):
        """
        Working random baseline.
        """
        actions = self.valid_joint_actions(state)

        if not actions:
            return {RED: None, BLUE: None}

        red_target, blue_target = self.rng.choice(actions)

        return {
            RED: red_target,
            BLUE: blue_target,
        }

    def _select_greedy(self, state):
        """
        Working nearest-target baseline.

        Each UAV chooses the nearest remaining target from its own legal set.
        """
        action = {}

        for uav in (RED, BLUE):
            candidates = self.available_targets(state, uav)

            if not candidates:
                action[uav] = None
                continue

            uav_position = self._uav_position(state, uav)

            action[uav] = min(
                candidates,
                key=lambda target_id: _distance(
                    uav_position,
                    self._target_by_id(state, target_id)["position"],
                ),
            )

        return action

    # ------------------------------------------------------------------
    # Q-learning - STUDENT IMPLEMENTATION
    # ------------------------------------------------------------------

    def _select_q_learning(self, state):
        """
        Epsilon-greedy tabular Q-learning action selection.

        The action is JOINT:
            (red_next_target, blue_next_target)

        During training epsilon controls exploration.  The planner temporarily
        sets epsilon=0 for the final evaluation rollout.
        """
        encoded_state = self.encode_state(state)
        actions = self.valid_joint_actions(state)

        if not actions:
            return {RED: None, BLUE: None}

        # Exploration.
        if self.rng.random() < self.epsilon:
            chosen = self.rng.choice(actions)
            return self.decode_action(chosen)

        # Exploitation.  Random tie-breaking avoids a permanent bias toward
        # the first legal target when all Q-values are initially zero.
        values = [
            self.q[(encoded_state, action)]
            for action in actions
        ]
        best_value = max(values)

        best_actions = [
            action
            for action, value in zip(actions, values)
            if abs(value - best_value) < 1e-12
        ]

        chosen = self.rng.choice(best_actions)
        return self.decode_action(chosen)

    def _update_q_learning(
        self,
        state,
        action,
        reward,
        next_state,
        done,
    ):
        """
        Standard one-step tabular Q-learning Bellman update:

            Q(s,a) <- Q(s,a) + alpha *
                      [r + gamma * max_a' Q(s',a') - Q(s,a)]
        """
        s = self.encode_state(state)
        a = self.encode_action(action)

        old_q = self.q[(s, a)]

        if done:
            target = float(reward)
        else:
            s_next = self.encode_state(next_state)
            next_actions = self.valid_joint_actions(next_state)

            if next_actions:
                best_next = max(
                    self.q[(s_next, next_action)]
                    for next_action in next_actions
                )
            else:
                best_next = 0.0

            target = float(reward) + self.gamma * best_next

        self.q[(s, a)] = old_q + self.alpha * (target - old_q)

    # ------------------------------------------------------------------
    # DQN - OPTIONAL ADVANCED EXTENSION
    # ------------------------------------------------------------------

    def _select_dqn(self, state):
        raise NotImplementedError(
            "[AURA LAB 3 ADVANCED] Implement DQN action selection."
        )

    def _update_dqn(
        self,
        state,
        action,
        reward,
        next_state,
        done,
    ):
        raise NotImplementedError(
            "[AURA LAB 3 ADVANCED] Implement DQN training."
        )

    # ------------------------------------------------------------------
    # State/action encoding helpers for Q-learning
    # ------------------------------------------------------------------

    def encode_state(self, state):
        """
        Compact discrete state for tabular Q-learning.

        Returns:
            (
                red_current_target,
                blue_current_target,
                remaining_targets_bitmask,
            )

        The bitmask contains ALL unvisited targets. Ownership is already known
        through self.owner.
        """
        red_current = (
            state.get("uavs", {})
            .get(RED, {})
            .get("current_target", None)
        )

        blue_current = (
            state.get("uavs", {})
            .get(BLUE, {})
            .get("current_target", None)
        )

        remaining_mask = 0

        for target in self._targets(state):
            if bool(target.get("visited", False)):
                continue

            target_id = int(target["id"])
            bit = self.target_to_bit[target_id]
            remaining_mask |= (1 << bit)

        return (
            red_current,
            blue_current,
            remaining_mask,
        )

    @staticmethod
    def encode_action(action):
        """
        Convert action dictionary to hashable joint action.
        """
        return (
            action.get(RED, None),
            action.get(BLUE, None),
        )

    @staticmethod
    def decode_action(action_tuple):
        """
        Convert hashable joint action back to the environment format.
        """
        return {
            RED: action_tuple[0],
            BLUE: action_tuple[1],
        }

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def get_diagnostics(self, state=None):
        result = {
            "method": self.method,
            "shared_assignment": self.shared_assignment,
            "assigned_red": sum(
                1 for owner in self.owner.values() if owner == RED
            ),
            "assigned_blue": sum(
                1 for owner in self.owner.values() if owner == BLUE
            ),
            "q_entries": len(self.q),
            "epsilon": self.epsilon,
        }

        if state is not None:
            result["remaining_red"] = len(
                self.available_targets(state, RED)
            )
            result["remaining_blue"] = len(
                self.available_targets(state, BLUE)
            )

        return result

    # ------------------------------------------------------------------
    # Input helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _targets(state):
        targets = state.get("targets", None)

        if targets is None:
            raise ValueError(
                "State must contain a 'targets' list."
            )

        return targets

    @staticmethod
    def _uav_position(state, uav):
        try:
            return state["uavs"][uav]["position"]
        except KeyError:
            raise ValueError(
                "State is missing position for UAV {!r}.".format(uav)
            )

    def _target_by_id(self, state, target_id):
        for target in self._targets(state):
            if int(target["id"]) == int(target_id):
                return target

        raise KeyError("Unknown target id: {}".format(target_id))

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError(
                "Agent is not initialized. Call reset(initial_state) first."
            )


if __name__ == "__main__":
    # Tiny standalone demonstration of the ownership rules and greedy policy.
    demo_state = {
        "uavs": {
            "red": {
                "position": [0.0, 0.0, 0.0],
                "current_target": None,
            },
            "blue": {
                "position": [10.0, 0.0, 0.0],
                "current_target": None,
            },
        },
        "targets": [
            {
                "id": 0,
                "position": [1.0, 1.0, 0.0],
                "color": "red",
                "visited": False,
            },
            {
                "id": 1,
                "position": [9.0, 1.0, 0.0],
                "color": "blue",
                "visited": False,
            },
            {
                "id": 2,
                "position": [3.0, 2.0, 0.0],
                "color": "purple",
                "visited": False,
            },
            {
                "id": 3,
                "position": [7.0, 2.0, 0.0],
                "color": "shared",
                "visited": False,
            },
        ],
        "done": False,
    }

    agent = StudentAgent(
        method="greedy",
        shared_assignment="balanced_nearest",
    )

    agent.reset(demo_state)

    print("AURA Lab 3 demo")
    print("assignment:", agent.get_assignment())
    print("action:", agent.select_action(demo_state))
    print("diagnostics:", agent.get_diagnostics(demo_state))
