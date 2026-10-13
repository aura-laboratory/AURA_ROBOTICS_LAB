#!/usr/bin/env python3
"""AURA Lab 1 - student-facing motion-planning logic.

The same StudentPlanner object is used in all three Lab 1 modes:

  * RRT mode     -> backend RRT for local paths
  * A* mode      -> backend A* for local paths
  * STUDENT mode -> StudentPlanner.plan_path() for local paths

In every mode, the Lab 1 pipeline also performs:

  1) assignment of shared/purple viewpoints,
  2) heading interpolation,
  3) trajectory generation and smoothing,
  4) temporal two-UAV collision avoidance.

Student exercises
-----------------
Question B:
    Replace assign_shared_viewpoints() with your own shared-viewpoint
    assignment strategy.

Question C:
    Replace plan_path() with your own collision-free local path planner.

A working reference implementation is provided for both functions so that
the Lab can run before you implement your own solutions.

For Question C, keep the assignment strategy implemented in Question B.
If your assignment does not work correctly, the provided reference K-Means
assignment may be used for evaluating the path planner.

Heading interpolation, trajectory generation/smoothing and temporal
two-UAV collision avoidance are provided parts of the Lab pipeline and
should not be modified.
"""

import itertools
import math
import time

import numpy as np
from scipy.spatial import KDTree
from sklearn.cluster import KMeans

from data_types import Pose


def _wrap_angle(angle):
    """Wrap angle to (-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _xyz(point):
    return np.array(
        [float(point.x), float(point.y), float(point.z)],
        dtype=float,
    )


def _distance(a, b):
    return float(np.linalg.norm(_xyz(a) - _xyz(b)))


class StudentPlanner:
    """Student-facing AURA Lab 1 planning policy."""

    name = "AURA Lab 1 Student Planner"

    def __init__(
        self,
        dt,
        safety_margin=0.15,
        max_start_delay=60.0,
        random_seed=7,
    ):
        self.dt = float(dt)
        self.safety_margin = float(safety_margin)
        self.max_start_delay = float(max_start_delay)
        self.rng = np.random.default_rng(int(random_seed))

        # Cached environment data used only by the STUDENT local planner.
        self._cached_problem_id = None
        self._obstacle_tree = None
        self._bounds_min = None
        self._bounds_max = None

    # ==================================================================
    # QUESTION B: SHARED/PURPLE VIEWPOINT ASSIGNMENT
    # ==================================================================
    def assign_shared_viewpoints(self, problem, viewpoints):
        """Assign shared/purple viewpoints to the UAVs.

        STUDENT TASK — QUESTION B
        -------------------------
        Replace the reference policy below with your own assignment strategy.

        Your method must decide which UAV receives each shared/purple
        viewpoint.

        Each shared viewpoint must be assigned to exactly one UAV.
        """

        # QUESTION B:
        # Replace this call with your own assignment strategy.
        return self._reference_assign_shared_viewpoints(problem, viewpoints)

    def _reference_assign_shared_viewpoints(self, problem, viewpoints):
        """Reference solution for Question B: deterministic K-Means assignment."""

        k = int(problem.number_of_robots)
        clusters = [[] for _ in range(k)]

        if not viewpoints:
            return clusters

        positions = np.array(
            [_xyz(vp.pose.point) for vp in viewpoints]
        )

        starts = np.array([
            [
                float(problem.start_poses[r].position.x),
                float(problem.start_poses[r].position.y),
                float(problem.start_poses[r].position.z),
            ]
            for r in range(k)
        ])

        # Fewer shared viewpoints than UAVs:
        # assign each viewpoint to the nearest UAV start.
        if len(viewpoints) < k:
            for vp, pos in zip(viewpoints, positions):
                robot_idx = int(
                    np.argmin(
                        np.linalg.norm(starts - pos, axis=1)
                    )
                )
                clusters[robot_idx].append(vp)

            return clusters

        # Reference deterministic K-Means assignment.
        model = KMeans(
            n_clusters=k,
            random_state=0,
            n_init=10,
        )

        labels = model.fit_predict(positions)
        centers = model.cluster_centers_

        # K-Means cluster labels are arbitrary.
        #
        # Map the clusters one-to-one to the UAV start positions using
        # the assignment with the smallest total start-to-center distance.
        best_perm = None
        best_cost = float("inf")

        for perm in itertools.permutations(range(k)):
            cost = 0.0

            for c in range(k):
                cost += float(
                    np.linalg.norm(
                        centers[c] - starts[perm[c]]
                    )
                )

            if cost < best_cost:
                best_cost = cost
                best_perm = perm

        for vp, label in zip(viewpoints, labels):
            robot_idx = int(best_perm[int(label)])
            clusters[robot_idx].append(vp)

        print(
            "[AURA LAB 1] shared-viewpoint assignment:",
            [len(c) for c in clusters],
            "viewpoints",
        )

        return clusters

    # ==================================================================
    # QUESTION C: STUDENT LOCAL PATH PLANNER
    # ==================================================================
    def plan_path(
        self,
        p_from,
        p_to,
        problem,
        planner_config,
    ):
        """Plan one collision-free local path for STUDENT mode.

        This method is called ONLY when running::

            ./run_lab1_student.sh

        RRT and A* modes use the provided backend implementations instead.

        Question C uses the assignment currently returned by
        assign_shared_viewpoints(). Normally this should be the strategy
        implemented in Question B.

        Parameters
        ----------
        p_from, p_to : Pose
            Start and goal poses for one TSP edge.

        problem : InspectionProblem
            Contains obstacle points, safety area and altitude limits.

        planner_config : dict
            Lab 1 planner parameters.

        Returns
        -------
        path : list[Pose]
            Start pose, optional intermediate poses, goal pose.

        distance : float
            Geometric path length.

        STUDENT TASK — QUESTION C
        -------------------------
        Replace the reference implementation below with your own
        collision-free local path planner.

        Possible approaches include RRT*, PRM or another suitable
        path-planning method.

        Contract
        --------
          * path[0] must be p_from,
          * path[-1] must be p_to,
          * the path must respect obstacle clearance,
          * intermediate headings may be None,
          * heading interpolation is performed later by the provided
            Lab 1 pipeline.
        """

        # QUESTION C:
        # Replace this call with your own local path planner.
        return self._reference_student_rrt(
            p_from,
            p_to,
            problem,
            planner_config,
        )

    # ==================================================================
    # REFERENCE STUDENT PATH PLANNER
    # ==================================================================
    def _reference_student_rrt(
        self,
        p_from,
        p_to,
        problem,
        planner_config,
    ):
        """Small educational RRT used as the reference solution.

        This implementation is intentionally independent from the
        backend's rrt.py.

        It allows the Lab to run before Question C is implemented and
        provides a simple working example that can be studied or replaced.
        """

        self._prepare_environment(problem)

        start = _xyz(p_from.point)
        goal = _xyz(p_to.point)

        clearance = float(
            planner_config["safety_distance"]
        )

        timeout = float(
            planner_config.get(
                "timeout",
                60.0,
            )
        )

        step_size = float(
            planner_config.get(
                "student/step_size",
                3.0,
            )
        )

        goal_bias = float(
            planner_config.get(
                "student/goal_bias",
                0.20,
            )
        )

        max_iterations = int(
            planner_config.get(
                "student/max_iterations",
                4000,
            )
        )

        collision_step = float(
            planner_config.get(
                "student/collision_check_step",
                0.25,
            )
        )

        # --------------------------------------------------------------
        # Fast path
        #
        # If the start and goal can already be connected directly
        # without violating obstacle clearance, use the straight segment.
        # --------------------------------------------------------------
        if self._segment_is_free(
            start,
            goal,
            clearance,
            collision_step,
        ):
            return (
                [p_from, p_to],
                float(np.linalg.norm(goal - start)),
            )

        # RRT tree.
        nodes = [start.copy()]
        parents = [-1]

        start_time = time.monotonic()
        goal_parent = None

        for _ in range(max_iterations):

            if time.monotonic() - start_time > timeout:
                break

            # ----------------------------------------------------------
            # Goal-biased random sampling
            # ----------------------------------------------------------
            if self.rng.random() < goal_bias:
                sample = goal
            else:
                sample = self.rng.uniform(
                    self._bounds_min,
                    self._bounds_max,
                )

            # ----------------------------------------------------------
            # Find nearest node in the current RRT
            # ----------------------------------------------------------
            node_array = np.asarray(nodes)

            nearest_idx = int(
                np.argmin(
                    np.linalg.norm(
                        node_array - sample,
                        axis=1,
                    )
                )
            )

            nearest = nodes[nearest_idx]

            # ----------------------------------------------------------
            # Extend toward the sample
            # ----------------------------------------------------------
            direction = sample - nearest
            direction_norm = float(
                np.linalg.norm(direction)
            )

            if direction_norm <= 1e-9:
                continue

            step = min(
                step_size,
                direction_norm,
            )

            new_node = (
                nearest
                + direction / direction_norm * step
            )

            # Keep samples inside the allowed environment bounds.
            if not self._inside_bounds(new_node):
                continue

            # Check the new RRT edge against obstacles.
            if not self._segment_is_free(
                nearest,
                new_node,
                clearance,
                collision_step,
            ):
                continue

            nodes.append(new_node)
            parents.append(nearest_idx)

            new_idx = len(nodes) - 1

            # ----------------------------------------------------------
            # Try connecting the newest node directly to the goal
            # ----------------------------------------------------------
            if self._segment_is_free(
                new_node,
                goal,
                clearance,
                collision_step,
            ):
                goal_parent = new_idx
                break

        # --------------------------------------------------------------
        # Planning failed
        # --------------------------------------------------------------
        if goal_parent is None:
            raise RuntimeError(
                "[AURA LAB 1 STUDENT] Reference student planner "
                "could not find a collision-free path. "
                "Implement or improve StudentPlanner.plan_path()."
            )

        # --------------------------------------------------------------
        # Reconstruct tree branch:
        #
        # start -> ... -> goal
        # --------------------------------------------------------------
        xyz_path = [goal.copy()]

        idx = goal_parent

        while idx >= 0:
            xyz_path.append(
                nodes[idx].copy()
            )
            idx = parents[idx]

        xyz_path.reverse()

        # --------------------------------------------------------------
        # Convert XYZ samples back to Pose objects.
        #
        # Preserve the exact start/end Pose objects and headings.
        # Intermediate headings are intentionally left undefined.
        #
        # They are filled later by interpolate_headings().
        # --------------------------------------------------------------
        pose_path = [p_from]

        for xyz in xyz_path[1:-1]:
            pose_path.append(
                Pose(
                    float(xyz[0]),
                    float(xyz[1]),
                    float(xyz[2]),
                    None,
                )
            )

        pose_path.append(p_to)

        # --------------------------------------------------------------
        # Geometric path length
        # --------------------------------------------------------------
        distance = 0.0

        for i in range(
            1,
            len(pose_path),
        ):
            distance += _distance(
                pose_path[i - 1].point,
                pose_path[i].point,
            )

        print(
            "[AURA LAB 1 STUDENT] "
            "local path: {} poses, {:.2f} m".format(
                len(pose_path),
                distance,
            )
        )

        return pose_path, distance

    # ==================================================================
    # INTERNAL HELPERS USED BY THE REFERENCE STUDENT PLANNER
    # ==================================================================
    def _prepare_environment(self, problem):
        """Cache obstacle KD-tree and simple 3D sampling bounds."""

        problem_id = id(problem)

        if self._cached_problem_id == problem_id:
            return

        obstacles = np.array(
            [
                [o.x, o.y, o.z]
                for o in problem.obstacle_points
            ],
            dtype=float,
        )

        self._obstacle_tree = (
            KDTree(obstacles)
            if len(obstacles) > 0
            else None
        )

        xs = [
            float(p.x)
            for p in problem.safety_area
        ]

        ys = [
            float(p.y)
            for p in problem.safety_area
        ]

        self._bounds_min = np.array(
            [
                min(xs),
                min(ys),
                float(problem.min_height),
            ],
            dtype=float,
        )

        self._bounds_max = np.array(
            [
                max(xs),
                max(ys),
                float(problem.max_height),
            ],
            dtype=float,
        )

        self._cached_problem_id = problem_id

    def _inside_bounds(self, xyz):
        """Check whether a sampled point is inside the allowed 3D bounds."""

        return bool(
            np.all(
                xyz >= self._bounds_min
            )
            and np.all(
                xyz <= self._bounds_max
            )
        )

    def _segment_is_free(
        self,
        start,
        goal,
        clearance,
        sample_step,
    ):
        """Collision-check a segment against the obstacle point cloud."""

        if self._obstacle_tree is None:
            return True

        delta = goal - start

        length = float(
            np.linalg.norm(delta)
        )

        if length <= 1e-9:
            distances, _ = (
                self._obstacle_tree.query(start)
            )

            return (
                float(distances)
                >= clearance
            )

        n = max(
            2,
            int(
                math.ceil(
                    length / sample_step
                )
            )
            + 1,
        )

        alpha = np.linspace(
            0.0,
            1.0,
            n,
        )

        samples = (
            start[None, :]
            + alpha[:, None]
            * delta[None, :]
        )

        distances, _ = (
            self._obstacle_tree.query(samples)
        )

        return bool(
            np.all(
                np.asarray(distances)
                >= clearance
            )
        )

    # ==================================================================
    # PROVIDED: HEADING INTERPOLATION
    #
    # DO NOT MODIFY FOR THE LAB ASSIGNMENT
    # ==================================================================
    def interpolate_headings(self, waypoints):
        """Fill missing headings using distance-based interpolation.

        The local path planner may return intermediate points without
        headings.

        This provided function interpolates the UAV heading between
        viewpoints that have defined inspection headings.

        This stage is part of the provided Lab 1 pipeline and is not a
        student task.
        """

        if not waypoints:
            return waypoints

        if (
            waypoints[0].heading is None
            or waypoints[-1].heading is None
        ):
            raise ValueError(
                "First and last waypoint "
                "must have a defined heading"
            )

        idx = 0

        while idx < len(waypoints) - 1:

            end_idx = None

            # Find the next waypoint whose heading is defined.
            for j in range(
                idx + 1,
                len(waypoints),
            ):
                if (
                    waypoints[j].heading
                    is not None
                ):
                    end_idx = j
                    break

            if end_idx is None:
                raise ValueError(
                    "No closing waypoint "
                    "with a defined heading"
                )

            start_heading = _wrap_angle(
                waypoints[idx].heading
            )

            end_heading = _wrap_angle(
                waypoints[end_idx].heading
            )

            delta_heading = _wrap_angle(
                end_heading
                - start_heading
            )

            # ----------------------------------------------------------
            # Compute geometric distance between the two viewpoints
            # ----------------------------------------------------------
            total_length = 0.0

            for j in range(
                idx + 1,
                end_idx + 1,
            ):
                total_length += _distance(
                    waypoints[j - 1].point,
                    waypoints[j].point,
                )

            # ----------------------------------------------------------
            # Interpolate heading according to travelled distance
            # ----------------------------------------------------------
            travelled = 0.0

            for j in range(
                idx + 1,
                end_idx,
            ):
                travelled += _distance(
                    waypoints[j - 1].point,
                    waypoints[j].point,
                )

                alpha = (
                    0.0
                    if total_length <= 1e-9
                    else travelled / total_length
                )

                waypoints[j].heading = (
                    _wrap_angle(
                        start_heading
                        + alpha * delta_heading
                    )
                )

            # Keep the required inspection heading at the endpoint.
            waypoints[end_idx].heading = (
                end_heading
            )

            idx = end_idx

        return waypoints

    # ==================================================================
    # PROVIDED: TEMPORAL TWO-UAV COLLISION AVOIDANCE
    #
    # DO NOT MODIFY FOR THE LAB ASSIGNMENT
    # ==================================================================
    def resolve_start_delay(
        self,
        trajectory_utils,
        trajectories,
        safety_distance,
    ):
        """Resolve two-UAV temporal conflicts by delaying one UAV.

        After the trajectories have been generated, they are checked for
        possible UAV-UAV collisions.

        If a collision exists, the UAV with the shorter trajectory is
        delayed at its start in increments of self.dt until the temporal
        conflict disappears or max_start_delay is reached.

        This stage is provided and is not a student task.
        """

        if len(trajectories) != 2:
            return (
                trajectories,
                [],
                [],
            )

        # Select the UAV with the shorter trajectory duration.
        times = [
            t.getTime()
            for t in trajectories
        ]

        delay_idx = int(
            np.argmin(times)
        )

        other_idx = 1 - delay_idx

        check_distance = (
            float(safety_distance)
            + self.safety_margin
        )

        # Initial collision check.
        collision, _ = (
            trajectory_utils.trajectoriesCollide(
                trajectories[delay_idx],
                trajectories[other_idx],
                check_distance,
            )
        )

        delay = 0.0

        # --------------------------------------------------------------
        # Delay the selected UAV at its start until there is no temporal
        # collision or the maximum allowed delay is reached.
        # --------------------------------------------------------------
        while (
            collision
            and delay + self.dt
            <= self.max_start_delay + 1e-9
        ):
            trajectories[
                delay_idx
            ].delayStart(self.dt)

            delay += self.dt

            collision, _ = (
                trajectory_utils.trajectoriesCollide(
                    trajectories[delay_idx],
                    trajectories[other_idx],
                    check_distance,
                )
            )

        if collision:
            print(
                "[AURA] WARNING: no collision-free "
                "start delay found within {:.1f} s".format(
                    self.max_start_delay
                )
            )
        else:
            print(
                "[AURA] temporal deconfliction: "
                "delayed UAV index {} by {:.1f} s".format(
                    delay_idx,
                    delay,
                )
            )

        if delay <= 1e-9:
            return (
                trajectories,
                [],
                [],
            )

        return (
            trajectories,
            [delay_idx],
            [delay],
        )