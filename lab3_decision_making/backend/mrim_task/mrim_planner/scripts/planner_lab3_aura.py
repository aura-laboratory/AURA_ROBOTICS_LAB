#!/usr/bin/env python3
"""
AURA integrated planner for Labs 1 and 3.

Lab 1 mode (decision_making/method = "tsp"):
  - StudentPlanner assigns shared/purple viewpoints,
  - backend TSP chooses visit order,
  - configured A*/RRT planner connects consecutive viewpoints.

Lab 3 modes (random / greedy / q_learning / dqn):
  - RED inspection points remain assigned to RED,
  - BLUE inspection points remain assigned to BLUE,
  - SHARED/PURPLE points are assigned by StudentAgent,
  - StudentAgent chooses the high-level visit order,
  - the existing A*/RRT local planner still computes collision-free legs.

In every mode the existing StudentPlanner hooks still provide:
  - heading interpolation,
  - temporal multi-UAV deconfliction.
"""

import rospy
import rospkg
import numpy as np

from utils import *
from solvers.tsp_solvers import *
from trajectory import Trajectory, TrajectoryUtils
from student_planner_lab3_aura import StudentPlanner
from student_agent import StudentAgent


class MrimPlanner:

    ALLOWED_COLLISION_AVOIDANCE_METHODS = [
        'none',
        'delay_2nd_till_1st_UAV_finishes',
        'delay_till_no_collisions_occur',
    ]

    def __init__(self):

        rospy.init_node('mrim_planner', anonymous=True)

        # ---------------------- load problem ----------------------
        problem_filename = rospy.get_param('~problem/name')
        session_problem = rospy.get_param('~session_problem')

        problem_filepath = (
            rospkg.RosPack().get_path('mrim_resources')
            + "/problems/"
            + problem_filename
        )

        problem, log_msg = ProblemLoader().loadProblem(problem_filepath)

        if problem is None:
            rospy.logerr(log_msg)
            rospy.signal_shutdown(log_msg)
            exit(-1)

        # -------- load parameters from ROS custom config --------
        self._viewpoints_distance = rospy.get_param(
            '~viewpoints/distance', 3.0
        )
        self._plot = rospy.get_param(
            '~problem/plot', False
        )
        self._trajectory_dt = rospy.get_param(
            '~trajectories/dt', 0.2
        )
        self._smoothing_sampling_step = rospy.get_param(
            '~path_smoothing/sampling_step', 0.1
        )
        self._smoothing_distance = rospy.get_param(
            '~path_smoothing/lookahead_dist', 0.3
        )
        self._sample_with_stops = rospy.get_param(
            '~trajectory_sampling/with_stops', True
        )
        self._global_frame = rospy.get_param(
            '~global_frame', "gps_origin"
        )
        self._tsp_clustering_method = rospy.get_param(
            '~tsp/clustering', 'random'
        )

        # ---------------- Lab 3 decision making ----------------
        # Default "tsp" preserves the exact Lab 1 behavior.
        self._decision_method = rospy.get_param(
            '~decision_making/method', 'tsp'
        )
        self._shared_assignment = rospy.get_param(
            '~decision_making/shared_assignment', 'balanced_nearest'
        )
        self._rl_alpha = rospy.get_param(
            '~decision_making/alpha', 0.20
        )
        self._rl_gamma = rospy.get_param(
            '~decision_making/gamma', 0.95
        )
        self._rl_epsilon = rospy.get_param(
            '~decision_making/epsilon', 0.20
        )
        self._rl_training_episodes = rospy.get_param(
            '~decision_making/training_episodes', 1200
        )
        self._rl_completion_bonus = rospy.get_param(
            '~decision_making/completion_bonus', 100.0
        )
        self._rl_inspection_reward = rospy.get_param(
            '~decision_making/inspection_reward', 20.0
        )
        self._rl_seed = rospy.get_param(
            '~decision_making/random_seed', 7
        )

        allowed_decision_methods = (
            'tsp',
            'random',
            'greedy',
            'q_learning',
            'dqn',
        )
        if self._decision_method not in allowed_decision_methods:
            raise ValueError(
                'Unsupported decision_making/method: {}. Allowed: {}'.format(
                    self._decision_method,
                    allowed_decision_methods,
                )
            )

        rospy.loginfo(
            '[AURA] decision-making method: %s',
            self._decision_method,
        )

        max_vel_x = rospy.get_param(
            '~dynamic_constraints/max_velocity/x', 1.0
        )
        max_vel_y = rospy.get_param(
            '~dynamic_constraints/max_velocity/y', 1.0
        )
        max_vel_z = rospy.get_param(
            '~dynamic_constraints/max_velocity/z', 1.0
        )
        max_acc_x = rospy.get_param(
            '~dynamic_constraints/max_acceleration/x', 1.0
        )
        max_acc_y = rospy.get_param(
            '~dynamic_constraints/max_acceleration/y', 1.0
        )
        max_acc_z = rospy.get_param(
            '~dynamic_constraints/max_acceleration/z', 1.0
        )

        self._max_velocity = (
            max_vel_x,
            max_vel_y,
            max_vel_z,
        )
        self._max_acceleration = (
            max_acc_x,
            max_acc_y,
            max_acc_z,
        )

        self._max_heading_rate = rospy.get_param(
            '~dynamic_constraints/max_heading_rate', 1.0
        )
        self._max_heading_acceleration = rospy.get_param(
            '~dynamic_constraints/max_heading_rate_acceleration', 1.0
        )

        # ---------------- setup collision avoidance ----------------
        self._safety_distance_mutual = rospy.get_param(
            '~trajectories/min_distance/mutual'
        )
        self._collision_avoidance = rospy.get_param(
            '~collision_avoidance/method', 'none'
        )

        # ------------------- setup path planner -------------------
        self._path_planner = {}

        self._path_planner['timeout'] = rospy.get_param(
            '~path_planner/timeout', 1.0
        )
        self._path_planner['path_planning_method'] = rospy.get_param(
            '~path_planner/method', 'rrt'
        )
        self._path_planner['safety_distance'] = rospy.get_param(
            '~trajectories/min_distance/obstacles'
        )
        self._path_planner['distance_estimation_method'] = rospy.get_param(
            '~tsp/distance_estimates', 'euclidean'
        )
        self._path_planner['straighten'] = rospy.get_param(
            '~path_planner/straighten_paths'
        )

        pp_method = self._path_planner['path_planning_method']
        de_method = self._path_planner['distance_estimation_method']

        if pp_method == 'astar' or de_method == 'astar':
            self._path_planner['astar/grid_resolution'] = rospy.get_param(
                '~path_planner/astar/grid_resolution'
            )

        if pp_method.startswith('rrt') or de_method.startswith('rrt'):
            self._path_planner['rrt/branch_size'] = rospy.get_param(
                '~path_planner/rrt/branch_size'
            )
            self._path_planner['rrt/sampling/method'] = rospy.get_param(
                '~path_planner/rrt/sampling/method'
            )
            self._path_planner['rrtstar/neighborhood'] = None

            if pp_method == 'rrtstar' or de_method == 'rrtstar':
                self._path_planner['rrtstar/neighborhood'] = rospy.get_param(
                    '~path_planner/rrt/star/neighborhood',
                    None,
                )

            if self._path_planner['rrt/sampling/method'] == 'gaussian':
                self._path_planner[
                    'rrt/sampling/gaussian/stddev_inflation'
                ] = rospy.get_param(
                    '~path_planner/rrt/sampling/gaussian/stddev_inflation'
                )

        # ----------------- initiate ROS publishers ----------------
        self.publisher_trajectory_1 = rospy.Publisher(
            "~trajectory_1_out",
            TrajectoryReference,
            queue_size=1,
            latch=True,
        )
        self.publisher_trajectory_2 = rospy.Publisher(
            "~trajectory_2_out",
            TrajectoryReference,
            queue_size=1,
            latch=True,
        )
        self.problem_publisher = rospy.Publisher(
            "~problem_out",
            InspectionProblem,
            queue_size=1,
            latch=True,
        )

        rate = rospy.Rate(1.0)
        rate.sleep()

        # --------------------- publish problem --------------------
        self.problem_publisher.publish(problem)

        # -------------------- plan trajectories -------------------
        trajectories, plotter = self.planTrajectories(problem)

        # -------------- convert to ROS trajectories --------------
        ros_trajectory_1 = trajectoryToRosMsg(
            trajectories[0].getPoses(),
            self._global_frame,
        )
        ros_trajectory_2 = trajectoryToRosMsg(
            trajectories[1].getPoses(),
            self._global_frame,
        )

        # ------------------ publish trajectories ------------------
        self.publisher_trajectory_1.publish(ros_trajectory_1)
        self.publisher_trajectory_2.publish(ros_trajectory_2)

        plotter.show(legend=True)

        rospy.loginfo(
            'Trajectories published, staying on, the publishers are latched.'
        )
        rospy.spin()

    # ------------------------------------------------------------------
    # Lab 3 helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _color_for_inspection_point(problem, inspection_point):
        """
        Convert the original inspectability rule to the Lab-3 color names.

        Robot index 0 is RED, robot index 1 is BLUE.
        Points inspectable by more than one robot are SHARED/PURPLE.
        """
        if len(inspection_point.inspectability) != 1:
            return 'shared'

        robot_id = inspection_point.inspectability[0]

        if robot_id == problem.robot_ids[0]:
            return 'red'

        if robot_id == problem.robot_ids[1]:
            return 'blue'

        raise ValueError(
            'Inspection point {} references unknown robot id {}'.format(
                inspection_point.idx,
                robot_id,
            )
        )

    @staticmethod
    def _target_position(viewpoint):
        return [
            float(viewpoint.pose.point.x),
            float(viewpoint.pose.point.y),
            float(viewpoint.pose.point.z),
        ]

    def _make_agent_initial_state(
        self,
        problem,
        target_viewpoints,
        target_colors,
    ):
        """
        Build the common StudentAgent state from the inspection problem.

        Ground-truth/estimated localization is intentionally not coupled into
        this first Lab-3 version: the high-level policy reasons on the planning
        state while Lab 2 remains an independent estimator experiment.
        """
        return {
            'uavs': {
                'red': {
                    'position': [
                        float(problem.start_poses[0].position.x),
                        float(problem.start_poses[0].position.y),
                        float(problem.start_poses[0].position.z),
                    ],
                    'current_target': None,
                },
                'blue': {
                    'position': [
                        float(problem.start_poses[1].position.x),
                        float(problem.start_poses[1].position.y),
                        float(problem.start_poses[1].position.z),
                    ],
                    'current_target': None,
                },
            },
            'targets': [
                {
                    'id': int(target_id),
                    'position': self._target_position(viewpoint),
                    'color': target_colors[target_id],
                    'visited': False,
                }
                for target_id, viewpoint in sorted(
                    target_viewpoints.items(),
                    key=lambda item: int(item[0]),
                )
            ],
            'done': False,
        }

    @staticmethod
    def _copy_agent_state(state):
        """Small explicit copy; avoids adding a deepcopy dependency."""
        return {
            'uavs': {
                name: {
                    'position': list(values['position']),
                    'current_target': values.get('current_target', None),
                }
                for name, values in state['uavs'].items()
            },
            'targets': [
                {
                    'id': int(target['id']),
                    'position': list(target['position']),
                    'color': target['color'],
                    'visited': bool(target.get('visited', False)),
                }
                for target in state['targets']
            ],
            'done': bool(state.get('done', False)),
        }

    def _q_learning_transition(self, state, action):
        """
        Apply one abstract Lab-3 joint action and return:
            next_state, reward, done

        Reward encourages inspection while minimizing the slowest UAV leg:
            + inspection_reward for every newly visited target
            - max(RED leg distance, BLUE leg distance)
            + completion_bonus when all targets are finished

        The max-leg term approximates two UAVs moving in parallel, so the
        learned policy targets mission makespan rather than only total distance.
        """
        next_state = self._copy_agent_state(state)

        target_lookup = {
            int(target['id']): target
            for target in next_state['targets']
        }

        leg_distances = []
        newly_visited = 0

        for uav_name in ('red', 'blue'):
            target_id = action.get(uav_name, None)

            if target_id is None:
                leg_distances.append(0.0)
                continue

            target_id = int(target_id)
            target = target_lookup[target_id]

            old_position = np.asarray(
                next_state['uavs'][uav_name]['position'],
                dtype=float,
            )
            new_position = np.asarray(
                target['position'],
                dtype=float,
            )

            leg_distances.append(
                float(np.linalg.norm(new_position - old_position))
            )

            if not target['visited']:
                target['visited'] = True
                newly_visited += 1

            next_state['uavs'][uav_name]['position'] = list(
                target['position']
            )
            next_state['uavs'][uav_name]['current_target'] = target_id

        remaining = [
            target
            for target in next_state['targets']
            if not target['visited']
        ]
        done = len(remaining) == 0
        next_state['done'] = done

        step_cost = max(leg_distances) if leg_distances else 0.0

        reward = (
            self._rl_inspection_reward * newly_visited
            - step_cost
        )

        if done:
            reward += self._rl_completion_bonus

        return next_state, float(reward), done

    def _train_q_learning(self, agent, initial_state):
        """
        Fast headless training on the abstract target-selection problem.

        No RRT is run during training.  RRT is used only once afterward for
        the final learned visit order, keeping the lab fast enough to use.
        """
        episodes = max(1, int(self._rl_training_episodes))

        rospy.loginfo(
            '[AURA LAB 3] training Q-learning for %d episodes...',
            episodes,
        )

        original_epsilon = agent.epsilon

        for episode in range(episodes):

            state = self._copy_agent_state(initial_state)

            # Linear epsilon decay: keep some exploration until the end.
            progress = float(episode) / max(1, episodes - 1)
            agent.epsilon = max(
                0.02,
                original_epsilon * (1.0 - 0.90 * progress),
            )

            max_steps = max(1, len(state['targets']) + 2)

            for _ in range(max_steps):

                if state.get('done', False):
                    break

                action = agent.select_action(state)

                next_state, reward, done = self._q_learning_transition(
                    state,
                    action,
                )

                agent.update(
                    state,
                    action,
                    reward,
                    next_state,
                    done,
                )

                state = next_state

                if done:
                    break

        # Evaluation should be greedy/deterministic with respect to Q.
        agent.epsilon = 0.0

        rospy.loginfo(
            '[AURA LAB 3] Q-learning training complete | Q entries: %d',
            len(agent.q),
        )

    def _agent_visit_orders(self, agent, initial_state):
        """
        Roll out a high-level policy in the abstract target-selection state.

        Returns:
            {
                'red':  [target_id, ...],
                'blue': [target_id, ...],
            }

        Random and greedy are working baselines. Q-learning/DQN call the
        student implementations from student_agent.py.
        """
        if agent.method == 'q_learning':
            self._train_q_learning(
                agent,
                initial_state,
            )

        state = self._copy_agent_state(initial_state)
        orders = {
            'red': [],
            'blue': [],
        }

        target_lookup = {
            int(target['id']): target
            for target in state['targets']
        }

        max_steps = max(1, len(state['targets']) + 2)

        for step in range(max_steps):

            remaining = [
                target
                for target in state['targets']
                if not target['visited']
            ]

            if not remaining:
                state['done'] = True
                break

            action = agent.select_action(state)

            rospy.loginfo(
                '[AURA LAB 3] decision %d | RED -> %s | BLUE -> %s',
                step,
                str(action.get('red', None)),
                str(action.get('blue', None)),
            )

            selected_any = False

            for uav_name in ('red', 'blue'):
                target_id = action.get(uav_name, None)

                if target_id is None:
                    continue

                target_id = int(target_id)

                if target_id not in target_lookup:
                    raise ValueError(
                        'Agent selected unknown target {}'.format(target_id)
                    )

                target = target_lookup[target_id]

                if target['visited']:
                    raise ValueError(
                        'Agent selected already visited target {}'.format(
                            target_id
                        )
                    )

                legal = agent.available_targets(state, uav_name)
                if target_id not in legal:
                    raise ValueError(
                        'Illegal Lab-3 action: {} selected target {}. '
                        'Legal targets are {}'.format(
                            uav_name,
                            target_id,
                            legal,
                        )
                    )

                orders[uav_name].append(target_id)
                target['visited'] = True
                state['uavs'][uav_name]['position'] = list(
                    target['position']
                )
                state['uavs'][uav_name]['current_target'] = target_id
                selected_any = True

            if not selected_any:
                raise RuntimeError(
                    'StudentAgent returned no target while targets remain.'
                )

        remaining = [
            target['id']
            for target in state['targets']
            if not target['visited']
        ]

        if remaining:
            raise RuntimeError(
                'Lab-3 policy did not finish all targets: {}'.format(
                    remaining
                )
            )

        return orders

    def _plan_fixed_order(
        self,
        problem,
        owned_viewpoints,
        ordered_target_ids,
        tsp_solver,
    ):
        """
        Connect an already chosen target order with the Lab-1 local planner.

        This deliberately bypasses TSP sequencing, but still uses exactly the
        configured A*/RRT/RRT* path planner for every consecutive leg.

        The route returns to the UAV start pose, matching the original TSP
        tour semantics and the evaluator's final-point expectation.
        """
        start_viewpoint = owned_viewpoints[0]

        target_by_id = {
            int(viewpoint.idx): viewpoint
            for viewpoint in owned_viewpoints[1:]
        }

        ordered_viewpoints = [start_viewpoint]

        for target_id in ordered_target_ids:
            target_id = int(target_id)

            if target_id not in target_by_id:
                raise ValueError(
                    'Target {} is not owned by this UAV.'.format(
                        target_id
                    )
                )

            ordered_viewpoints.append(target_by_id[target_id])

        # Always return to the start, as the original TSP tour does.
        ordered_viewpoints.append(start_viewpoint)

        if len(ordered_viewpoints) == 2:
            return [
                start_viewpoint.pose,
                start_viewpoint.pose,
            ]

        # Setup KD tree / bounds / A* grid once for this robot.
        tsp_solver.setup(
            problem,
            self._path_planner,
            owned_viewpoints,
        )

        path = []

        for idx in range(len(ordered_viewpoints) - 1):

            p_from = ordered_viewpoints[idx].pose
            p_to = ordered_viewpoints[idx + 1].pose

            leg, _ = tsp_solver.compute_path(
                p_from,
                p_to,
                self._path_planner,
                self._path_planner['path_planning_method'],
            )

            # Mirror TSPSolver3D.compute_tsp_tour(): discard each leg's final
            # sample so the next leg re-inserts that viewpoint with its
            # required inspection heading.
            path.extend(leg[:-1])

        path.append(ordered_viewpoints[-1].pose)

        return path

    def planTrajectories(self, problem):

        # --------------- create visualization object --------------
        plotter = ProblemPlotter(self._plot)
        plotter.addProblem(problem)

        # ----- initialize objects for TSP and trajectory utils ----
        tsp_solver = TSPSolver3D()
        trajectory_utils = TrajectoryUtils(
            self._max_velocity,
            self._max_acceleration,
            self._trajectory_dt,
        )

        # AURA student-facing logic.
        student = StudentPlanner(
            dt=self._trajectory_dt,
            safety_margin=0.15,
            max_start_delay=60.0,
        )

        # ==========================================================
        # 1) BUILD TARGETS + ASSIGN OWNERSHIP
        # ==========================================================
        print('[ASSIGNING VIEWPOINTS TO UAVs]')

        # One start viewpoint per robot.
        viewpoints = []

        for r in range(problem.number_of_robots):
            start_vp = Viewpoint(
                0,
                Pose(
                    problem.start_poses[r].position.x,
                    problem.start_poses[r].position.y,
                    problem.start_poses[r].position.z,
                    problem.start_poses[r].heading,
                ),
            )
            viewpoints.append([start_vp])

        # Build each inspection viewpoint exactly once.
        target_viewpoints = {}
        target_colors = {}
        shared_viewpoints = []

        for ip in problem.inspection_points:

            viewpoint = inspectionPointToViewPoint(
                ip,
                self._viewpoints_distance,
            )

            target_id = int(ip.idx)
            target_viewpoints[target_id] = viewpoint
            target_colors[target_id] = self._color_for_inspection_point(
                problem,
                ip,
            )

            if target_colors[target_id] == 'shared':
                shared_viewpoints.append(viewpoint)

        agent = None
        agent_state = None
        visit_orders = None

        if self._decision_method == 'tsp':

            # LAB 1 behavior: fixed-color points go to the required UAV,
            # StudentPlanner distributes purple/shared points.
            for target_id, viewpoint in target_viewpoints.items():

                color = target_colors[target_id]

                if color == 'red':
                    viewpoints[0].append(viewpoint)

                elif color == 'blue':
                    viewpoints[1].append(viewpoint)

            clusters = student.assign_shared_viewpoints(
                problem,
                shared_viewpoints,
            )

            for r in range(problem.number_of_robots):
                viewpoints[r].extend(clusters[r])

        else:

            # LAB 3 behavior: StudentAgent owns BOTH the shared-point
            # allocation and the high-level visit order.
            agent_state = self._make_agent_initial_state(
                problem,
                target_viewpoints,
                target_colors,
            )

            agent = StudentAgent(
                method=self._decision_method,
                shared_assignment=self._shared_assignment,
                alpha=self._rl_alpha,
                gamma=self._rl_gamma,
                epsilon=self._rl_epsilon,
                random_seed=self._rl_seed,
            )

            agent.reset(agent_state)
            ownership = agent.get_assignment()

            rospy.loginfo(
                '[AURA LAB 3] target ownership: %s',
                str(ownership),
            )

            for target_id, viewpoint in target_viewpoints.items():

                owner = ownership[int(target_id)]

                if owner == 'red':
                    viewpoints[0].append(viewpoint)

                elif owner == 'blue':
                    viewpoints[1].append(viewpoint)

                else:
                    raise ValueError(
                        'Unknown owner {} for target {}'.format(
                            owner,
                            target_id,
                        )
                    )

            visit_orders = self._agent_visit_orders(
                agent,
                agent_state,
            )

            rospy.loginfo(
                '[AURA LAB 3] RED visit order: %s',
                str(visit_orders['red']),
            )
            rospy.loginfo(
                '[AURA LAB 3] BLUE visit order: %s',
                str(visit_orders['blue']),
            )

        # Print ownership / viewpoint sets.
        for i in range(len(viewpoints)):
            print('viewpoints for UAV:', problem.robot_ids[i])
            for vp in viewpoints[i]:
                print('   [{:d}]:'.format(vp.idx), vp.pose)

        # Add VPs to offline visualization.
        plotter.addViewPoints(
            viewpoints,
            self._viewpoints_distance,
            self._viewpoints_distance,
        )

        # Check whether VPs themselves collide with environment.
        for i in range(len(viewpoints)):
            for vp in viewpoints[i]:
                point = vp.pose.point
                if pointCollidesWithObstacles(
                    point,
                    [
                        Point(o.x, o.y, o.z)
                        for o in problem.obstacle_points
                    ],
                    self._path_planner['safety_distance'],
                ):
                    rospy.logwarn(
                        'VP at %s collides with obstacles.',
                        point,
                    )

        # ==========================================================
        # 2) HIGH-LEVEL ORDER + LOCAL PATH PLANNING
        # ==========================================================

        waypoints = []

        for i in range(problem.number_of_robots):

            if self._decision_method == 'tsp':

                print('[PLANNING TSP TOUR] UAV {}'.format(
                    problem.robot_ids[i]
                ))

                # Original Lab-1 sequencing.
                robot_waypoints = tsp_solver.plan_tour(
                    problem,
                    viewpoints[i],
                    self._path_planner,
                )

            else:

                uav_name = 'red' if i == 0 else 'blue'

                print(
                    '[AURA LAB 3] planning {} policy order for UAV {}'.format(
                        self._decision_method,
                        problem.robot_ids[i],
                    )
                )

                # Lab-3 sequencing: StudentAgent has already chosen the
                # inspection order. The existing local planner now connects
                # only those selected consecutive edges.
                robot_waypoints = self._plan_fixed_order(
                    problem,
                    viewpoints[i],
                    visit_orders[uav_name],
                    tsp_solver,
                )

            # StudentPlanner heading interpolation remains common to all labs.
            robot_waypoints = student.interpolate_headings(
                robot_waypoints
            )

            waypoints.append(robot_waypoints)

            plotter.addWaypoints(
                robot_waypoints,
                color=COLORS[i],
                lw=1.2,
                label=(
                    'traj (id: '
                    + str(problem.robot_ids[i])
                    + ')'
                ),
            )

        # ==========================================================
        # 3) SAMPLE GEOMETRIC PATHS INTO TIME TRAJECTORIES
        # ==========================================================
        trajectories = []

        constraints_velocity = [
            self._max_velocity[0],
            self._max_velocity[1],
            self._max_velocity[2],
            self._max_heading_rate,
        ]

        constraints_acceleration = [
            self._max_acceleration[0],
            self._max_acceleration[1],
            self._max_acceleration[2],
            self._max_heading_acceleration,
        ]

        for r in range(problem.number_of_robots):

            print(
                "[PRE COLLISION AVOIDANCE] for robot with ID: {:d}".format(
                    problem.robot_ids[r]
                )
            )

            print('[GENERATING TRAJECTORY]')
            trajectory = Trajectory(
                self._trajectory_dt,
                waypoints[r],
            )

            print('[SAMPLING TRAJECTORY]')
            trajectory = trajectory_utils.sampleTrajectoryThroughWaypoints(
                trajectory,
                with_stops=self._sample_with_stops,
                smooth_path=True,
                smoothing_la_dist=self._smoothing_distance,
                smoothing_sampling_step=self._smoothing_sampling_step,
                velocity_limits=constraints_velocity,
                acceleration_limits=constraints_acceleration,
            )

            if trajectory is None:
                rospy.logerr(
                    'Unable to sample trajectory through waypoints. '
                    'Read the log output to find out why.'
                )
                rospy.signal_shutdown(
                    'Unable to sample trajectory through waypoints. '
                    'Read the log output to find out why.'
                )
                exit(-3)

            trajectories.append(trajectory)

        # ==========================================================
        # 4) MULTI-UAV TEMPORAL DECONFLICTION
        # ==========================================================
        print('[AURA COLLISION AVOIDANCE]')

        # AURA hook #3:
        # Delay the shorter trajectory until sampled UAV-UAV collisions
        # disappear.  We deliberately use this student-facing method rather
        # than the original config-selected backend method for this test.
        trajectories, delayed_robots, delays = (
            student.resolve_start_delay(
                trajectory_utils,
                trajectories,
                self._safety_distance_mutual,
            )
        )

        # ------ Add trajectories to offline visualization -----
        for i in range(problem.number_of_robots):
            plotter.addTrajectoryPoses(
                trajectories[i].getPoses(),
                color=COLORS[i],
                label=(
                    'traj. samples (id: '
                    + str(problem.robot_ids[i])
                    + ')'
                ),
            )

        # ---------------- Print trajectory infos ----------------
        print('###############################')
        print('##### Output trajectories #####')
        print('###############################')

        traj_t_max_idx = np.argmax(
            [t.getTime() for t in trajectories]
        )
        traj_d_max_idx = np.argmax(
            [t.getLength() for t in trajectories]
        )

        for r in range(problem.number_of_robots):
            print('UAV ID: {:d}'.format(problem.robot_ids[r]))
            print(
                '   Number of VPs:   {:d}'.format(
                    len(viewpoints[r]) - 1
                )
            )

            postfix = (
                ' (max)'
                if r == traj_t_max_idx
                else ''
            )

            if r in delayed_robots:
                idx = delayed_robots.index(r)
                postfix += (
                    ' (incl. {:.2f} s delay)'.format(
                        delays[idx]
                    )
                )

            print(
                '   Trajectory time: {:0.2f} s{:s}'.format(
                    trajectories[r].getTime(),
                    postfix,
                )
            )
            print(
                '   Trajectory len:  {:0.2f} m{:s}'.format(
                    trajectories[r].getLength(),
                    (
                        ' (max)'
                        if r == traj_d_max_idx
                        else ''
                    ),
                )
            )

        print('###############################')

        return trajectories, plotter


if __name__ == '__main__':
    try:
        mrim_planner = MrimPlanner()
    except rospy.ROSInterruptException:
        pass
