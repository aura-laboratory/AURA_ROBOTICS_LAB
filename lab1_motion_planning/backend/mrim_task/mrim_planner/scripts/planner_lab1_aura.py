#!/usr/bin/env python3
"""
AURA Lab 1 planner integration.

Lab 1 combines the following stages of the multi-UAV planning pipeline:

  1) shared-viewpoint assignment,
  2) TSP visit ordering,
  3) local path planning,
  4) heading interpolation,
  5) trajectory generation and smoothing,
  6) temporal two-UAV collision avoidance.

Three local path-planning modes are available:

  * rrt     -> provided backend RRT
  * astar   -> provided backend A*
  * student -> StudentPlanner.plan_path()

Student work:
  * Question B -> implement the shared-viewpoint assignment strategy.
  * Question C -> implement the local path planner.

TSP ordering, heading interpolation, trajectory generation/smoothing
and temporal two-UAV collision avoidance are provided parts of the
pipeline and should not be modified for the Lab assignment.
"""

import rospy
import rospkg
import numpy as np

from utils import *
from solvers.tsp_solvers import *
from trajectory import Trajectory, TrajectoryUtils
from student_planner_lab1_aura import StudentPlanner


class MrimPlanner:

    ALLOWED_COLLISION_AVOIDANCE_METHODS = [
        'none',
        'delay_2nd_till_1st_UAV_finishes',
        'delay_till_no_collisions_occur',
    ]

    def __init__(self):

        rospy.init_node('mrim_planner', anonymous=True)

        # ----------------------------------------------------------
        # LOAD PROBLEM
        # ----------------------------------------------------------
        problem_filename = rospy.get_param('~problem/name')
        session_problem = rospy.get_param('~session_problem')

        problem_filepath = (
            rospkg.RosPack().get_path('mrim_resources')
            + "/problems/"
            + problem_filename
        )

        problem, log_msg = ProblemLoader().loadProblem(
            problem_filepath
        )

        if problem is None:
            rospy.logerr(log_msg)
            rospy.signal_shutdown(log_msg)
            exit(-1)

        # ----------------------------------------------------------
        # LOAD PARAMETERS
        # ----------------------------------------------------------
        self._viewpoints_distance = rospy.get_param(
            '~viewpoints/distance',
            3.0,
        )

        self._plot = rospy.get_param(
            '~problem/plot',
            False,
        )

        self._trajectory_dt = rospy.get_param(
            '~trajectories/dt',
            0.2,
        )

        self._smoothing_sampling_step = rospy.get_param(
            '~path_smoothing/sampling_step',
            0.1,
        )

        self._smoothing_distance = rospy.get_param(
            '~path_smoothing/lookahead_dist',
            0.3,
        )

        self._sample_with_stops = rospy.get_param(
            '~trajectory_sampling/with_stops',
            True,
        )

        self._global_frame = rospy.get_param(
            '~global_frame',
            "gps_origin",
        )

        self._tsp_clustering_method = rospy.get_param(
            '~tsp/clustering',
            'random',
        )

        # ----------------------------------------------------------
        # UAV DYNAMIC CONSTRAINTS
        # ----------------------------------------------------------
        max_vel_x = rospy.get_param(
            '~dynamic_constraints/max_velocity/x',
            1.0,
        )

        max_vel_y = rospy.get_param(
            '~dynamic_constraints/max_velocity/y',
            1.0,
        )

        max_vel_z = rospy.get_param(
            '~dynamic_constraints/max_velocity/z',
            1.0,
        )

        max_acc_x = rospy.get_param(
            '~dynamic_constraints/max_acceleration/x',
            1.0,
        )

        max_acc_y = rospy.get_param(
            '~dynamic_constraints/max_acceleration/y',
            1.0,
        )

        max_acc_z = rospy.get_param(
            '~dynamic_constraints/max_acceleration/z',
            1.0,
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
            '~dynamic_constraints/max_heading_rate',
            1.0,
        )

        self._max_heading_acceleration = rospy.get_param(
            '~dynamic_constraints/max_heading_rate_acceleration',
            1.0,
        )

        # ----------------------------------------------------------
        # COLLISION AVOIDANCE PARAMETERS
        # ----------------------------------------------------------
        self._safety_distance_mutual = rospy.get_param(
            '~trajectories/min_distance/mutual'
        )

        self._collision_avoidance = rospy.get_param(
            '~collision_avoidance/method',
            'none',
        )

        # ----------------------------------------------------------
        # PATH PLANNER PARAMETERS
        # ----------------------------------------------------------
        self._path_planner = {}

        self._path_planner['timeout'] = rospy.get_param(
            '~path_planner/timeout',
            1.0,
        )

        self._path_planner[
            'path_planning_method'
        ] = rospy.get_param(
            '~path_planner/method',
            'rrt',
        )

        self._path_planner[
            'safety_distance'
        ] = rospy.get_param(
            '~trajectories/min_distance/obstacles'
        )

        self._path_planner[
            'distance_estimation_method'
        ] = rospy.get_param(
            '~tsp/distance_estimates',
            'euclidean',
        )

        self._path_planner[
            'straighten'
        ] = rospy.get_param(
            '~path_planner/straighten_paths'
        )

        # Parameters used only by StudentPlanner.plan_path()
        # when running STUDENT mode.
        self._path_planner[
            'student/step_size'
        ] = rospy.get_param(
            '~path_planner/student/step_size',
            3.0,
        )

        self._path_planner[
            'student/goal_bias'
        ] = rospy.get_param(
            '~path_planner/student/goal_bias',
            0.20,
        )

        self._path_planner[
            'student/max_iterations'
        ] = rospy.get_param(
            '~path_planner/student/max_iterations',
            4000,
        )

        self._path_planner[
            'student/collision_check_step'
        ] = rospy.get_param(
            '~path_planner/student/collision_check_step',
            0.25,
        )

        pp_method = self._path_planner[
            'path_planning_method'
        ]

        de_method = self._path_planner[
            'distance_estimation_method'
        ]

        if pp_method not in (
            'rrt',
            'rrtstar',
            'astar',
            'student',
        ):
            raise ValueError(
                'AURA Lab 1 path_planner.method must be one of: '
                'rrt, rrtstar, astar, student. Got: {}'.format(
                    pp_method
                )
            )

        if (
            pp_method == 'astar'
            or de_method == 'astar'
        ):
            self._path_planner[
                'astar/grid_resolution'
            ] = rospy.get_param(
                '~path_planner/astar/grid_resolution'
            )

        if (
            pp_method.startswith('rrt')
            or de_method.startswith('rrt')
        ):
            self._path_planner[
                'rrt/branch_size'
            ] = rospy.get_param(
                '~path_planner/rrt/branch_size'
            )

            self._path_planner[
                'rrt/sampling/method'
            ] = rospy.get_param(
                '~path_planner/rrt/sampling/method'
            )

            self._path_planner[
                'rrtstar/neighborhood'
            ] = None

            if (
                pp_method == 'rrtstar'
                or de_method == 'rrtstar'
            ):
                self._path_planner[
                    'rrtstar/neighborhood'
                ] = rospy.get_param(
                    '~path_planner/rrt/star/neighborhood',
                    None,
                )

            if (
                self._path_planner[
                    'rrt/sampling/method'
                ]
                == 'gaussian'
            ):
                self._path_planner[
                    'rrt/sampling/gaussian/'
                    'stddev_inflation'
                ] = rospy.get_param(
                    '~path_planner/rrt/sampling/'
                    'gaussian/stddev_inflation'
                )

        # ----------------------------------------------------------
        # ROS PUBLISHERS
        # ----------------------------------------------------------
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

        # Publish inspection problem.
        self.problem_publisher.publish(problem)

        # Run complete Lab 1 planning pipeline.
        trajectories, plotter = self.planTrajectories(
            problem
        )

        # ----------------------------------------------------------
        # CONVERT TO ROS TRAJECTORIES
        # ----------------------------------------------------------
        ros_trajectory_1 = trajectoryToRosMsg(
            trajectories[0].getPoses(),
            self._global_frame,
        )

        ros_trajectory_2 = trajectoryToRosMsg(
            trajectories[1].getPoses(),
            self._global_frame,
        )

        # ----------------------------------------------------------
        # PUBLISH TRAJECTORIES
        # ----------------------------------------------------------
        self.publisher_trajectory_1.publish(
            ros_trajectory_1
        )

        self.publisher_trajectory_2.publish(
            ros_trajectory_2
        )

        plotter.show(legend=True)

        rospy.loginfo(
            'Trajectories published, staying on, '
            'the publishers are latched.'
        )

        rospy.spin()

    def planTrajectories(self, problem):

        # ----------------------------------------------------------
        # VISUALIZATION
        # ----------------------------------------------------------
        plotter = ProblemPlotter(
            self._plot
        )

        plotter.addProblem(problem)

        # ----------------------------------------------------------
        # PROVIDED TSP + TRAJECTORY UTILITIES
        # ----------------------------------------------------------
        tsp_solver = TSPSolver3D()

        trajectory_utils = TrajectoryUtils(
            self._max_velocity,
            self._max_acceleration,
            self._trajectory_dt,
        )

        # ----------------------------------------------------------
        # STUDENT-FACING LAB 1 LOGIC
        # ----------------------------------------------------------
        student = StudentPlanner(
            dt=self._trajectory_dt,
            safety_margin=0.15,
            max_start_delay=60.0,
        )

        # ==========================================================
        # 1) VIEWPOINT ASSIGNMENT
        #
        # QUESTION B:
        # shared/purple viewpoint assignment is performed through
        # StudentPlanner.assign_shared_viewpoints().
        # ==========================================================
        print(
            '[ASSIGNING VIEWPOINTS TO UAVs]'
        )

        viewpoints = []
        nonclustered_vps = []

        for r in range(
            problem.number_of_robots
        ):

            # ------------------------------------------------------
            # Add the UAV starting pose.
            # ------------------------------------------------------
            start_vp = Viewpoint(
                0,
                Pose(
                    problem.start_poses[
                        r
                    ].position.x,
                    problem.start_poses[
                        r
                    ].position.y,
                    problem.start_poses[
                        r
                    ].position.z,
                    problem.start_poses[
                        r
                    ].heading,
                ),
            )

            viewpoints.append(
                [start_vp]
            )

            robot_id = problem.robot_ids[r]

            for ip in problem.inspection_points:

                # Convert inspection point to viewpoint.
                viewpoint = (
                    inspectionPointToViewPoint(
                        ip,
                        self._viewpoints_distance,
                    )
                )

                # --------------------------------------------------
                # RED / BLUE viewpoint:
                #
                # uniquely inspectable inspection points are
                # automatically assigned to their required UAV.
                # --------------------------------------------------
                if (
                    len(ip.inspectability) == 1
                    and robot_id
                    in ip.inspectability
                ):
                    viewpoints[
                        r
                    ].append(
                        viewpoint
                    )

                # --------------------------------------------------
                # PURPLE / SHARED viewpoint:
                #
                # collect once and assign later using the strategy
                # implemented in Question B.
                # --------------------------------------------------
                elif (
                    len(ip.inspectability) != 1
                    and ip.idx not in [
                        vp.idx
                        for vp
                        in nonclustered_vps
                    ]
                ):
                    nonclustered_vps.append(
                        viewpoint
                    )

        # ==========================================================
        # QUESTION B:
        # SHARED / PURPLE VIEWPOINT ASSIGNMENT
        # ==========================================================
        clusters = (
            student.assign_shared_viewpoints(
                problem,
                nonclustered_vps,
            )
        )

        # ----------------------------------------------------------
        # Report-friendly output:
        #
        # show exactly which shared viewpoints were assigned to
        # each UAV.
        # ----------------------------------------------------------
        print(
            '[AURA LAB 1] '
            'SHARED VIEWPOINT ASSIGNMENT'
        )

        for r in range(
            problem.number_of_robots
        ):
            print(
                '   UAV ID {}: {} shared viewpoints -> {}'.format(
                    problem.robot_ids[r],
                    len(clusters[r]),
                    [
                        vp.idx
                        for vp
                        in clusters[r]
                    ],
                )
            )

        # Add shared viewpoints to the fixed RED/BLUE viewpoints.
        for r in range(
            problem.number_of_robots
        ):
            viewpoints[r].extend(
                clusters[r]
            )

        # ----------------------------------------------------------
        # Print the complete viewpoint set for each UAV:
        #
        # start + fixed RED/BLUE + shared viewpoints.
        # ----------------------------------------------------------
        for i in range(
            len(viewpoints)
        ):
            print(
                'viewpoints for UAV:',
                problem.robot_ids[i],
            )

            for vp in viewpoints[i]:
                print(
                    '   [{:d}]:'.format(
                        vp.idx
                    ),
                    vp.pose,
                )

        # Add viewpoints to offline visualization.
        plotter.addViewPoints(
            viewpoints,
            self._viewpoints_distance,
            self._viewpoints_distance,
        )

        # ----------------------------------------------------------
        # Check whether viewpoints themselves collide with obstacles.
        # ----------------------------------------------------------
        for i in range(
            len(viewpoints)
        ):
            for vp in viewpoints[i]:

                point = vp.pose.point

                if pointCollidesWithObstacles(
                    point,
                    [
                        Point(
                            o.x,
                            o.y,
                            o.z,
                        )
                        for o
                        in problem.obstacle_points
                    ],
                    self._path_planner[
                        'safety_distance'
                    ],
                ):
                    rospy.logwarn(
                        'VP at %s collides with obstacles.',
                        point,
                    )

        # ==========================================================
        # 2) PROVIDED TSP ORDERING + LOCAL PATH PLANNING
        #
        # The TSP solver determines the order in which each UAV
        # visits its assigned viewpoints.
        #
        # In STUDENT mode, Question C controls only the local path
        # between consecutive viewpoints.
        # ==========================================================
        print(
            '[PLANNING TSP TOUR]'
        )

        waypoints = []

        for i in range(
            problem.number_of_robots
        ):

            method = self._path_planner[
                'path_planning_method'
            ]

            if method == 'student':

                # --------------------------------------------------
                # PROVIDED TSP ORDERING
                #
                # Keep TSP ordering separate from the student's
                # local path planner.
                #
                # Euclidean costs determine the viewpoint visit
                # order in STUDENT mode.
                # --------------------------------------------------
                ordering_config = dict(
                    self._path_planner
                )

                ordering_config[
                    'path_planning_method'
                ] = 'euclidean'

                ordering_config[
                    'distance_estimation_method'
                ] = 'euclidean'

                ordered_poses = (
                    tsp_solver.plan_tour(
                        problem,
                        viewpoints[i],
                        ordering_config,
                    )
                )

                # --------------------------------------------------
                # QUESTION C:
                #
                # StudentPlanner.plan_path() is called for every
                # consecutive edge of the provided TSP ordering.
                # --------------------------------------------------
                robot_waypoints = []

                for edge_idx in range(
                    len(ordered_poses) - 1
                ):
                    segment, _ = (
                        student.plan_path(
                            ordered_poses[
                                edge_idx
                            ],
                            ordered_poses[
                                edge_idx + 1
                            ],
                            problem,
                            self._path_planner,
                        )
                    )

                    robot_waypoints.extend(
                        segment[:-1]
                    )

                if ordered_poses:
                    robot_waypoints.append(
                        ordered_poses[-1]
                    )

            else:

                # --------------------------------------------------
                # PROVIDED A* / RRT PATH-PLANNING MODES
                #
                # Used in Question A.
                # --------------------------------------------------
                robot_waypoints = (
                    tsp_solver.plan_tour(
                        problem,
                        viewpoints[i],
                        self._path_planner,
                    )
                )

            # ======================================================
            # PROVIDED: HEADING INTERPOLATION
            #
            # DO NOT MODIFY FOR THE LAB ASSIGNMENT.
            #
            # Intermediate local-planner samples may have no
            # heading. The provided method interpolates headings
            # while preserving the required inspection headings.
            # ======================================================
            robot_waypoints = (
                student.interpolate_headings(
                    robot_waypoints
                )
            )

            waypoints.append(
                robot_waypoints
            )

            plotter.addWaypoints(
                robot_waypoints,
                color=COLORS[i],
                lw=1.2,
                label=(
                    'traj (id: '
                    + str(
                        problem.robot_ids[i]
                    )
                    + ')'
                ),
            )

        # ==========================================================
        # 3) PROVIDED: TRAJECTORY GENERATION + SMOOTHING
        #
        # DO NOT MODIFY FOR THE LAB ASSIGNMENT.
        #
        # The geometric paths are converted into time-parameterized
        # trajectories. Sampling and smoothing are applied together
        # with the UAV velocity and acceleration constraints.
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

        for r in range(
            problem.number_of_robots
        ):

            print(
                '[PRE COLLISION AVOIDANCE] '
                'for robot with ID: {:d}'.format(
                    problem.robot_ids[r]
                )
            )

            print(
                '[GENERATING TRAJECTORY]'
            )

            trajectory = Trajectory(
                self._trajectory_dt,
                waypoints[r],
            )

            print(
                '[SAMPLING TRAJECTORY]'
            )

            trajectory = (
                trajectory_utils
                .sampleTrajectoryThroughWaypoints(
                    trajectory,
                    with_stops=(
                        self._sample_with_stops
                    ),
                    smooth_path=True,
                    smoothing_la_dist=(
                        self._smoothing_distance
                    ),
                    smoothing_sampling_step=(
                        self._smoothing_sampling_step
                    ),
                    velocity_limits=(
                        constraints_velocity
                    ),
                    acceleration_limits=(
                        constraints_acceleration
                    ),
                )
            )

            if trajectory is None:

                rospy.logerr(
                    'Unable to sample trajectory '
                    'through waypoints. '
                    'Read the log output to find out why.'
                )

                rospy.signal_shutdown(
                    'Unable to sample trajectory '
                    'through waypoints. '
                    'Read the log output to find out why.'
                )

                exit(-3)

            trajectories.append(
                trajectory
            )

        # ==========================================================
        # 4) PROVIDED: MULTI-UAV TEMPORAL COLLISION AVOIDANCE
        #
        # DO NOT MODIFY FOR THE LAB ASSIGNMENT.
        #
        # After both trajectories have been generated, the system
        # checks them for UAV-UAV conflicts. If required, the UAV
        # with the shorter trajectory is delayed at its start.
        # ==========================================================
        print(
            '[AURA COLLISION AVOIDANCE]'
        )

        trajectories, delayed_robots, delays = (
            student.resolve_start_delay(
                trajectory_utils,
                trajectories,
                self._safety_distance_mutual,
            )
        )

        # ----------------------------------------------------------
        # Add trajectories to offline visualization.
        # ----------------------------------------------------------
        for i in range(
            problem.number_of_robots
        ):
            plotter.addTrajectoryPoses(
                trajectories[i].getPoses(),
                color=COLORS[i],
                label=(
                    'traj. samples (id: '
                    + str(
                        problem.robot_ids[i]
                    )
                    + ')'
                ),
            )

        # ==========================================================
        # OUTPUT TRAJECTORY INFORMATION
        # ==========================================================
        print(
            '###############################'
        )

        print(
            '##### Output trajectories #####'
        )

        print(
            '###############################'
        )

        traj_t_max_idx = np.argmax(
            [
                t.getTime()
                for t in trajectories
            ]
        )

        traj_d_max_idx = np.argmax(
            [
                t.getLength()
                for t in trajectories
            ]
        )

        for r in range(
            problem.number_of_robots
        ):

            print(
                'UAV ID: {:d}'.format(
                    problem.robot_ids[r]
                )
            )

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

                idx = (
                    delayed_robots.index(r)
                )

                postfix += (
                    ' (incl. {:.2f} s delay)'.format(
                        delays[idx]
                    )
                )

            print(
                '   Trajectory time: '
                '{:0.2f} s{:s}'.format(
                    trajectories[
                        r
                    ].getTime(),
                    postfix,
                )
            )

            print(
                '   Trajectory len:  '
                '{:0.2f} m{:s}'.format(
                    trajectories[
                        r
                    ].getLength(),
                    (
                        ' (max)'
                        if r
                        == traj_d_max_idx
                        else ''
                    ),
                )
            )

        print(
            '###############################'
        )

        return trajectories, plotter


if __name__ == '__main__':
    try:
        mrim_planner = MrimPlanner()
    except rospy.ROSInterruptException:
        pass