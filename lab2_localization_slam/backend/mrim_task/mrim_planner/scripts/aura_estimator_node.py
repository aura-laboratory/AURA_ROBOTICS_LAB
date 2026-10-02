#!/usr/bin/env python3
"""
AURA Lab 2 - Estimation Infrastructure Node
===========================================

This node is instructor/infrastructure code. Students should NOT edit it.

Purpose
-------
It connects the running UAV simulator to the common StudentEstimator API.

For the first Lab 2 scenario it:
  1. subscribes to simulator ground-truth odometry,
  2. hides ground truth from the student estimator,
  3. generates controlled noisy incremental odometry,
  4. generates controlled GPS-like position fixes,
  5. runs a dead-reckoning baseline and the selected student estimator,
  6. publishes paths/poses for RViz,
  7. reports running position RMSE.

For EKF-SLAM this version also generates controlled synthetic landmark
observations (known IDs, noisy range/bearing/elevation). Ground-truth landmark
positions stay inside this infrastructure node and are not passed to the
student estimator.

Expected runtime layout
-----------------------
Place this file and student_estimator.py in the same ROS scripts directory:

    mrim_task/mrim_planner/scripts/
        aura_estimator_node.py
        student_estimator.py

Default truth topic:
    /multirotor_simulator/uav1/odom

The student never receives the ground-truth message directly.
"""

import math
import rospy
import numpy as np

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

from student_estimator import StudentEstimator, wrap_angle


def quaternion_to_yaw(q):
    """Quaternion -> yaw, without depending on tf.transformations."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Return (x, y, z, w) quaternion for a yaw-only orientation."""
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def world_delta_to_body(yaw, dx_world, dy_world, dz_world):
    """Rotate a world-frame translation increment into the previous body frame."""
    c = math.cos(yaw)
    s = math.sin(yaw)

    dx_body = c * dx_world + s * dy_world
    dy_body = -s * dx_world + c * dy_world

    return dx_body, dy_body, dz_world


class AuraEstimatorNode:

    def __init__(self):
        rospy.init_node("aura_lab2_estimator")

        # ----------------------------- parameters -----------------------------
        self.truth_topic = rospy.get_param(
            "~truth_topic",
            "/multirotor_simulator/uav1/odom",
        )

        self.student_method = rospy.get_param(
            "~method",
            "dead_reckoning",
        )

        self.update_rate = float(
            rospy.get_param("~update_rate", 10.0)
        )

        self.gps_period = float(
            rospy.get_param("~gps_period", 1.0)
        )

        self.gps_sigma_xy = float(
            rospy.get_param("~gps_sigma_xy", 0.45)
        )
        self.gps_sigma_z = float(
            rospy.get_param("~gps_sigma_z", 0.30)
        )

        # EKF-SLAM synthetic landmark sensor. Data association is known by ID.
        self.landmark_period = float(
            rospy.get_param("~landmark_period", 0.50)
        )
        self.landmark_max_range = float(
            rospy.get_param("~landmark_max_range", 28.0)
        )
        self.landmark_sigma_range = float(
            rospy.get_param("~landmark_sigma_range", 0.15)
        )
        self.landmark_sigma_bearing = float(
            rospy.get_param("~landmark_sigma_bearing", 0.02)
        )
        self.landmark_sigma_elevation = float(
            rospy.get_param("~landmark_sigma_elevation", 0.02)
        )
        self.slam_use_gps = bool(
            rospy.get_param("~slam_use_gps", False)
        )

        # Incremental odometry noise.
        self.odom_sigma_xy = float(
            rospy.get_param("~odom_sigma_xy", 0.008)
        )
        self.odom_sigma_z = float(
            rospy.get_param("~odom_sigma_z", 0.004)
        )
        self.odom_sigma_yaw = float(
            rospy.get_param("~odom_sigma_yaw", 0.002)
        )

        # Small systematic error makes dead reckoning visibly drift.
        self.odom_scale_xy = float(
            rospy.get_param("~odom_scale_xy", 1.012)
        )
        self.odom_scale_z = float(
            rospy.get_param("~odom_scale_z", 1.005)
        )
        self.yaw_bias_rate = float(
            rospy.get_param("~yaw_bias_rate", 0.002)
        )

        # Optional GPS-denied interval, measured from first received sample.
        # Set duration=0 to disable.
        self.gps_dropout_start = float(
            rospy.get_param("~gps_dropout_start", 0.0)
        )
        self.gps_dropout_duration = float(
            rospy.get_param("~gps_dropout_duration", 0.0)
        )

        self.max_path_points = int(
            rospy.get_param("~max_path_points", 2500)
        )

        random_seed = int(
            rospy.get_param("~random_seed", 13)
        )
        self.rng = np.random.default_rng(random_seed)

        # ----------------------------- estimators -----------------------------
        self.student = StudentEstimator(
            method=self.student_method,
            random_seed=random_seed,
        )

        self.baseline = StudentEstimator(
            method="dead_reckoning",
            random_seed=random_seed,
        )

        # ----------------------------- publishers -----------------------------
        self.pub_truth_path = rospy.Publisher(
            "~ground_truth_path",
            Path,
            queue_size=1,
            latch=True,
        )
        self.pub_odom_path = rospy.Publisher(
            "~dead_reckoning_path",
            Path,
            queue_size=1,
            latch=True,
        )
        self.pub_student_path = rospy.Publisher(
            "~student_path",
            Path,
            queue_size=1,
            latch=True,
        )

        self.pub_baseline_pose = rospy.Publisher(
            "~dead_reckoning_pose",
            Odometry,
            queue_size=1,
        )
        self.pub_student_pose = rospy.Publisher(
            "~student_pose",
            Odometry,
            queue_size=1,
        )
        self.pub_noisy_gps = rospy.Publisher(
            "~noisy_gps",
            PointStamped,
            queue_size=1,
        )
        self.pub_slam_true_landmarks = rospy.Publisher(
            "~slam_true_landmarks",
            MarkerArray,
            queue_size=1,
            latch=True,
        )
        self.pub_slam_estimated_landmarks = rospy.Publisher(
            "~slam_estimated_landmarks",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        self.truth_path = Path()
        self.odom_path = Path()
        self.student_path = Path()

        # ------------------------------- state --------------------------------
        self.initialized = False
        self.frame_id = "world_origin"

        self.start_time = None
        self.last_processed_time = None
        self.last_gps_time = None
        self.last_landmark_time = None

        # Hidden simulator landmark map used only to synthesize measurements.
        self.true_landmarks = {}

        self.prev_truth = None

        self.student_sq_errors = []
        self.baseline_sq_errors = []

        self.last_report_time = None

        # Subscribe last so callbacks cannot happen before setup is complete.
        self.sub_truth = rospy.Subscriber(
            self.truth_topic,
            Odometry,
            self.truth_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "[AURA LAB 2] estimator node started | method=%s | truth=%s",
            self.student_method,
            self.truth_topic,
        )

        rospy.loginfo(
            "[AURA LAB 2] controlled sensors: odom %.1f Hz, GPS %.2f Hz",
            self.update_rate,
            1.0 / self.gps_period if self.gps_period > 0.0 else 0.0,
        )

        if self.student_method == "ekf_slam":
            rospy.loginfo(
                "[AURA LAB 2 SLAM] synthetic landmarks enabled | rate=%.2f Hz | max_range=%.1f m | GPS=%s",
                1.0 / self.landmark_period if self.landmark_period > 0.0 else 0.0,
                self.landmark_max_range,
                "ON" if self.slam_use_gps else "OFF",
            )

    # ----------------------------------------------------------------------
    # Truth callback
    # ----------------------------------------------------------------------

    def truth_callback(self, msg):
        stamp = self._message_time(msg)

        truth = self._odom_to_state(msg)

        if not self.initialized:
            self._initialize(msg, stamp, truth)
            return

        min_dt = 1.0 / max(self.update_rate, 1e-6)
        if stamp - self.last_processed_time < min_dt:
            return

        dt = stamp - self.last_processed_time
        if dt <= 0.0:
            return

        motion = self._make_noisy_motion(
            previous=self.prev_truth,
            current=truth,
            dt=dt,
        )

        measurements = []

        # For SLAM, GPS is disabled by default so the correction comes from
        # re-observed landmarks. It can be enabled with ~slam_use_gps:=true.
        allow_gps = (
            self.student_method != "ekf_slam"
            or self.slam_use_gps
        )

        if allow_gps and self._gps_due(stamp):
            if not self._gps_is_dropped(stamp):
                gps = self._make_noisy_gps(truth)
                measurements.append(gps)
                self._publish_noisy_gps(gps, msg.header.stamp)
            self.last_gps_time = stamp

        if (
            self.student_method == "ekf_slam"
            and self._landmarks_due(stamp)
        ):
            measurements.extend(
                self._make_landmark_measurements(truth)
            )
            self.last_landmark_time = stamp

        try:
            self.baseline.predict(motion)
            self.baseline.update(measurements)

            self.student.predict(motion)
            self.student.update(measurements)

        except NotImplementedError as exc:
            rospy.logerr(
                "[AURA LAB 2] Selected method '%s' still contains a student TODO: %s",
                self.student_method,
                str(exc),
            )
            rospy.signal_shutdown(
                "Selected estimator is not implemented yet."
            )
            return

        baseline_pose = self.baseline.get_pose()
        student_pose = self.student.get_pose()

        self._accumulate_errors(
            truth,
            baseline_pose,
            student_pose,
        )

        self._append_paths(
            msg.header.stamp,
            truth,
            baseline_pose,
            student_pose,
        )

        self._publish_pose(
            self.pub_baseline_pose,
            baseline_pose,
            msg.header.stamp,
            "aura_dead_reckoning",
        )

        self._publish_pose(
            self.pub_student_pose,
            student_pose,
            msg.header.stamp,
            "aura_student_estimate",
        )

        self._publish_paths(msg.header.stamp)

        if self.student_method == "ekf_slam":
            self._publish_slam_landmarks(msg.header.stamp)

        self._report_metrics(stamp)

        self.prev_truth = truth.copy()
        self.last_processed_time = stamp

    # ----------------------------------------------------------------------
    # Initialization
    # ----------------------------------------------------------------------

    def _initialize(self, msg, stamp, truth):
        self.frame_id = (
            msg.header.frame_id
            if msg.header.frame_id
            else "world_origin"
        )

        self.start_time = stamp
        self.last_processed_time = stamp
        self.last_gps_time = stamp
        self.last_landmark_time = stamp
        self.last_report_time = stamp

        self.prev_truth = truth.copy()

        if self.student_method == "ekf_slam":
            self.true_landmarks = self._build_slam_landmarks(truth)

        initial_covariance = np.diag(
            [0.05, 0.05, 0.05, 0.02]
        ) ** 2

        self.baseline.reset(
            truth,
            initial_covariance,
        )

        self.student.reset(
            truth,
            initial_covariance,
        )

        self.truth_path.header.frame_id = self.frame_id
        self.odom_path.header.frame_id = self.frame_id
        self.student_path.header.frame_id = self.frame_id

        self._append_paths(
            msg.header.stamp,
            truth,
            self.baseline.get_pose(),
            self.student.get_pose(),
        )

        self._publish_paths(msg.header.stamp)

        self.initialized = True

        if self.student_method == "ekf_slam":
            self._publish_slam_landmarks(msg.header.stamp)
            rospy.loginfo(
                "[AURA LAB 2 SLAM] hidden world contains %d landmarks",
                len(self.true_landmarks),
            )

        rospy.loginfo(
            "[AURA LAB 2] initialized at x=%.2f y=%.2f z=%.2f yaw=%.2f",
            truth[0],
            truth[1],
            truth[2],
            truth[3],
        )

    # ----------------------------------------------------------------------
    # Controlled sensor generation
    # ----------------------------------------------------------------------

    def _make_noisy_motion(self, previous, current, dt):
        dx_world = current[0] - previous[0]
        dy_world = current[1] - previous[1]
        dz_world = current[2] - previous[2]

        dx_body, dy_body, dz_body = world_delta_to_body(
            previous[3],
            dx_world,
            dy_world,
            dz_world,
        )

        dyaw = wrap_angle(
            current[3] - previous[3]
        )

        dx_body = (
            self.odom_scale_xy * dx_body
            + self.rng.normal(0.0, self.odom_sigma_xy)
        )
        dy_body = (
            self.odom_scale_xy * dy_body
            + self.rng.normal(0.0, self.odom_sigma_xy)
        )
        dz_body = (
            self.odom_scale_z * dz_body
            + self.rng.normal(0.0, self.odom_sigma_z)
        )
        dyaw = (
            dyaw
            + self.yaw_bias_rate * dt
            + self.rng.normal(0.0, self.odom_sigma_yaw)
        )

        return {
            "dt": float(dt),
            "dx": float(dx_body),
            "dy": float(dy_body),
            "dz": float(dz_body),
            "dyaw": float(dyaw),
        }

    def _make_noisy_gps(self, truth):
        sx2 = self.gps_sigma_xy ** 2
        sy2 = self.gps_sigma_xy ** 2
        sz2 = self.gps_sigma_z ** 2

        z = np.array(
            [
                truth[0] + self.rng.normal(0.0, self.gps_sigma_xy),
                truth[1] + self.rng.normal(0.0, self.gps_sigma_xy),
                truth[2] + self.rng.normal(0.0, self.gps_sigma_z),
            ],
            dtype=float,
        )

        return {
            "type": "gps",
            "position": z.tolist(),
            "covariance": [sx2, sy2, sz2],
        }


    def _build_slam_landmarks(self, initial_truth):
        """
        Create a fixed hidden landmark map around the mission start.

        These positions are infrastructure ground truth. The student estimator
        receives only noisy range/bearing/elevation plus landmark ID.
        """
        x0, y0, z0 = [
            float(initial_truth[0]),
            float(initial_truth[1]),
            float(initial_truth[2]),
        ]

        offsets = [
            ( 8.0,   0.0,  0.0),
            ( 0.0,   8.0,  1.0),
            (-8.0,   0.0, -1.0),
            ( 0.0,  -8.0,  0.5),
            (12.0,  12.0,  2.0),
            (-12.0, 12.0,  1.0),
            (12.0, -12.0, -1.0),
            (-12.0,-12.0,  2.0),
            (20.0,   0.0,  2.0),
            (-20.0,  0.0,  1.0),
            ( 0.0,  20.0, -1.0),
            ( 0.0, -20.0,  2.0),
        ]

        return {
            i: np.array(
                [x0 + dx, y0 + dy, z0 + dz],
                dtype=float,
            )
            for i, (dx, dy, dz) in enumerate(offsets)
        }

    def _landmarks_due(self, stamp):
        if self.landmark_period <= 0.0:
            return False
        return (
            stamp - self.last_landmark_time
            >= self.landmark_period
        )

    def _make_landmark_measurements(self, truth):
        measurements = []

        px, py, pz, yaw = [float(v) for v in truth]

        for landmark_id, position in self.true_landmarks.items():
            dx = float(position[0] - px)
            dy = float(position[1] - py)
            dz = float(position[2] - pz)

            horizontal = math.sqrt(dx*dx + dy*dy)
            distance = math.sqrt(
                dx*dx + dy*dy + dz*dz
            )

            if distance > self.landmark_max_range:
                continue

            bearing = wrap_angle(
                math.atan2(dy, dx) - yaw
            )
            elevation = math.atan2(
                dz,
                max(horizontal, 1e-9),
            )

            noisy_range = max(
                0.05,
                distance
                + self.rng.normal(
                    0.0,
                    self.landmark_sigma_range,
                ),
            )
            noisy_bearing = wrap_angle(
                bearing
                + self.rng.normal(
                    0.0,
                    self.landmark_sigma_bearing,
                )
            )
            noisy_elevation = (
                elevation
                + self.rng.normal(
                    0.0,
                    self.landmark_sigma_elevation,
                )
            )

            measurements.append(
                {
                    "type": "landmark",
                    "id": int(landmark_id),
                    "range": float(noisy_range),
                    "bearing": float(noisy_bearing),
                    "elevation": float(noisy_elevation),
                    "range_variance": float(
                        self.landmark_sigma_range ** 2
                    ),
                    "bearing_variance": float(
                        self.landmark_sigma_bearing ** 2
                    ),
                    "elevation_variance": float(
                        self.landmark_sigma_elevation ** 2
                    ),
                }
            )

        return measurements

    def _gps_due(self, stamp):
        if self.gps_period <= 0.0:
            return False

        return (
            stamp - self.last_gps_time
            >= self.gps_period
        )

    def _gps_is_dropped(self, stamp):
        if self.gps_dropout_duration <= 0.0:
            return False

        elapsed = stamp - self.start_time

        return (
            self.gps_dropout_start
            <= elapsed
            < self.gps_dropout_start + self.gps_dropout_duration
        )

    # ----------------------------------------------------------------------
    # Metrics
    # ----------------------------------------------------------------------

    def _accumulate_errors(self, truth, baseline_pose, student_pose):
        b = np.array(
            [
                baseline_pose["x"],
                baseline_pose["y"],
                baseline_pose["z"],
            ]
        )

        s = np.array(
            [
                student_pose["x"],
                student_pose["y"],
                student_pose["z"],
            ]
        )

        t = truth[0:3]

        self.baseline_sq_errors.append(
            float(np.sum((b - t) ** 2))
        )
        self.student_sq_errors.append(
            float(np.sum((s - t) ** 2))
        )

    def _report_metrics(self, stamp):
        if stamp - self.last_report_time < 5.0:
            return

        baseline_rmse = math.sqrt(
            float(np.mean(self.baseline_sq_errors))
        )

        student_rmse = math.sqrt(
            float(np.mean(self.student_sq_errors))
        )

        improvement = 0.0
        if baseline_rmse > 1e-9:
            improvement = (
                100.0
                * (baseline_rmse - student_rmse)
                / baseline_rmse
            )

        rospy.loginfo(
            "[AURA LAB 2] RMSE | dead-reckoning=%.3f m | student(%s)=%.3f m | improvement=%+.1f%%",
            baseline_rmse,
            self.student_method,
            student_rmse,
            improvement,
        )

        if self.student_method == "ekf_slam":
            estimated = self.student.get_map()
            errors = []
            for landmark_id, estimate in estimated.items():
                if int(landmark_id) not in self.true_landmarks:
                    continue
                estimate = np.asarray(estimate, dtype=float)
                truth_lm = self.true_landmarks[int(landmark_id)]
                errors.append(
                    float(np.sum((estimate - truth_lm) ** 2))
                )

            if errors:
                map_rmse = math.sqrt(float(np.mean(errors)))
                rospy.loginfo(
                    "[AURA LAB 2 SLAM] mapped=%d/%d | landmark RMSE=%.3f m",
                    len(errors),
                    len(self.true_landmarks),
                    map_rmse,
                )

        self.last_report_time = stamp

    # ----------------------------------------------------------------------
    # ROS conversion / visualization
    # ----------------------------------------------------------------------

    def _odom_to_state(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        return np.array(
            [
                float(p.x),
                float(p.y),
                float(p.z),
                float(quaternion_to_yaw(q)),
            ],
            dtype=float,
        )

    def _message_time(self, msg):
        stamp = msg.header.stamp.to_sec()

        if stamp <= 0.0:
            stamp = rospy.get_time()

        return float(stamp)

    def _pose_stamped(self, pose, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.pose.position.x = float(pose["x"])
        msg.pose.position.y = float(pose["y"])
        msg.pose.position.z = float(pose["z"])

        qx, qy, qz, qw = yaw_to_quaternion(
            pose["yaw"]
        )

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def _state_pose_stamped(self, state, stamp):
        return self._pose_stamped(
            {
                "x": state[0],
                "y": state[1],
                "z": state[2],
                "yaw": state[3],
            },
            stamp,
        )

    def _append_paths(
        self,
        stamp,
        truth,
        baseline_pose,
        student_pose,
    ):
        self.truth_path.poses.append(
            self._state_pose_stamped(truth, stamp)
        )

        self.odom_path.poses.append(
            self._pose_stamped(baseline_pose, stamp)
        )

        self.student_path.poses.append(
            self._pose_stamped(student_pose, stamp)
        )

        for path in (
            self.truth_path,
            self.odom_path,
            self.student_path,
        ):
            if len(path.poses) > self.max_path_points:
                path.poses = path.poses[-self.max_path_points:]

    def _publish_paths(self, stamp):
        for path in (
            self.truth_path,
            self.odom_path,
            self.student_path,
        ):
            path.header.stamp = stamp
            path.header.frame_id = self.frame_id

        self.pub_truth_path.publish(
            self.truth_path
        )
        self.pub_odom_path.publish(
            self.odom_path
        )
        self.pub_student_path.publish(
            self.student_path
        )

    def _publish_pose(
        self,
        publisher,
        pose,
        stamp,
        child_frame_id,
    ):
        msg = Odometry()

        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = child_frame_id

        msg.pose.pose.position.x = float(
            pose["x"]
        )
        msg.pose.pose.position.y = float(
            pose["y"]
        )
        msg.pose.pose.position.z = float(
            pose["z"]
        )

        qx, qy, qz, qw = yaw_to_quaternion(
            pose["yaw"]
        )

        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        publisher.publish(msg)


    def _landmark_marker(self, namespace, marker_id, points, stamp, rgba):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = int(marker_id)
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.55
        marker.scale.y = 0.55
        marker.scale.z = 0.55
        marker.color.r = float(rgba[0])
        marker.color.g = float(rgba[1])
        marker.color.b = float(rgba[2])
        marker.color.a = float(rgba[3])

        from geometry_msgs.msg import Point
        for p in points:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = float(p[2])
            marker.points.append(point)

        return marker

    def _publish_slam_landmarks(self, stamp):
        true_points = [
            self.true_landmarks[k]
            for k in sorted(self.true_landmarks)
        ]

        estimated_map = self.student.get_map()
        estimated_points = [
            estimated_map[k]
            for k in sorted(estimated_map)
        ]

        true_array = MarkerArray()
        true_array.markers.append(
            self._landmark_marker(
                "aura_slam_true",
                0,
                true_points,
                stamp,
                (1.0, 1.0, 1.0, 0.85),
            )
        )

        estimated_array = MarkerArray()
        estimated_array.markers.append(
            self._landmark_marker(
                "aura_slam_estimated",
                0,
                estimated_points,
                stamp,
                (0.0, 1.0, 1.0, 0.95),
            )
        )

        self.pub_slam_true_landmarks.publish(true_array)
        self.pub_slam_estimated_landmarks.publish(estimated_array)

    def _publish_noisy_gps(self, gps, stamp):
        msg = PointStamped()

        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.point.x = float(gps["position"][0])
        msg.point.y = float(gps["position"][1])
        msg.point.z = float(gps["position"][2])

        self.pub_noisy_gps.publish(msg)


if __name__ == "__main__":
    try:
        node = AuraEstimatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

