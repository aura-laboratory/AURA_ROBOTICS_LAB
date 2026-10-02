#!/usr/bin/env python3
"""
AURA Lab 2 - Student Estimator with working EKF + EKF-SLAM reference
===================================================================

Common pose state:
    [x, y, z, yaw]

Methods:
    dead_reckoning
    gps_fusion
    ekf
    particle_filter   (student TODO)
    ekf_slam          (working reference)
    graph_slam        (advanced TODO)

EKF-SLAM measurements:
    {
        "type": "landmark",
        "id": 4,
        "range": 7.42,
        "bearing": -0.31,
        "elevation": 0.08,
        "range_variance": 0.0225,
        "bearing_variance": 0.0004,
        "elevation_variance": 0.0004,
    }

The EKF-SLAM augmented state is:
    [robot_x, robot_y, robot_z, robot_yaw,
     lm0_x, lm0_y, lm0_z,
     lm1_x, lm1_y, lm1_z, ...]
"""

import math
import numpy as np


def wrap_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def body_delta_to_world(yaw, dx, dy, dz):
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array([
        c * float(dx) - s * float(dy),
        s * float(dx) + c * float(dy),
        float(dz),
    ], dtype=float)


class StudentEstimator:
    SUPPORTED_METHODS = (
        "dead_reckoning",
        "gps_fusion",
        "ekf",
        "particle_filter",
        "ekf_slam",
        "graph_slam",
    )

    def __init__(
        self,
        method="dead_reckoning",
        known_landmarks=None,
        num_particles=500,
        random_seed=7,
    ):
        if method not in self.SUPPORTED_METHODS:
            raise ValueError("Unknown estimator method: {}".format(method))

        self.method = method
        self.num_particles = int(num_particles)
        self.rng = np.random.default_rng(int(random_seed))

        self.gps_fusion_alpha_xy = 0.20
        self.gps_fusion_alpha_z = 0.25

        self.known_landmarks = {}
        if known_landmarks:
            for landmark_id, position in known_landmarks.items():
                self.known_landmarks[int(landmark_id)] = np.asarray(
                    position, dtype=float
                ).reshape(3)

        self.x = np.zeros(4, dtype=float)
        self.P = np.eye(4, dtype=float)

        # EKF-SLAM: landmark id -> starting state index.
        self.landmark_index = {}
        self.landmarks = {}

        self.particles = None
        self.weights = None
        self.history = []

        self.initialized = False
        self.steps = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self, initial_pose, initial_covariance=None):
        self.x = np.asarray(initial_pose, dtype=float).reshape(4).copy()
        self.x[3] = wrap_angle(self.x[3])

        if initial_covariance is None:
            self.P = np.diag([0.05, 0.05, 0.05, 0.02]) ** 2
        else:
            self.P = np.asarray(initial_covariance, dtype=float).reshape(4, 4).copy()

        self.landmark_index = {}
        self.landmarks = {}
        self.history = []
        self.steps = 0

        if self.method == "particle_filter":
            self._initialize_particles()

        self.initialized = True

    def predict(self, motion):
        self._require_initialized()

        if self.method == "dead_reckoning":
            self._predict_dead_reckoning(motion)
        elif self.method == "gps_fusion":
            self._predict_dead_reckoning(motion)
        elif self.method == "ekf":
            self._predict_ekf(motion)
        elif self.method == "particle_filter":
            self._predict_particle_filter(motion)
        elif self.method == "ekf_slam":
            self._predict_ekf_slam(motion)
        elif self.method == "graph_slam":
            self._predict_graph_slam(motion)

        self.steps += 1

    def update(self, measurements):
        self._require_initialized()
        measurements = [] if measurements is None else list(measurements)

        if self.method == "dead_reckoning":
            return
        if self.method == "gps_fusion":
            self._update_gps_fusion(measurements)
        elif self.method == "ekf":
            self._update_ekf(measurements)
        elif self.method == "particle_filter":
            self._update_particle_filter(measurements)
        elif self.method == "ekf_slam":
            self._update_ekf_slam(measurements)
        elif self.method == "graph_slam":
            self._update_graph_slam(measurements)

    def get_pose(self):
        self._require_initialized()
        return {
            "x": float(self.x[0]),
            "y": float(self.x[1]),
            "z": float(self.x[2]),
            "yaw": float(wrap_angle(self.x[3])),
        }

    def get_map(self):
        self._sync_landmarks()
        return {
            int(k): np.asarray(v, dtype=float).tolist()
            for k, v in self.landmarks.items()
        }

    def get_diagnostics(self):
        return {
            "method": self.method,
            "steps": int(self.steps),
            "num_landmarks": int(len(self.landmark_index)),
            "state_dimension": int(self.x.size),
            "covariance_trace": float(np.trace(self.P)),
        }

    # ------------------------------------------------------------------
    # Dead reckoning / simple GPS fusion
    # ------------------------------------------------------------------

    def _predict_dead_reckoning(self, motion):
        dx = float(motion.get("dx", 0.0))
        dy = float(motion.get("dy", 0.0))
        dz = float(motion.get("dz", 0.0))
        dyaw = float(motion.get("dyaw", 0.0))

        self.x[0:3] += body_delta_to_world(self.x[3], dx, dy, dz)
        self.x[3] = wrap_angle(self.x[3] + dyaw)

    def _update_gps_fusion(self, measurements):
        for m in measurements:
            if m.get("type") != "gps":
                continue
            z = np.asarray(m["position"], dtype=float).reshape(3)
            axy = float(self.gps_fusion_alpha_xy)
            az = float(self.gps_fusion_alpha_z)
            self.x[0] = (1.0 - axy) * self.x[0] + axy * z[0]
            self.x[1] = (1.0 - axy) * self.x[1] + axy * z[1]
            self.x[2] = (1.0 - az) * self.x[2] + az * z[2]

    # ------------------------------------------------------------------
    # EKF localization
    # ------------------------------------------------------------------

    def _motion_terms(self, motion):
        dx = float(motion.get("dx", 0.0))
        dy = float(motion.get("dy", 0.0))
        dz = float(motion.get("dz", 0.0))
        dyaw = float(motion.get("dyaw", 0.0))
        yaw = float(self.x[3])

        c = math.cos(yaw)
        s = math.sin(yaw)

        delta = np.array([
            c * dx - s * dy,
            s * dx + c * dy,
            dz,
        ], dtype=float)

        dxd_yaw = -s * dx - c * dy
        dyd_yaw =  c * dx - s * dy

        # Conservative motion-dependent process noise.
        sx = 0.015 + 0.020 * abs(dx)
        sy = 0.015 + 0.020 * abs(dy)
        sz = 0.010 + 0.020 * abs(dz)
        syaw = 0.004 + 0.020 * abs(dyaw)
        q = np.diag([sx*sx, sy*sy, sz*sz, syaw*syaw])

        return delta, dyaw, dxd_yaw, dyd_yaw, q

    def _predict_ekf(self, motion):
        delta, dyaw, dxd_yaw, dyd_yaw, Q = self._motion_terms(motion)

        F = np.eye(4, dtype=float)
        F[0, 3] = dxd_yaw
        F[1, 3] = dyd_yaw

        self.x[0:3] += delta
        self.x[3] = wrap_angle(self.x[3] + dyaw)

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def _update_ekf(self, measurements):
        H = np.zeros((3, 4), dtype=float)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        I = np.eye(4, dtype=float)

        for m in measurements:
            if m.get("type") != "gps":
                continue

            z = np.asarray(m["position"], dtype=float).reshape(3)
            variances = np.asarray(
                m.get("covariance", [0.45**2, 0.45**2, 0.30**2]),
                dtype=float,
            ).reshape(3)
            variances = np.maximum(variances, 1e-9)
            R = np.diag(variances)

            innovation = z - H @ self.x
            S = H @ self.P @ H.T + R
            K = self.P @ H.T @ np.linalg.inv(S)

            self.x = self.x + K @ innovation
            self.x[3] = wrap_angle(self.x[3])

            KH = K @ H
            self.P = (I - KH) @ self.P @ (I - KH).T + K @ R @ K.T
            self.P = 0.5 * (self.P + self.P.T)

    # ------------------------------------------------------------------
    # EKF-SLAM
    # ------------------------------------------------------------------

    def _predict_ekf_slam(self, motion):
        delta, dyaw, dxd_yaw, dyd_yaw, Qrobot = self._motion_terms(motion)

        n = self.x.size
        F = np.eye(n, dtype=float)
        F[0, 3] = dxd_yaw
        F[1, 3] = dyd_yaw

        self.x[0:3] += delta
        self.x[3] = wrap_angle(self.x[3] + dyaw)

        Q = np.zeros((n, n), dtype=float)
        Q[0:4, 0:4] = Qrobot

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        self._sync_landmarks()

    def _update_ekf_slam(self, measurements):
        # Optional GPS can still be used if the infrastructure enables it.
        for m in measurements:
            if m.get("type") == "gps":
                self._slam_gps_update(m)

        for m in measurements:
            if m.get("type") != "landmark":
                continue

            landmark_id = int(m["id"])

            if landmark_id not in self.landmark_index:
                self._initialize_landmark(m)
                continue

            self._slam_landmark_update(m)

        self._sync_landmarks()

    def _measurement_covariance(self, m):
        rv = float(m.get("range_variance", 0.15 ** 2))
        bv = float(m.get("bearing_variance", 0.02 ** 2))
        ev = float(m.get("elevation_variance", 0.02 ** 2))
        return np.diag([
            max(rv, 1e-9),
            max(bv, 1e-9),
            max(ev, 1e-9),
        ])

    def _initialize_landmark(self, m):
        landmark_id = int(m["id"])
        r = max(float(m["range"]), 1e-6)
        b = float(m["bearing"])
        e = float(m.get("elevation", 0.0))

        rx, ry, rz, yaw = [float(v) for v in self.x[0:4]]
        theta = yaw + b
        ce = math.cos(e)
        se = math.sin(e)
        ct = math.cos(theta)
        st = math.sin(theta)

        lm = np.array([
            rx + r * ce * ct,
            ry + r * ce * st,
            rz + r * se,
        ], dtype=float)

        old_n = self.x.size
        oldP = self.P.copy()

        # Jacobian of landmark initialization wrt robot state.
        Jx = np.zeros((3, 4), dtype=float)
        Jx[0, 0] = 1.0
        Jx[1, 1] = 1.0
        Jx[2, 2] = 1.0
        Jx[0, 3] = -r * ce * st
        Jx[1, 3] =  r * ce * ct

        # Jacobian wrt measurement [range, bearing, elevation].
        Jz = np.array([
            [ce * ct, -r * ce * st, -r * se * ct],
            [ce * st,  r * ce * ct, -r * se * st],
            [se,        0.0,          r * ce],
        ], dtype=float)

        R = self._measurement_covariance(m)

        cross_landmark_old = Jx @ oldP[0:4, :]               # 3 x old_n
        cross_old_landmark = cross_landmark_old.T            # old_n x 3
        Pll = Jx @ oldP[0:4, 0:4] @ Jx.T + Jz @ R @ Jz.T

        newP = np.zeros((old_n + 3, old_n + 3), dtype=float)
        newP[:old_n, :old_n] = oldP
        newP[:old_n, old_n:] = cross_old_landmark
        newP[old_n:, :old_n] = cross_landmark_old
        newP[old_n:, old_n:] = Pll

        self.x = np.concatenate([self.x, lm])
        self.P = 0.5 * (newP + newP.T)
        self.landmark_index[landmark_id] = old_n
        self._sync_landmarks()

    def _slam_landmark_update(self, m):
        landmark_id = int(m["id"])
        idx = self.landmark_index[landmark_id]

        rx, ry, rz, yaw = [float(v) for v in self.x[0:4]]
        lx, ly, lz = [float(v) for v in self.x[idx:idx+3]]

        dx = lx - rx
        dy = ly - ry
        dz = lz - rz

        q = dx*dx + dy*dy
        rho = math.sqrt(max(q, 1e-12))
        r2 = q + dz*dz
        r = math.sqrt(max(r2, 1e-12))

        predicted = np.array([
            r,
            wrap_angle(math.atan2(dy, dx) - yaw),
            math.atan2(dz, rho),
        ], dtype=float)

        z = np.array([
            float(m["range"]),
            float(m["bearing"]),
            float(m.get("elevation", 0.0)),
        ], dtype=float)

        innovation = z - predicted
        innovation[1] = wrap_angle(innovation[1])
        innovation[2] = wrap_angle(innovation[2])

        n = self.x.size
        H = np.zeros((3, n), dtype=float)

        # d(range) / d(delta)
        gr = np.array([dx/r, dy/r, dz/r], dtype=float)

        # d(bearing) / d(delta)
        gb = np.array([-dy/max(q,1e-12), dx/max(q,1e-12), 0.0], dtype=float)

        # d(elevation) / d(delta)
        ge = np.array([
            -dz * dx / (max(r2,1e-12) * rho),
            -dz * dy / (max(r2,1e-12) * rho),
             rho / max(r2,1e-12),
        ], dtype=float)

        # Robot position contributes with negative delta derivative.
        H[0, 0:3] = -gr
        H[1, 0:3] = -gb
        H[2, 0:3] = -ge
        H[1, 3] = -1.0

        # Landmark position contributes with positive derivative.
        H[0, idx:idx+3] = gr
        H[1, idx:idx+3] = gb
        H[2, idx:idx+3] = ge

        R = self._measurement_covariance(m)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[3] = wrap_angle(self.x[3])

        I = np.eye(n, dtype=float)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def _slam_gps_update(self, m):
        n = self.x.size
        H = np.zeros((3, n), dtype=float)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        z = np.asarray(m["position"], dtype=float).reshape(3)
        variances = np.asarray(
            m.get("covariance", [0.45**2, 0.45**2, 0.30**2]),
            dtype=float,
        ).reshape(3)
        R = np.diag(np.maximum(variances, 1e-9))

        innovation = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.x[3] = wrap_angle(self.x[3])

        I = np.eye(n, dtype=float)
        KH = K @ H
        self.P = (I - KH) @ self.P @ (I - KH).T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def _sync_landmarks(self):
        if not self.landmark_index:
            self.landmarks = {}
            return
        self.landmarks = {
            int(landmark_id): self.x[idx:idx+3].copy()
            for landmark_id, idx in self.landmark_index.items()
        }

    # ------------------------------------------------------------------
    # Particle filter / Graph SLAM remain student extensions
    # ------------------------------------------------------------------

    def _initialize_particles(self):
        self.particles = np.repeat(
            self.x.reshape(1, 4), self.num_particles, axis=0
        )
        self.weights = np.ones(self.num_particles, dtype=float) / self.num_particles

    def _predict_particle_filter(self, motion):
        raise NotImplementedError(
            "[AURA LAB 2 TODO] Implement particle-filter prediction."
        )

    def _update_particle_filter(self, measurements):
        raise NotImplementedError(
            "[AURA LAB 2 TODO] Implement particle-filter update."
        )

    def _predict_graph_slam(self, motion):
        self.history.append({"type": "motion", "motion": dict(motion)})
        raise NotImplementedError(
            "[AURA LAB 2 ADVANCED] Implement GraphSLAM motion factor."
        )

    def _update_graph_slam(self, measurements):
        self.history.append({
            "type": "measurements",
            "measurements": list(measurements),
        })
        raise NotImplementedError(
            "[AURA LAB 2 ADVANCED] Implement GraphSLAM measurement factors."
        )

    def _require_initialized(self):
        if not self.initialized:
            raise RuntimeError("Call reset(initial_pose) before using estimator.")


if __name__ == "__main__":
    e = StudentEstimator(method="ekf_slam")
    e.reset([0.0, 0.0, 0.0, 0.0])
    e.predict({"dt": 0.1, "dx": 1.0, "dy": 0.0, "dz": 0.0, "dyaw": 0.01})
    e.update([{
        "type": "landmark",
        "id": 0,
        "range": 5.0,
        "bearing": 0.2,
        "elevation": 0.0,
    }])
    print("pose:", e.get_pose())
    print("map:", e.get_map())
    print("diagnostics:", e.get_diagnostics())

