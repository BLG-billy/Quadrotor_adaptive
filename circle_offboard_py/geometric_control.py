"""SE(3) control law in physical units, independent of ROS and flight actions."""
import math

import numpy as np


class GeometricControlLaw:
    def geometric_control(self, p_ref, v_ref, a_ref, dt):
        """Evaluate paper equations (17)-(23), returning physical f and M."""
        desired_force, rc = self.desired_force_and_rotation(p_ref, v_ref, a_ref)
        ex = self.position - p_ref
        ev = self.velocity - v_ref
        e3 = np.array([0.0, 0.0, 1.0])
        omega_c, omega_c_dot = self.commanded_rates(rc, dt)

        r = self.rotation
        omega = self.angular_velocity
        eR = self.vee(0.5 * (rc.T @ r - r.T @ rc))       # Eq. (21)
        transport = r.T @ rc
        eOmega = omega - transport @ omega_c             # Eq. (21)

        thrust = -float(desired_force @ (r @ e3))        # Eq. (19)
        attitude_term = -self.kR * eR
        rate_term = -self.kOmega * eOmega
        gyro_term = np.cross(omega, self.inertia @ omega)
        transport_term = -self.inertia @ (self.hat(omega) @ transport @ omega_c)
        acceleration_term = self.inertia @ transport @ omega_c_dot
        moment = attitude_term + rate_term + gyro_term + transport_term + acceleration_term
        self.control_diagnostics = dict(
            eR=eR, eOmega=eOmega, omega_c=omega_c, omega_c_dot=omega_c_dot,
            attitude=attitude_term, rate=rate_term, gyro=gyro_term,
            transport=transport_term, acceleration=acceleration_term)
        return thrust, moment, (ex, eR, ev, eOmega)

    def desired_force_and_rotation(self, p_ref, v_ref, a_ref):
        """Shared position/velocity loop for wrench and body-rate interfaces."""
        e3 = np.array([0.0, 0.0, 1.0])
        ex = self.position - p_ref                         # Eq. (17)
        ev = self.velocity - v_ref                        # Eq. (18)
        desired_force = (                                # A in Eq. (23)
            -self.kx * ex - self.kv * ev
            - self.mass * self.gravity * e3
            + self.mass * a_ref)
        self.force_derivative_context = (desired_force.copy(), ev.copy(), np.asarray(a_ref))
        force_norm = float(np.linalg.norm(desired_force))
        if force_norm < 1e-6:
            raise RuntimeError('geometric controller desired force is zero')

        b3c = -desired_force / force_norm                 # Eq. (23)
        rc = self.commanded_rotation(b3c)                 # Eq. (22)
        return desired_force, rc

    def commanded_rotation(self, b3c):
        """Construct Rc using desired heading, equations (22) and (38)."""
        b1d = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        b2_raw = np.cross(b3c, b1d)
        b2_norm = float(np.linalg.norm(b2_raw))
        if b2_norm < 1e-6:  # Paper assumes b1d is not parallel to b3c.
            b1d = np.array([0.0, 1.0, 0.0])
            b2_raw = np.cross(b3c, b1d)
            b2_norm = float(np.linalg.norm(b2_raw))
        b2c = b2_raw / b2_norm
        b1c = np.cross(b2c, b3c)
        return np.column_stack((b1c, b2c, b3c))

    @staticmethod
    def unit_vector_derivatives(q, q_dot, q_ddot):
        """Normalize a vector and analytically differentiate it twice."""
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            raise ValueError('Cannot differentiate a zero direction vector')
        product = q @ q_dot
        unit = q / norm
        first = q_dot / norm - q * product / norm**3
        second = (q_ddot / norm - 2 * q_dot * product / norm**3
                  - q * (q_dot @ q_dot + q @ q_ddot) / norm**3
                  + 3 * q * product**2 / norm**5)
        return unit, first, second

    def commanded_rates(self, rc, dt):
        """L1Quad/Lee Appendix F analytic derivatives; no sampled differences.

        dt is retained for the shared CTBR API. Reference jerk/snap are supplied
        atomically by the trajectory node; missing derivatives mean zero, not
        numerical differentiation of asynchronous state messages.
        """
        force, ev, a_ref = self.force_derivative_context
        rotation = self.rotation
        omega = getattr(self, 'angular_velocity', np.zeros(3))
        jerk = getattr(self, 'reference_jerk', np.zeros(3))
        snap = getattr(self, 'reference_snap', np.zeros(3))
        b3 = rotation[:, 2]
        b3_dot = rotation @ self.hat(omega) @ np.array([0., 0., 1.])
        thrust = -force @ b3
        ea = np.array([0., 0., self.gravity]) - thrust / self.mass * b3 - a_ref
        force_dot = -self.kx * ev - self.kv * ea + self.mass * jerk
        thrust_dot = -force_dot @ b3 - force @ b3_dot
        ej = -thrust_dot / self.mass * b3 - thrust / self.mass * b3_dot - jerk
        force_ddot = -self.kx * ea - self.kv * ej + self.mass * snap
        b3c, db3, ddb3 = self.unit_vector_derivatives(-force, -force_dot, -force_ddot)

        heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.])
        yaw_rate = getattr(self, 'reference_yaw_rate', 0.)
        yaw_accel = getattr(self, 'reference_yaw_acceleration', 0.)
        tangent = np.array([-math.sin(self.yaw), math.cos(self.yaw), 0.])
        dh = yaw_rate * tangent
        ddh = yaw_accel * tangent - yaw_rate**2 * heading
        if np.linalg.norm(np.cross(b3c, heading)) < 1e-6:
            # Match commanded_rotation's singular-heading fallback.
            heading, dh, ddh = np.array([0., 1., 0.]), np.zeros(3), np.zeros(3)
        cross = np.cross(b3c, heading)
        dcross = np.cross(db3, heading) + np.cross(b3c, dh)
        ddcross = (np.cross(ddb3, heading) + 2 * np.cross(db3, dh)
                   + np.cross(b3c, ddh))
        b2c, db2, ddb2 = self.unit_vector_derivatives(cross, dcross, ddcross)
        db1 = np.cross(db2, b3c) + np.cross(b2c, db3)
        ddb1 = np.cross(ddb2, b3c) + 2 * np.cross(db2, db3) + np.cross(b2c, ddb3)
        rc_dot = np.column_stack((db1, db2, db3))
        rc_ddot = np.column_stack((ddb1, ddb2, ddb3))
        raw_rate = self.vee(self.skew_part(rc.T @ rc_dot))
        raw_acceleration = self.vee(self.skew_part(
            rc.T @ rc_ddot - self.hat(raw_rate) @ self.hat(raw_rate)))
        self.commanded_rate_limited = np.abs(raw_rate) > self.max_commanded_rate
        # The derivative of a componentwise saturated rate is zero outside
        # its interval; do not feed acceleration beyond a held rate limit.
        raw_acceleration = np.where(self.commanded_rate_limited, 0., raw_acceleration)
        self.commanded_acceleration_limited = (
            np.abs(raw_acceleration) > self.max_commanded_acceleration)
        return (np.clip(raw_rate, -self.max_commanded_rate, self.max_commanded_rate),
                np.clip(raw_acceleration, -self.max_commanded_acceleration,
                        self.max_commanded_acceleration))

    @staticmethod
    def quaternion_to_rotation(quaternion):
        q = np.asarray(quaternion, dtype=float)
        q_norm = float(np.linalg.norm(q))
        if q_norm < 1e-9:
            raise ValueError('invalid zero quaternion')
        w, x, y, z = q / q_norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)]])

    @staticmethod
    def hat(vector):
        x, y, z = vector
        return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])

    @staticmethod
    def vee(matrix):
        return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])

    @staticmethod
    def skew_part(matrix):
        return 0.5 * (matrix - matrix.T)
