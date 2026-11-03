"""Static inverse for stock PX4 GZ x500, retaining PX4's normalized allocator.

Requires stock rotor ordering/geometry, normalized RPY mixer and THR_MDL_FAC=0.
Models steady-state rotor forces, not measured wrench or motor lag.
"""
import numpy as np


class X500ActuatorMap:
    """Invert physical rotor forces, then encode them as normalized PX4 inputs."""

    def __init__(self, max_output=.9, max_torque=.8, thrust_coefficient=8.54858e-6,
                 speed_min=150., speed_max=1000., arm=.174, moment_ratio=.016):
        values = np.array([max_output, max_torque, thrust_coefficient, speed_min,
                           speed_max, arm, moment_ratio])
        if (not np.isfinite(values).all() or not 0 < max_output <= 1
                or not 0 < max_torque <= 1 or thrust_coefficient <= 0
                or speed_min < 0 or speed_max <= speed_min or arm <= 0 or moment_ratio <= 0):
            raise ValueError('Invalid x500 actuator model or limits')
        self.max_output, self.max_torque = max_output, max_torque
        self.kf = thrust_coefficient
        self.speed_min = speed_min
        self.speed_span = speed_max - speed_min
        # FRD rotor positions (+x,+y),(-x,-y),(+x,-y),(-x,+y).
        sx, sy, sz = np.array([-1, 1, 1, -1]), np.array([1, -1, 1, -1]), np.array([1, 1, -1, -1])
        self.force_matrix = np.vstack((np.ones(4), arm * sx, arm * sy, moment_ratio * sz))
        self.force_inverse = np.linalg.inv(self.force_matrix)
        # motor_u = collective + sx/sqrt(2)*roll + sy/sqrt(2)*pitch + sz*yaw
        self.mixer = np.column_stack((np.ones(4), sx / np.sqrt(2), sy / np.sqrt(2), sz))
        self.encoder = np.linalg.inv(self.mixer)
        self.last_rotor_outputs = np.zeros(4)
        self.last_torque_fraction = 1.

    def rotor_force(self, output):
        return self.kf * (self.speed_min + self.speed_span * np.asarray(output))**2

    def collective(self, thrust):
        """Exact equal-rotor collective inverse, also used by CTBR."""
        force = np.clip(thrust / 4, self.rotor_force(0.), self.rotor_force(self.max_output))
        return float(np.clip((np.sqrt(force / self.kf) - self.speed_min) / self.speed_span,
                             0., self.max_output))

    def reconstruct(self, normalized):
        """Equivalent steady-state wrench from actual float32 wire inputs."""
        rotors = np.clip(self.mixer @ normalized, 0., 1.)
        return self.force_matrix @ self.rotor_force(rotors)

    def map_attitude_priority(self, thrust, moment):
        """Shift collective to retain attitude authority during inverted recovery.

        This deliberately trades collective tracking for moment tracking. Use
        only in the explicit flip maneuver, never as the normal flight default.
        """
        moment = np.asarray(moment, dtype=float)
        if moment.shape != (3,) or not np.isfinite(np.r_[thrust, moment]).all():
            raise ValueError('Wrench must contain four finite values')
        lower = self.rotor_force(0.)
        upper = self.rotor_force(self.max_output - 1e-6)
        delta = self.force_inverse @ np.r_[0., moment]
        fraction = min(1., (upper - lower) / max(float(np.ptp(delta)), 1e-15))
        delta *= fraction
        collective = float(np.clip(thrust / 4, lower - delta.min(), upper - delta.max()))
        normalized, sent = self.map(4 * collective, fraction * moment)
        self.last_torque_fraction *= fraction
        return normalized, sent

    def map(self, thrust, moment):
        """Preserve collective and uniformly reduce moments until feasible.

        Unlike independent wrench clipping, this enforces all motor bounds
        before PX4 allocation. A tiny interior margin absorbs float32 rounding.
        """
        request = np.r_[thrust, moment].astype(float)
        if request.shape != (4,) or not np.isfinite(request).all():
            raise ValueError('Wrench must contain four finite values')
        lower, upper = self.rotor_force(0.), self.rotor_force(self.max_output - 1e-6)
        collective_force = np.clip(thrust / 4, lower, upper)
        delta_force = self.force_inverse @ np.r_[0., request[1:]]

        def candidate(fraction):
            forces = collective_force + fraction * delta_force
            if np.any(forces < lower) or np.any(forces > upper):
                return None
            outputs = (np.sqrt(forces / self.kf) - self.speed_min) / self.speed_span
            normalized = self.encoder @ outputs
            if np.any(np.abs(normalized[1:]) > self.max_torque):
                return None
            return normalized

        normalized = candidate(1.)
        fraction = 1.
        if normalized is None:
            lo, hi = 0., 1.
            for _ in range(32):
                mid = (lo + hi) / 2
                if candidate(mid) is None:
                    hi = mid
                else:
                    lo = mid
            fraction = lo
            normalized = candidate(lo)
        normalized = normalized.astype(np.float32).astype(float)
        self.last_rotor_outputs = self.mixer @ normalized
        self.last_torque_fraction = fraction
        return normalized, self.reconstruct(normalized)
