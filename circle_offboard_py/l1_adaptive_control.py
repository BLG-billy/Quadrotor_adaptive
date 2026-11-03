"""L1Quad Algorithm 1 in physical NED/FRD units, without ROS dependencies.

Uses measured z=[v_NED, Omega_FRD], previous *sent* wrench [N, N*m],
Euler prediction, piecewise-constant estimation and a first-order matched LPF.
The caller supplies one fresh observation per step and its elapsed time. This
variable-step implementation is not a reproduction of the fixed-Ts proof.
"""

import numpy as np


class L1AdaptiveControl:
    """Estimate six disturbances and compensate four matched wrench channels."""

    def __init__(self, mass, inertia, gravity=9.81,
                 predictor_rates=(5., 5., 5., 10., 10., 10.),
                 cutoff=(10., 3., 3., 3.), moment_cutoff_second=None):
        self.mass = float(mass)
        self.gravity = float(gravity)
        self.inertia = np.asarray(inertia, dtype=float)
        self.predictor_rates = np.asarray(predictor_rates, dtype=float)
        self.cutoff = np.asarray(cutoff, dtype=float)
        if (not np.isfinite([self.mass, self.gravity]).all()
                or self.mass <= 0 or self.gravity <= 0
                or self.inertia.shape != (3, 3)
                or not np.isfinite(self.inertia).all()
                or not np.allclose(self.inertia, self.inertia.T)
                or np.min(np.linalg.eigvalsh(self.inertia)) <= 0):
            raise ValueError('L1 requires positive mass/gravity and SPD inertia')
        for value, size in ((self.predictor_rates, 6), (self.cutoff, 4)):
            if value.shape != (size,) or not np.isfinite(value).all() or np.any(value <= 0):
                raise ValueError('L1 rates and cutoffs must be finite positive vectors')
        self.inertia_inverse = np.linalg.inv(self.inertia)
        self.moment_cutoff_second = (None if moment_cutoff_second is None else
                                     np.asarray(moment_cutoff_second, dtype=float))
        if self.moment_cutoff_second is not None:
            value = self.moment_cutoff_second
            if value.shape != (3,) or not np.isfinite(value).all() or np.any(value <= 0):
                raise ValueError('Second moment cutoffs must be three finite positive values')
        self.reset()

    def reset(self):
        """Discard all prediction, estimation and filter history."""
        self.z_hat = None
        self.previous_z = None
        self.previous_rotation = None
        self.sigma = np.zeros(6)
        self.filtered_matched = np.zeros(4)
        self.filtered_second = np.zeros(3)
        self.prediction_error = np.zeros(6)

    def input_matrix(self, rotation):
        """Columns: thrust, Mx, My, Mz, unmatched body-x/y forces."""
        matrix = np.zeros((6, 6))
        matrix[:3, 0] = -rotation[:, 2] / self.mass
        matrix[3:, 1:4] = self.inertia_inverse
        matrix[:3, 4:6] = rotation[:, :2] / self.mass
        return matrix

    def solve_input_matrix(self, rotation, value):
        """Exact block inverse of Bbar; avoids solving a 6x6 system per step."""
        force_body = self.mass * (rotation.T @ value[:3])
        return np.r_[-force_body[2], self.inertia @ value[3:], force_body[:2]]

    def drift(self, z):
        """Nominal gravity and rigid-body gyroscopic acceleration."""
        omega = z[3:]
        return np.r_[0., 0., self.gravity,
                     -self.inertia_inverse @ np.cross(omega, self.inertia @ omega)]

    def step(self, z, rotation, sent_wrench, dt):
        """Return unbounded adaptive wrench; first observation only initializes.

        sent_wrench is the previous interval's command after actuator-interface
        clipping, converted back through the nominal wrench mapping. It is not
        a measurement of actual rotor forces. No desired trajectory enters here.
        """
        z = np.asarray(z, dtype=float)
        rotation = np.asarray(rotation, dtype=float)
        sent_wrench = np.asarray(sent_wrench, dtype=float)
        if (z.shape != (6,) or rotation.shape != (3, 3)
                or sent_wrench.shape != (4,)
                or not all(np.isfinite(x).all() for x in (z, rotation, sent_wrench))
                or not np.isfinite(dt) or dt <= 0
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or np.linalg.det(rotation) <= 0):
            raise ValueError('Invalid L1 observation, wrench, rotation or elapsed time')
        # Forward Euler prediction must not make the As feedback unstable.
        if np.max(self.predictor_rates) * dt >= 1.0:
            raise ValueError('L1 sample interval too large for predictor rates')
        if self.z_hat is None:
            self.z_hat = z.copy()
        else:
            previous_b = self.input_matrix(self.previous_rotation)
            derivative = (self.drift(self.previous_z)
                          + previous_b[:, :4] @ sent_wrench
                          + previous_b @ self.sigma
                          - self.predictor_rates * (self.z_hat - self.previous_z))
            self.z_hat = self.z_hat + dt * derivative
            self.prediction_error = self.z_hat - z
            # Algorithm 1 calls this positive auxiliary quantity h(k).
            # The predictor injects Bbar @ sigma = -h(k), per Eq. (19)-(21).
            exponent = -self.predictor_rates * dt
            coefficient = (-self.predictor_rates * np.exp(exponent)
                           / np.expm1(exponent))
            auxiliary = coefficient * self.prediction_error
            self.sigma = self.solve_input_matrix(rotation, -auxiliary)
            blend = -np.expm1(-self.cutoff * dt)
            self.filtered_matched += blend * (self.sigma[:4] - self.filtered_matched)
            if self.moment_cutoff_second is not None:
                blend_second = -np.expm1(-self.moment_cutoff_second * dt)
                self.filtered_second += blend_second * (
                    self.filtered_matched[1:] - self.filtered_second)
        if not all(np.isfinite(x).all() for x in
                   (self.z_hat, self.sigma, self.filtered_matched, self.filtered_second)):
            self.reset()
            raise ValueError('Non-finite L1 predictor state; history reset')
        self.previous_z = z.copy()
        self.previous_rotation = rotation.copy()
        result = -self.filtered_matched.copy()
        if self.moment_cutoff_second is not None:
            result[1:] = -self.filtered_second
        return result
