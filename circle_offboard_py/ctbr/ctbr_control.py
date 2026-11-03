"""Geometric attitude feedback producing body rates for PX4's rate loop."""
import numpy as np

from ..geometric_control import GeometricControlLaw


class CTBRControlLaw(GeometricControlLaw):
    def ctbr_control(self, p_ref, v_ref, a_ref, dt):
        """Return physical thrust and FRD body-rate targets in rad/s.

        Omega_sp = R.T Rc Omega_c - K_att e_R. PX4 closes the angular-rate
        loop; the paper's physical moment law is not used for this interface.
        """
        force, rc = self.desired_force_and_rotation(p_ref, v_ref, a_ref)
        omega_c, _ = self.commanded_rates(rc, dt)
        r = self.rotation
        e_r = self.vee(0.5 * (rc.T @ r - r.T @ rc))
        rate = r.T @ rc @ omega_c - self.ctbr_attitude_gain * e_r
        rate = np.clip(rate, -self.max_commanded_rate, self.max_commanded_rate)
        thrust = -float(force @ r[:, 2])
        return thrust, rate
