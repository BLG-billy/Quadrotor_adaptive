"""Check CTBR equilibrium, restoring direction, frame transport and limits."""
import math

import numpy as np
import pytest

from circle_offboard_py.ctbr.ctbr_control import CTBRControlLaw


def controller():
    law = CTBRControlLaw()
    law.position = np.array([0.0, 0.0, -1.0])
    law.velocity = np.zeros(3)
    law.rotation = np.eye(3)
    law.mass, law.gravity, law.yaw = 2.0643076923076924, 9.81, 0.0
    law.kx, law.kv = np.array([4.0, 4.0, 6.0]), np.array([3.0, 3.0, 4.0])
    law.ctbr_attitude_gain = np.array([4.0, 4.0, 3.0])
    law.max_commanded_rate, law.max_commanded_acceleration = 3.0, 15.0
    law.previous_rc = law.previous_omega_c = None
    return law


def evaluate(law):
    return law.ctbr_control(np.array([0.0, 0.0, -1.0]), np.zeros(3), np.zeros(3), 0.01)


def test_hover_equilibrium():
    law = controller()
    thrust, rates = evaluate(law)
    assert thrust == pytest.approx(law.mass * law.gravity)
    assert np.allclose(rates, 0.0)


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_attitude_feedback_restores_each_body_axis(axis):
    law = controller()
    angle = 0.1
    q = np.zeros(4)
    q[0], q[axis + 1] = math.cos(angle / 2), math.sin(angle / 2)
    law.rotation = law.quaternion_to_rotation(q)
    # Isolate attitude feedback: analytic model feedforward is nonzero even
    # on the first call when a tilted vehicle would accelerate horizontally.
    law.commanded_rates = lambda rc, dt: (np.zeros(3), np.zeros(3))
    _, rates = evaluate(law)
    assert rates[axis] == pytest.approx(-law.ctbr_attitude_gain[axis] * math.sin(angle))
    assert np.allclose(np.delete(rates, axis), 0.0)


def test_feedforward_is_transported_to_current_body_frame():
    law = controller()
    angle = 0.4
    law.rotation = law.quaternion_to_rotation([math.cos(angle / 2), 0, 0, math.sin(angle / 2)])
    omega_c = np.array([0.2, 0.1, 0.3])
    law.commanded_rates = lambda rc, dt: (omega_c, np.zeros(3))
    _, rates = evaluate(law)
    e_r = np.array([0.0, 0.0, math.sin(angle)])
    expected = law.rotation.T @ omega_c - law.ctbr_attitude_gain * e_r
    assert np.allclose(rates, expected)


def test_final_rate_command_is_limited():
    law = controller()
    law.commanded_rates = lambda rc, dt: (np.array([30.0, -30.0, 0.0]), np.zeros(3))
    _, rates = evaluate(law)
    assert np.allclose(rates, [3.0, -3.0, 0.0])
