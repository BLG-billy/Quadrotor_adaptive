"""Independent derivative, nonlinear allocation and boundary regressions."""
import numpy as np
import pytest

from circle_offboard_py.geometric_control import GeometricControlLaw
from circle_offboard_py.actuator_mapping import X500ActuatorMap
from circle_offboard_py.trajectory_reference import circle_reference, takeoff_reference, unpack_reference


def law():
    control = GeometricControlLaw()
    control.mass, control.gravity, control.yaw = 2.0643076923, 9.81, .2
    control.inertia = np.diag([.02166667, .02166667, .04])
    control.kx, control.kv = np.array([4., 4., 6.]), np.array([3., 3., 4.])
    control.kR, control.kOmega = np.array([.8, .8, .5]), np.array([.2, .2, .12])
    control.position, control.velocity = np.array([.1, -.1, -1.]), np.array([.03, .04, .02])
    control.rotation = control.quaternion_to_rotation([.99, .02, -.03, .04])
    control.angular_velocity = np.array([.1, -.05, .03])
    control.max_commanded_rate, control.max_commanded_acceleration = 100., 1000.
    return control


def test_analytic_rates_match_independent_time_differentiation():
    c = law()
    p, v, a, j, s = circle_reference(.7, 1., .8, 2.)
    p[2] = -1.
    c.reference_jerk, c.reference_snap = j, s
    c.reference_yaw_rate, c.reference_yaw_acceleration = .3, -.1
    force, rc = c.desired_force_and_rotation(p, v, a)
    omega, acceleration = c.commanded_rates(rc, .01)
    b3 = c.rotation[:, 2]
    db3 = c.rotation @ c.hat(c.angular_velocity) @ [0., 0., 1.]
    thrust = -force @ b3
    actual_a = np.array([0., 0., c.gravity]) - thrust / c.mass * b3
    fd = -c.kx * (c.velocity - v) - c.kv * (actual_a - a) + c.mass * j
    thrust_dot = -fd @ b3 - force @ db3
    actual_j = -thrust_dot / c.mass * b3 - thrust / c.mass * db3
    p0, v0, yaw0 = c.position.copy(), c.velocity.copy(), c.yaw

    def desired_rotation(t):
        c.position = p0 + v0 * t + actual_a * t * t / 2
        c.velocity = v0 + actual_a * t + actual_j * t * t / 2
        c.yaw = yaw0 + .3 * t - .1 * t * t / 2
        return c.desired_force_and_rotation(
            p + v * t + a * t * t / 2, v + a * t + j * t * t / 2, a + j * t + s * t * t / 2)[1]

    h = 1e-4
    minus, plus = desired_rotation(-h), desired_rotation(h)
    dr = (plus - minus) / (2 * h)
    ddr = (plus - 2 * rc + minus) / (h * h)
    assert np.allclose(omega, c.vee(c.skew_part(rc.T @ dr)), atol=1e-7)
    assert np.allclose(acceleration, c.vee(c.skew_part(
        rc.T @ ddr - c.hat(omega) @ c.hat(omega))), atol=1e-6)


def test_velocity_step_does_not_create_difference_impulse_or_depend_on_dt():
    c = law()
    c.position = np.array([0., 0., -1.])
    c.rotation, c.angular_velocity = np.eye(3), np.zeros(3)
    c.yaw = 0.
    moments = []
    for vx, dt in [(0., .01), (.02, .01), (.02, .01), (.02, .002)]:
        c.velocity = np.array([vx, 0., 0.])
        moments.append(c.geometric_control(c.position, np.zeros(3), np.zeros(3), dt)[1])
    assert np.linalg.norm(moments[1]) < .03
    assert np.allclose(moments[1], moments[2])
    assert np.allclose(moments[2], moments[3])


@pytest.mark.parametrize('boundary', [0., 2.])
def test_circle_entry_and_end_of_ramp_are_continuous_through_snap(boundary):
    left = ([np.zeros(3) for _ in range(5)] if boundary == 0.
            else circle_reference(boundary - 1e-7, 1., .8, 2.))
    right = circle_reference(boundary + 1e-7, 1., .8, 2.)
    for x, y in zip(left, right):
        assert np.allclose(x, y, atol=3e-5)


def test_circle_derivatives_match_neighboring_analytic_orders():
    for t in (.3, 1.2, 3.):
        h = 1e-5
        now = circle_reference(t, 1., .8, 2.)
        left, right = circle_reference(t - h, 1., .8, 2.), circle_reference(t + h, 1., .8, 2.)
        for derivative in range(1, 5):
            assert np.allclose((right[derivative - 1] - left[derivative - 1]) / (2 * h),
                               now[derivative], atol=1e-7)


def test_takeoff_endpoints_and_reference_packet_validation():
    begin, end = takeoff_reference(0., 1., 2.), takeoff_reference(2., 1., 2.)
    assert np.allclose(begin, 0)
    assert np.allclose(end[0], [0, 0, -1])
    assert np.allclose(end[1:], 0)
    for data in ([0] * 18, [np.nan] * 19):
        with pytest.raises(ValueError):
            unpack_reference(data)


@pytest.mark.parametrize('wrench', [[20.25, .1, -.15, .02], [25., 10., -8., 3.],
                                    [100., 5., 5., 1.], [-1., 0., 0., 0.]])
def test_x500_allocation_is_motor_feasible_and_wire_reconstructs(wrench):
    model = X500ActuatorMap()
    normalized, sent = model.map(wrench[0], wrench[1:])
    # Independently apply the stock PX4 normalized mixer and Gazebo force law.
    t, x, y, z = normalized
    rotors = np.array([t - x / np.sqrt(2) + y / np.sqrt(2) + z,
                       t + x / np.sqrt(2) - y / np.sqrt(2) + z,
                       t + x / np.sqrt(2) + y / np.sqrt(2) - z,
                       t - x / np.sqrt(2) - y / np.sqrt(2) - z])
    assert np.min(rotors) >= -1e-7
    assert np.max(rotors) <= .9 + 1e-7
    forces = 8.54858e-6 * (150 + 850 * rotors)**2
    expected = [sum(forces), .174 * (-forces[0] + forces[1] + forces[2] - forces[3]),
                .174 * (forces[0] - forces[1] + forces[2] - forces[3]),
                .016 * (forces[0] + forces[1] - forces[2] - forces[3])]
    assert np.allclose(sent, expected, atol=1e-6)
    if wrench[0] == 20.25:
        assert np.allclose(sent, wrench, atol=2e-6)
    if wrench[0] == 25.:
        assert model.last_torque_fraction < 1
        assert sent[0] == pytest.approx(25., abs=2e-6)
        assert np.allclose(sent[1:], np.array(wrench[1:]) * model.last_torque_fraction, atol=2e-6)


def test_collective_inverse_has_correct_hover_and_incremental_gain():
    model = X500ActuatorMap()
    for thrust in (18., 20.25, 24.):
        u = model.collective(thrust)
        assert 4 * 8.54858e-6 * (150 + 850 * u)**2 == pytest.approx(thrust)
