"""Exercise gating, fresh samples, reset, and the disabled baseline path."""
import csv
import time

import numpy as np
import pytest
import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleOdometry
from std_msgs.msg import String

from circle_offboard_py.geometric_controller_node import GeometricControllerNode
from circle_offboard_py.l1_geometric_controller_node import L1GeometricControllerNode


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'enable_l1:=true',
                    '-p', f'results_dir:={tmp_path}'])
    instance = L1GeometricControllerNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def odometry(stamp):
    msg = VehicleOdometry()
    msg.timestamp_sample = msg.timestamp = stamp
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    msg.position = [0., 0., -1.]
    msg.q = [1., 0., 0., 0.]
    return msg


def reference(node):
    msg = TrajectorySetpoint()
    msg.position = [0., 0., -1.]
    msg.velocity = msg.acceleration = [0., 0., 0.]
    node.reference_callback(msg)
    return msg


def test_waiting_does_not_learn_and_running_initializes_from_measurement(node):
    node.odometry_callback(odometry(1))
    msg = reference(node)
    node.compute_and_publish(msg, .01)
    assert node.l1.z_hat is None
    assert np.allclose(node.applied_adaptive, 0)
    state = String()
    state.data = 'running'
    node.flight_cb(state)
    node.odometry_callback(odometry(2))
    node.compute_and_publish(msg, .01)
    assert np.allclose(node.l1.z_hat, np.r_[node.velocity, node.angular_velocity])
    assert np.allclose(node.applied_adaptive, 0)
    state.data = 'waiting'
    node.flight_cb(state)
    assert node.l1.z_hat is None
    assert np.allclose(node.l1.filtered_matched, 0)


def test_duplicate_samples_hold_command_and_stale_state_resets(node):
    msg = reference(node)
    state = String()
    state.data = 'running'
    node.flight_cb(state)
    node.odometry_callback(odometry(10))
    node.compute_and_publish(msg, .01)
    previous_sequence, previous_rows = node.odom_sequence, node.diagnostic_rows
    previous_time = node.last_odometry_time
    node.odometry_callback(odometry(10))
    node.timer_callback()
    assert node.odom_sequence == previous_sequence
    assert node.last_odometry_time == previous_time
    assert node.diagnostic_rows == previous_rows
    node.last_odometry_time = time.monotonic() - 1.
    node.timer_callback()
    assert node.l1.z_hat is None
    node.odometry_callback(odometry(11))
    node.timer_callback()
    assert node.l1.z_hat is not None
    assert np.allclose(node.applied_adaptive, 0)


def test_sample_clock_restart_resets_predictor(node):
    reference(node)
    state = String()
    state.data = 'running'
    node.flight_cb(state)
    node.odometry_callback(odometry(100))
    node.timer_callback()
    assert node.l1.z_hat is not None
    node.odometry_callback(odometry(50))
    assert node.l1.z_hat is None
    node.timer_callback()
    assert np.allclose(node.applied_adaptive, 0)


def test_compensation_is_added_before_clipping_and_log_matches_sent_input(node):
    msg = reference(node)
    node.actuator_map.max_output = .75  # Exercise final collective saturation.
    node.flight_running = True
    node.odometry_callback(odometry(1))
    node.l1.step = lambda *args: np.array([100., 5., -5., 5.])
    node.compute_and_publish(msg, .01)
    assert np.allclose(node.applied_adaptive, [5., .4, -.4, .15])
    expected = node.base_wrench + node.applied_adaptive
    assert expected[0] > node.last_sent_wrench[0]  # Final thrust limit is active.
    node.diagnostic_stream.flush()
    with node.diagnostic_path.open() as stream:
        row = next(csv.DictReader(stream))
    assert float(row['requested_f']) == pytest.approx(expected[0])
    assert float(row['sent_f']) == node.last_sent_wrench[0]
    assert row['saturated_f'] == '1'
    assert int(row['command_timestamp_us']) == node.last_command_timestamp
    # Next adaptation must see this clipped snapshot, not expected/requested.
    received = []
    node.l1.step = lambda z, r, sent, dt: (received.append(sent.copy()) or np.zeros(4))
    previous_sent = node.last_sent_wrench.copy()
    node.odometry_callback(odometry(2))
    node.compute_and_publish(msg, .01)
    assert np.array_equal(received[0], previous_sent)


def test_disabled_mode_preserves_baseline_without_l1_diagnostics(tmp_path):
    rclpy.init(args=['--ros-args', '-p', 'enable_l1:=false',
                    '-p', f'results_dir:={tmp_path}'])
    instance = L1GeometricControllerNode()
    try:
        assert instance.l1 is None
        instance.odometry_callback(odometry(1))
        msg = reference(instance)
        instance.compute_and_publish(msg, .01)
        actual = instance.last_sent_normalized.copy()
        GeometricControllerNode.compute_and_publish(instance, msg, .01)
        assert np.array_equal(actual, instance.last_sent_normalized)
        assert not list(tmp_path.glob('*l1_diagnostics.csv'))
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_baseline_uses_fresh_samples_and_sample_clock(tmp_path):
    from unittest.mock import patch
    rclpy.init(args=['--ros-args', '-p', f'results_dir:={tmp_path}'])
    instance = GeometricControllerNode()
    try:
        clock = [time.monotonic()]
        with patch('circle_offboard_py.geometric_controller_node.time.monotonic',
                   side_effect=lambda: clock[0]):
            reference(instance)
            calls = []
            instance.compute_and_publish = lambda msg, dt: calls.append(dt)
            instance.odometry_callback(odometry(1000000))
            instance.timer_callback()
            instance.timer_callback()
            instance.odometry_callback(odometry(1000000))
            instance.timer_callback()
            assert len(calls) == 1
            clock[0] += .015
            instance.odometry_callback(odometry(1015000))
            instance.timer_callback()
            assert calls[-1] == pytest.approx(.015)
            assert len(calls) == 2
            instance.last_odometry_time = clock[0] - 1.
            instance.timer_callback()
            assert len(calls) == 2
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_atomic_reference_keeps_snap_and_rejects_invalid_packet(tmp_path):
    from std_msgs.msg import Float64MultiArray
    rclpy.init(args=['--ros-args', '-p', f'results_dir:={tmp_path}'])
    instance = GeometricControllerNode()
    try:
        packet = Float64MultiArray()
        packet.data = [1., 0., 0., -1., 0., 0., 0., 0., 0., 0.,
                       .1, .2, .3, .4, .5, .6, .7, .8, .9]
        instance.high_order_callback(packet)
        instance.prepare_reference(instance.reference_msg)
        assert np.allclose(instance.reference_jerk, [.1, .2, .3])
        assert np.allclose(instance.reference_snap, [.4, .5, .6])
        assert instance.reference_yaw_rate == .8
        original = instance.reference_msg
        reference(instance)  # Legacy topic must not replace the atomic packet.
        assert instance.reference_msg is original
        packet.data = [float('nan')]*19
        instance.high_order_callback(packet)
        assert instance.reference_msg is original
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_injection_is_logged_but_excluded_from_predictor(node):
    from unittest.mock import patch
    msg = reference(node)
    for removed in ('gains_profile', 'actuator_mapping', 'hover_thrust', 'torque_scale_x'):
        assert not node.has_parameter(removed)
    assert np.allclose(node.kx, node.mass * np.array([2.5, 2.5, 2.5])**2)
    assert np.allclose(node.kv, 2 * node.mass * np.array([2.5, 2.5, 2.5]))
    assert np.allclose(node.kR, np.diag(node.inertia) * np.array([10., 10., 6.])**2)
    assert np.allclose(node.kOmega, 2 * .85 * np.diag(node.inertia) * np.array([10., 10., 6.]))
    assert np.allclose(node.l1.cutoff, [10., 3., 3., 3.])
    assert node.l1.moment_cutoff_second is None
    node.odometry_callback(odometry(1000000))
    node.disturbance_running = True
    node.disturbance_start_time = 100.
    node.prepare_reference(msg)
    with patch('circle_offboard_py.geometric_controller_node.time.monotonic', return_value=102.25):
        node.compute_and_publish(msg, .01)
    assert node.injected_thrust == pytest.approx(2. + 2. / np.sqrt(2))
    assert node.last_sent_wrench[0] - node.last_predictor_wrench[0] == pytest.approx(
        node.injected_thrust, abs=3e-6)
    assert np.allclose(node.last_predictor_wrench[1:], 0, atol=1e-6)


@pytest.mark.parametrize('l1_enabled, override, expected', [
    (False, None, False), (True, None, True),
    (False, True, True), (True, False, False),
])
def test_disturbance_default_follows_l1_but_explicit_override_wins(
        tmp_path, l1_enabled, override, expected):
    args = ['--ros-args', '-p', f'enable_l1:={str(l1_enabled).lower()}',
            '-p', f'results_dir:={tmp_path}']
    if override is not None:
        args += ['-p', f'enable_disturbance:={str(override).lower()}']
    rclpy.init(args=args)
    instance = L1GeometricControllerNode()
    try:
        from unittest.mock import patch
        assert instance.enable_disturbance is expected
        instance.disturbance_running = True
        instance.disturbance_start_time = 100.
        with patch('circle_offboard_py.geometric_controller_node.time.monotonic',
                   return_value=102.25):
            value = instance.current_disturbance()
        assert value == pytest.approx((2. + 2. / np.sqrt(2)) if expected else 0.)
    finally:
        instance.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('l1_enabled', [False, True])
def test_late_odometry_completes_due_update_without_halving_rate(tmp_path, l1_enabled):
    """A 100 Hz timer fires just before each 100 Hz odometry callback."""
    from unittest.mock import patch
    rclpy.init(args=['--ros-args', '-p', f'enable_l1:={str(l1_enabled).lower()}',
                    '-p', f'results_dir:={tmp_path}'])
    instance = L1GeometricControllerNode()
    try:
        epoch = time.monotonic()
        clock = [epoch]
        calls = []
        instance.compute_and_publish = lambda msg, dt: calls.append((clock[0], instance.base_odom_stamp))
        with patch('circle_offboard_py.geometric_controller_node.time.monotonic',
                   side_effect=lambda: clock[0]):
            reference(instance)
            for k in range(100):
                clock[0] = epoch + k*.01
                instance.reference_received = clock[0]
                instance.timer_callback()  # No fresh observation yet.
                clock[0] += .0002
                instance.odometry_callback(odometry((k+1)*10000))
                instance.odometry_callback(odometry((k+1)*10000))  # Duplicate.
        assert len(calls) == 100
        assert len({stamp for _, stamp in calls}) == 100
        assert (len(calls)-1)/(calls[-1][0]-calls[0][0]) == pytest.approx(100.)
    finally:
        instance.destroy_node()
        rclpy.shutdown()
