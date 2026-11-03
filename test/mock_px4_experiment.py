"""ROS feedback fixture used only in an isolated test domain."""
import signal
import csv
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (VehicleOdometry, VehicleLocalPosition,
                          VehicleStatus, VehicleCommand, VehicleThrustSetpoint,
                          TrajectorySetpoint, VehicleRatesSetpoint,
                          VehicleTorqueSetpoint, OffboardControlMode)


def main():
    directory = Path(sys.argv[1])
    position_mode = sys.argv[2] == 'position'
    ctbr_mode = sys.argv[2] == 'ctbr'
    l1_mode = sys.argv[2] in ('l1', 'l1observe')
    observe_only = sys.argv[2] == 'l1observe'
    rclpy.init()
    node = Node('mock_px4')
    prefix = '/refactor_test/fmu'
    odom_pub = node.create_publisher(VehicleOdometry, prefix + '/out/vehicle_odometry', 10)
    version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    status_pub = node.create_publisher(VehicleStatus, prefix + '/out/vehicle_status' + suffix, 10)
    version = getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0)
    suffix = f'_v{version}' if version else ''
    local_pub = node.create_publisher(
        VehicleLocalPosition, prefix + '/out/vehicle_local_position' + suffix, 10)
    commands, thrust_times = [], []
    rate_samples, torque_samples, modes, thrust_samples = [], [], [], []
    if l1_mode:
        node.create_subscription(
            VehicleThrustSetpoint, prefix + '/in/vehicle_thrust_setpoint',
            thrust_samples.append, 10)
    node.create_subscription(
        VehicleTorqueSetpoint, prefix + '/in/vehicle_torque_setpoint',
        torque_samples.append, 10)
    node.create_subscription(
        OffboardControlMode, prefix + '/in/offboard_control_mode', modes.append, 10)
    node.create_subscription(
        VehicleCommand, prefix + '/in/vehicle_command',
        lambda m: commands.append(m.command), 10)
    command_type = TrajectorySetpoint if position_mode else VehicleThrustSetpoint
    command_topic = '/in/trajectory_setpoint' if position_mode else '/in/vehicle_thrust_setpoint'
    if ctbr_mode:
        command_type = VehicleRatesSetpoint
        command_topic = '/in/vehicle_rates_setpoint'
        node.create_subscription(command_type, prefix + command_topic, rate_samples.append, 10)
    node.create_subscription(command_type, prefix + command_topic,
                             lambda m: thrust_times.append(time.monotonic()), 10)
    entry = 'circle_offboard' if position_mode else 'controller_experiment'
    if ctbr_mode:
        entry = 'ctbr.controller_experiment_ctbr'
    if l1_mode:
        entry = 'controller_experiment_l1'
    extra_args = ['-p', 'l1_apply_compensation:=false'] if observe_only else []
    process = subprocess.Popen([
        sys.executable, '-m', f'circle_offboard_py.{entry}', '--ros-args',
        '-p', 'fmu_namespace:=refactor_test', '-p', 'show_plot:=false',
        '-p', f'results_dir:={directory}', '-p', 'hover_seconds:=0.5',
        '-p', 'radius:=0.1', '-p', 'speed:=0.8', '-p', 'circle_laps:=1',
        '-p', 'circle_ramp_seconds:=0.0', *extra_args], start_new_session=True)
    start = time.monotonic()
    try:
        saved_at = None
        while time.monotonic() - start < 18:
            assert process.poll() is None, 'A child node crashed'
            stamp = node.get_clock().now().nanoseconds // 1000
            odom = VehicleOdometry()
            odom.timestamp = odom.timestamp_sample = stamp
            odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
            odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
            odom.position = [0.0, 0.0, -1.0]
            odom.velocity = odom.angular_velocity = [0.0, 0.0, 0.0]
            odom.q = [1.0, 0.0, 0.0, 0.0]
            odom_pub.publish(odom)
            status = VehicleStatus()
            status.timestamp = stamp
            if time.monotonic() - start > 3:
                status.arming_state = VehicleStatus.ARMING_STATE_ARMED
                status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
            status_pub.publish(status)
            local = VehicleLocalPosition()
            local.timestamp = stamp
            local.xy_valid = local.z_valid = True
            local.x, local.y, local.z = 0.0, 0.0, -1.0
            local_pub.publish(local)
            rclpy.spin_once(node, timeout_sec=0.01)
            if list(directory.glob('*.png')):
                saved_at = saved_at or time.monotonic()
                if time.monotonic() - saved_at > 1.2:
                    break
        assert list(directory.glob('*.png')), 'No automatic PNG output'
        assert VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH in commands
        assert VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM not in commands
        assert thrust_times and time.monotonic() - thrust_times[-1] > 0.5
        if ctbr_mode:
            assert rate_samples and not torque_samples
            values = np.array([[m.roll, m.pitch, m.yaw] for m in rate_samples])
            assert np.all(np.isfinite(values))
            assert np.max(np.abs(values)) <= 3.0
            assert np.max(np.abs(values[:, :2])) > 0.01
            assert all(-0.9 <= m.thrust_body[2] <= 0.0 for m in rate_samples)
            assert modes and all(m.body_rate and not m.thrust_and_torque for m in modes)
            assert all(not m.position and not m.attitude for m in modes)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        node.destroy_node()
        rclpy.shutdown()
    if l1_mode:
        diagnostics = list(directory.glob('*_l1_diagnostics.csv'))
        assert len(diagnostics) == 1
        with diagnostics[0].open() as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) > 20
        running_rows = [row for row in rows if row['running'] == '1']
        assert running_rows
        assert any(abs(float(row[f'adaptive_raw_{axis}'])) > 1e-4
                   for row in running_rows for axis in ('f', 'mx', 'my', 'mz'))
        for row in rows:
            sent = np.array([float(row[f'sent_{axis}']) for axis in ('f', 'mx', 'my', 'mz')])
            norm = np.array([float(row[f'normalized_{axis}'])
                             for axis in ('thrust', 'mx', 'my', 'mz')])
            applied = np.array([float(row[f'adaptive_applied_{axis}'])
                                for axis in ('f', 'mx', 'my', 'mz')])
            assert np.isfinite(sent).all()
            from circle_offboard_py.actuator_mapping import X500ActuatorMap
            assert np.allclose(sent, X500ActuatorMap().reconstruct(norm))
            assert np.all(np.abs(applied) <= [5., .4, .4, .15])
            if observe_only or row['running'] == '0':
                assert np.allclose(applied, 0)
        assert modes and all(m.thrust_and_torque and not m.body_rate for m in modes)
        # Compare diagnostic commands with actual messages received over ROS.
        by_stamp = {int(row['command_timestamp_us']): row for row in rows}
        verified_thrust = verified_torque = 0
        for sample in thrust_samples:
            if sample.timestamp in by_stamp:
                row = by_stamp[sample.timestamp]
                assert -sample.xyz[2] == float(row['normalized_thrust'])
                verified_thrust += 1
        for sample in torque_samples:
            if sample.timestamp in by_stamp:
                row = by_stamp[sample.timestamp]
                assert np.array_equal(sample.xyz,
                                      [float(row[f'normalized_{a}']) for a in ('mx', 'my', 'mz')])
                verified_torque += 1
        assert verified_thrust > 10 and verified_torque > 10


if __name__ == '__main__':
    main()
