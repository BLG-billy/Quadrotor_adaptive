#!/usr/bin/env python3

"""
Complete SE(3) geometric position controller for a PX4 multicopter.

Implements the position-controlled flight mode in Lee, Leok and McClamroch,
"Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on
SE(3)." PX4 NED/FRD conventions match the paper: gravity is +g*e3 and thrust
acts along -R*e3. Physical f and M are normalized only at the PX4 interface.
"""

import time
import csv
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64MultiArray
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleThrustSetpoint
from px4_msgs.msg import VehicleTorqueSetpoint


from .geometric_control import GeometricControlLaw
from .actuator_mapping import X500ActuatorMap
from .trajectory_reference import unpack_reference
from .injected_disturbance import thrust_disturbance


class GeometricControllerNode(Node, GeometricControlLaw):
    def __init__(self, node_name='geometric_controller'):
        super().__init__(node_name)

        # SITL must be started with PX4_UXRCE_DDS_NS=sim. Keep all command
        # topics separate from a real PX4's default /fmu namespace.
        self.declare_parameter('fmu_namespace', 'sim')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('enable_l1', False)
        # An explicit enable_disturbance override wins over this L1-dependent
        # default. Both switches are startup parameters, not runtime toggles.
        self.declare_parameter(
            'enable_disturbance', bool(self.get_parameter('enable_l1').value))
        self.enable_disturbance = bool(self.get_parameter('enable_disturbance').value)
        self.disturbance_start_time = None
        self.disturbance_running = False
        self.injected_thrust = 0.0
        self.last_predictor_wrench = np.zeros(4)

        # PX4 GZ x500 rigid-body parameters.
        # Base body plus the four 0.016076923 kg rotors.
        self.declare_parameter('mass', 2.0643076923076924)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('inertia_xx', 0.02166666666666667)
        self.declare_parameter('inertia_yy', 0.02166666666666667)
        self.declare_parameter('inertia_zz', 0.04000000000000001)

        # Mass/inertia-normalized initial bandwidths; still require SITL tuning.
        self.declare_parameter('position_frequency', [2.5, 2.5, 2.5])
        self.declare_parameter('attitude_frequency', [10.0, 10.0, 6.0])
        self.declare_parameter('position_damping', 1.0)
        self.declare_parameter('attitude_damping', 0.85)
        mass = float(self.get_parameter('mass').value)
        inertia = np.array([float(self.get_parameter('inertia_' + axis + axis).value)
                            for axis in 'xyz'])
        position_frequency = np.asarray(self.get_parameter('position_frequency').value)
        attitude_frequency = np.asarray(self.get_parameter('attitude_frequency').value)
        damping_p = float(self.get_parameter('position_damping').value)
        damping_r = float(self.get_parameter('attitude_damping').value)
        if (position_frequency.shape != (3,) or attitude_frequency.shape != (3,)
                or not np.isfinite(np.r_[position_frequency, attitude_frequency,
                                         damping_p, damping_r]).all()
                or np.any(position_frequency <= 0) or np.any(attitude_frequency <= 0)
                or damping_p <= 0 or damping_r <= 0):
            raise ValueError('Frequencies (rad/s) and damping must be positive')
        gain_vectors = (
            ('kx', mass * position_frequency**2),
            ('kv', 2 * damping_p * mass * position_frequency),
            ('kR', inertia * attitude_frequency**2),
            ('kOmega', 2 * damping_r * inertia * attitude_frequency))
        for prefix, gains in gain_vectors:
            for axis, value in zip('xyz', gains):
                self.declare_parameter(prefix + '_' + axis, float(value))

        # Standard GZ x500 nonlinear actuator mapping is the single interface.
        self.declare_parameter('state_timeout', 0.1)
        self.declare_parameter('save_control_diagnostics', True)
        self.declare_parameter(
            'results_dir', '~/ws_sensor_combined/src/circle_offboard_py/results')
        self.declare_parameter('experiment_name', 'geometric')
        self.declare_parameter('max_thrust', 0.90)
        self.declare_parameter('max_torque_command', 0.80)
        self.declare_parameter('max_commanded_rate', 3.0)
        self.declare_parameter('max_commanded_acceleration', 15.0)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        if not np.isfinite(self.rate_hz) or self.rate_hz <= 0:
            raise ValueError('rate_hz must be positive')
        self.dt_nominal = 1.0 / self.rate_hz
        self.yaw = float(self.get_parameter('yaw').value)

        self.mass = float(self.get_parameter('mass').value)
        self.gravity = float(self.get_parameter('gravity').value)
        self.inertia = np.diag([
            float(self.get_parameter('inertia_xx').value),
            float(self.get_parameter('inertia_yy').value),
            float(self.get_parameter('inertia_zz').value)])
        self.kx = self.parameter_vector('kx')
        self.kv = self.parameter_vector('kv')
        self.kR = self.parameter_vector('kR')
        self.kOmega = self.parameter_vector('kOmega')
        self.get_logger().info(
            f'Effective gains: Kx={self.kx.tolist()}, Kv={self.kv.tolist()}, '
            f'KR={self.kR.tolist()}, KOmega={self.kOmega.tolist()}')
        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.max_torque_command = float(
            self.get_parameter('max_torque_command').value)
        self.max_commanded_rate = float(
            self.get_parameter('max_commanded_rate').value)
        self.max_commanded_acceleration = float(
            self.get_parameter('max_commanded_acceleration').value)

        parameters = np.r_[self.mass, self.gravity, np.diag(self.inertia),
                           self.kx, self.kv, self.kR, self.kOmega,
                           self.max_thrust, self.max_torque_command,
                           self.max_commanded_rate, self.max_commanded_acceleration]
        if not np.isfinite(parameters).all() or np.any(parameters <= 0):
            raise ValueError('Controller gains, model and limits must be finite and positive')
        self.actuator_map = X500ActuatorMap(self.max_thrust, self.max_torque_command)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        if not np.isfinite(self.state_timeout) or self.state_timeout <= 0:
            raise ValueError('state_timeout must be finite and positive')
        self.base_odom_stamp = None
        self.base_odom_sequence = 0
        self.base_used_sequence = -1
        self.control_sample_stamp = None
        self.high_order_received = None
        self.reference_jerk = np.zeros(3)
        self.reference_snap = np.zeros(3)
        self.reference_yaw_rate = self.reference_yaw_acceleration = 0.
        self.control_diagnostics = {}
        self.control_stream = None
        self.control_rows = 0
        if node_name != 'ctbr_controller' and self.get_parameter('save_control_diagnostics').value:
            directory = Path(str(self.get_parameter('results_dir').value)).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            name = str(self.get_parameter('experiment_name').value)
            name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name)
            stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
            self.control_path = directory / f'{name}_{stamp}_control_diagnostics.csv'
            self.control_stream = self.control_path.open('w', newline='', encoding='utf-8')
            self.control_writer = csv.writer(self.control_stream)
            columns = ['timestamp_us', 'sample_us', 'dt_s', 'torque_fraction', 'injected_thrust_N',
                       'previous_callback_ms', 'callback_peak_ms', 'control_overruns',
                       'recent_control_hz', 'recent_odometry_hz']
            for field in ('eR', 'eOmega', 'omega_c', 'omega_c_dot', 'attitude',
                          'rate', 'gyro', 'transport', 'acceleration',
                          'rate_limited', 'acceleration_limited'):
                columns.extend(field + '_' + axis for axis in 'xyz')
            for field in ('requested', 'sent', 'normalized', 'limited'):
                columns.extend(field + '_' + axis for axis in ('f', 'mx', 'my', 'mz'))
            self.control_writer.writerow(columns)
        self.position = None
        self.velocity = None
        self.rotation = None
        self.angular_velocity = None
        self.last_control_time = None
        self.next_control_due = None
        self.control_scheduler_active = False
        self.control_callback_ms = 0.0
        self.control_callback_peak_ms = 0.0
        self.control_overruns = 0
        self.rate_window_start = time.monotonic()
        self.rate_window_commands = 0
        self.rate_window_odom_sequence = 0
        self.odometry_receive_hz = 0.0
        self.control_publish_hz = 0.0
        self.last_odometry_time = None
        self.last_sent_wrench = np.zeros(4)
        self.last_sent_normalized = np.zeros(4)
        self.last_command_timestamp = 0

        namespace = str(self.get_parameter('fmu_namespace').value).strip('/')
        if not namespace or namespace == 'fmu':
            raise ValueError('fmu_namespace must identify an isolated SITL vehicle')
        self.fmu_prefix = f'/{namespace}/fmu'
        self.create_control_publishers()
        self.reference_msg = None
        self.reference_received = None
        self.complete = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            VehicleOdometry, self.fmu_prefix + '/out/vehicle_odometry',
            self.odometry_callback, qos)
        self.create_subscription(
            TrajectorySetpoint, '/circle/reference_trajectory', self.reference_callback, 10)
        self.create_subscription(
            Float64MultiArray, '/circle/reference_high_order', self.high_order_callback, 1)
        self.create_subscription(String, '/circle/experiment_state', self.phase_callback, 10)
        self.create_subscription(String, '/circle/flight_state', self.disturbance_flight_callback, 10)

        self.timer = self.create_timer(self.dt_nominal, self.timer_callback)

    def create_control_publishers(self):
        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint, self.fmu_prefix + '/in/vehicle_thrust_setpoint', 10)
        self.torque_pub = self.create_publisher(
            VehicleTorqueSetpoint, self.fmu_prefix + '/in/vehicle_torque_setpoint', 10)

    def parameter_vector(self, prefix):
        return np.array([
            float(self.get_parameter(f'{prefix}_x').value),
            float(self.get_parameter(f'{prefix}_y').value),
            float(self.get_parameter(f'{prefix}_z').value)])

    def timestamp_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def odometry_callback(self, msg):
        stamp = int(msg.timestamp_sample or msg.timestamp)
        if stamp and stamp == self.base_odom_stamp:
            return
        position = np.asarray(msg.position, dtype=float)
        velocity = np.asarray(msg.velocity, dtype=float)
        quaternion = np.asarray(msg.q, dtype=float)
        angular_velocity = np.asarray(msg.angular_velocity, dtype=float)
        if not np.all(np.isfinite(np.concatenate(
                (position, velocity, quaternion, angular_velocity)))):
            return
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().error(
                'VehicleOdometry pose frame is not NED; controller paused.',
                throttle_duration_sec=2.0)
            return
        if msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.get_logger().error(
                'Odometry velocity frame is not NED; controller paused.',
                throttle_duration_sec=2.0)
            return
        if np.linalg.norm(quaternion) < 1e-9:
            return
        previous_stamp = self.base_odom_stamp
        self.base_odom_stamp = stamp
        self.base_odom_sequence += 1
        self.position = position
        self.velocity = velocity
        self.rotation = self.quaternion_to_rotation(quaternion)
        self.angular_velocity = angular_velocity
        self.last_odometry_time = time.monotonic()
        self.on_odometry_accepted(previous_stamp)
        # A timer may run just before the subscription delivering its sample.
        # Complete that due update here rather than waiting another 10 ms.
        if self.control_scheduler_active:
            self.timer_callback()

    def on_odometry_accepted(self, previous_stamp):
        """Hook after validation and before scheduling control for this sample."""

    def reference_callback(self, msg):
        # Legacy publishers remain supported; prefer the atomic high-order stream.
        now = time.monotonic()
        if self.high_order_received is not None and now - self.high_order_received < .5:
            return
        values = np.r_[msg.position, msg.velocity, msg.acceleration, msg.yaw]
        if not np.isfinite(values).all():
            return
        self.reference_msg = msg
        self.reference_received = now

    def high_order_callback(self, msg):
        try:
            reference = unpack_reference(msg.data)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.)
            return
        self.reference_msg = reference
        self.high_order_received = self.reference_received = time.monotonic()

    def prepare_reference(self, msg):
        """Use derivatives from the same packet, never a second asynchronous topic."""
        self.yaw = float(msg.yaw)
        self.reference_jerk = np.asarray(getattr(msg, 'jerk', np.zeros(3)), dtype=float)
        self.reference_snap = np.asarray(getattr(msg, 'snap', np.zeros(3)), dtype=float)
        self.reference_yaw_rate = float(getattr(msg, 'yawspeed', 0.))
        self.reference_yaw_acceleration = float(getattr(msg, 'yawaccel', 0.))
        if not np.isfinite(np.r_[self.reference_jerk, self.reference_snap,
                                 self.reference_yaw_rate, self.reference_yaw_acceleration]).all():
            raise ValueError('Non-finite trajectory derivatives')

    def disturbance_flight_callback(self, msg):
        self.disturbance_running = msg.data == 'running'
        if self.disturbance_running and self.disturbance_start_time is None:
            self.disturbance_start_time = time.monotonic()

    def current_disturbance(self):
        elapsed = (None if self.disturbance_start_time is None
                   else time.monotonic() - self.disturbance_start_time)
        self.injected_thrust = thrust_disturbance(
            elapsed, self.enable_disturbance and self.disturbance_running)
        return self.injected_thrust

    def map_control_wrench(self, thrust, moment):
        return self.actuator_map.map(thrust, moment)

    def phase_callback(self, msg):
        self.complete = msg.data == 'complete'

    def timer_callback(self):
        self.control_scheduler_active = True
        now = time.monotonic()
        if (self.complete or self.position is None or self.reference_msg is None
                or self.last_odometry_time is None
                or now - self.last_odometry_time > self.state_timeout
                or now - self.reference_received > 0.5):
            self.next_control_due = None
            return
        if self.base_used_sequence == self.base_odom_sequence:
            return
        if self.next_control_due is not None and now + 1e-9 < self.next_control_due:
            return
        # Anchor deadlines to a cadence, not now+period: 125 Hz input can then
        # supply 100 Hz control without quantizing down to 62.5 Hz.
        if self.next_control_due is None:
            self.next_control_due = now + self.dt_nominal
        else:
            slots = max(1, int((now - self.next_control_due) / self.dt_nominal) + 1)
            self.next_control_due += slots * self.dt_nominal
        stamp = self.base_odom_stamp
        if stamp and self.control_sample_stamp and stamp > self.control_sample_stamp:
            dt = (stamp - self.control_sample_stamp) * 1e-6
        else:
            dt = (self.dt_nominal if self.last_control_time is None
                  else now - self.last_control_time)
        self.last_control_time = now
        self.control_sample_stamp = stamp
        self.base_used_sequence = self.base_odom_sequence
        self.control_dt = dt
        callback_start = time.perf_counter()
        previous_command_timestamp = self.last_command_timestamp
        try:
            self.compute_and_publish(self.reference_msg, dt)
        except (ValueError, RuntimeError, FloatingPointError) as error:
            self.get_logger().error(f'Control paused: {error}', throttle_duration_sec=2.)
        finally:
            self.control_callback_ms = (time.perf_counter() - callback_start) * 1000.
            self.control_callback_peak_ms = max(
                self.control_callback_peak_ms, self.control_callback_ms)
            if self.control_callback_ms > self.dt_nominal * 1000.:
                self.control_overruns += 1
        if self.last_command_timestamp != previous_command_timestamp:
            self.rate_window_commands += 1
        window = now - self.rate_window_start
        if window >= 5.:
            self.control_publish_hz = self.rate_window_commands / window
            self.odometry_receive_hz = (
                self.base_odom_sequence - self.rate_window_odom_sequence) / window
            self.get_logger().info(
                f'Control rate={self.control_publish_hz:.1f} Hz, '
                f'odometry={self.odometry_receive_hz:.1f} Hz, '
                f'callback={self.control_callback_ms:.2f} ms, '
                f'peak={self.control_callback_peak_ms:.2f} ms, '
                f'overruns={self.control_overruns}')
            self.rate_window_start = now
            self.rate_window_commands = 0
            self.rate_window_odom_sequence = self.base_odom_sequence

    def compute_and_publish(self, msg, dt):
        self.prepare_reference(msg)
        thrust, moment, _ = self.geometric_control(
            np.asarray(msg.position), np.asarray(msg.velocity),
            np.asarray(msg.acceleration), dt)
        self.publish_wrench(thrust, moment)

    def publish_wrench(self, thrust, moment):
        # Positive paper thrust f acts along body -z.
        # Keep nominal post-limit input separate: injected uncertainty must not
        # be supplied to the predictor as if it were known control effort.
        normalized, sent = self.map_control_wrench(thrust, moment)
        self.last_predictor_wrench = sent.copy()
        disturbance = self.current_disturbance()
        requested = np.r_[thrust + disturbance, moment]
        if disturbance != 0.0:
            normalized, sent = self.map_control_wrench(requested[0], moment)
        thrust_command, torque_command = float(normalized[0]), normalized[1:]
        timestamp = self.timestamp_us()

        thrust_msg = VehicleThrustSetpoint()
        thrust_msg.timestamp = timestamp
        thrust_msg.timestamp_sample = self.base_odom_stamp or timestamp
        thrust_msg.xyz = [0.0, 0.0, -thrust_command]
        self.thrust_pub.publish(thrust_msg)

        torque_msg = VehicleTorqueSetpoint()
        torque_msg.timestamp = timestamp
        torque_msg.timestamp_sample = self.base_odom_stamp or timestamp
        torque_msg.xyz = torque_command.astype(float).tolist()
        self.torque_pub.publish(torque_msg)
        # Snapshot the clipped float32 wire command, not the requested wrench.
        self.last_sent_normalized = normalized.copy()
        self.last_sent_wrench = sent.copy()
        self.last_command_timestamp = timestamp
        self.write_control_diagnostics(requested)
        return sent.copy()

    def write_control_diagnostics(self, requested):
        if self.control_stream is None or not self.control_diagnostics:
            return
        row = [self.last_command_timestamp, self.base_odom_stamp,
               getattr(self, 'control_dt', self.dt_nominal),
               self.actuator_map.last_torque_fraction,
               self.injected_thrust, self.control_callback_ms,
               self.control_callback_peak_ms, self.control_overruns, self.control_publish_hz,
               self.odometry_receive_hz]
        for field in ('eR', 'eOmega', 'omega_c', 'omega_c_dot', 'attitude',
                      'rate', 'gyro', 'transport', 'acceleration'):
            row.extend(self.control_diagnostics[field])
        row.extend(self.commanded_rate_limited.astype(int))
        row.extend(self.commanded_acceleration_limited.astype(int))
        row.extend(requested)
        row.extend(self.last_sent_wrench)
        row.extend(self.last_sent_normalized)
        limited = ~np.isclose(requested, self.last_sent_wrench, rtol=1e-5, atol=1e-6)
        row.extend(limited.astype(int))
        self.control_writer.writerow(row)
        self.control_rows += 1
        if self.control_rows % 50 == 0:
            self.control_stream.flush()

    def destroy_node(self):
        if self.control_stream is not None:
            self.control_stream.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    from .l1_geometric_controller_node import L1GeometricControllerNode
    node = L1GeometricControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
