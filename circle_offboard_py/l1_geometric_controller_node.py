"""Optional L1 augmentation around the geometric wrench law."""
import csv
from datetime import datetime
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np
from std_msgs.msg import String

from .geometric_controller_node import GeometricControllerNode
from .l1_adaptive_control import L1AdaptiveControl


class L1GeometricControllerNode(GeometricControllerNode):
    def __init__(self):
        self.l1 = None
        self.diagnostic_stream = None
        self.flight_running = False
        self.odom_sequence = 0
        self.used_odom_sequence = -1
        self.odom_stamp = None
        self.last_adaptation_time = None
        self.last_adaptation_stamp = None
        self.adaptation_dt = 0.0
        self.reset_count = 0
        super().__init__()
        self.declare_parameter('l1_apply_compensation', True)
        self.enable_l1 = bool(self.get_parameter('enable_l1').value)
        if not self.enable_l1:
            return
        self.declare_parameter('l1_predictor_rates', [5., 5., 5., 10., 10., 10.])
        self.declare_parameter('l1_cutoff', [10., 3., 3., 3.])
        self.declare_parameter('l1_enable_second_filter', False)
        self.declare_parameter('l1_moment_cutoff_second', [15., 15., 15.])
        self.declare_parameter('l1_compensation_limits', [5., 0.4, 0.4, 0.15])
        self.declare_parameter('l1_max_interval', 0.05)
        self.apply_compensation = bool(self.get_parameter('l1_apply_compensation').value)
        self.l1 = L1AdaptiveControl(
            self.mass, self.inertia, self.gravity,
            self.get_parameter('l1_predictor_rates').value,
            self.get_parameter('l1_cutoff').value,
            (self.get_parameter('l1_moment_cutoff_second').value
             if self.get_parameter('l1_enable_second_filter').value else None))
        self.compensation_limits = np.asarray(
            self.get_parameter('l1_compensation_limits').value, dtype=float)
        self.max_interval = float(self.get_parameter('l1_max_interval').value)
        if (self.compensation_limits.shape != (4,)
                or not np.isfinite(self.compensation_limits).all()
                or np.any(self.compensation_limits < 0)
                or not np.isfinite(self.max_interval) or self.max_interval <= 0
                or self.max_interval * np.max(self.l1.predictor_rates) >= 1):
            raise ValueError('Invalid L1 compensation limits or maximum sample interval')
        self.create_subscription(String, '/circle/flight_state', self.flight_cb, 10)
        self.base_wrench = np.zeros(4)
        self.raw_adaptive = np.zeros(4)
        self.applied_adaptive = np.zeros(4)
        directory = Path(str(self.get_parameter('results_dir').value)).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        name = str(self.get_parameter('experiment_name').value)
        name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name)
        stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        self.diagnostic_path = directory / f'{name}_{stamp}_l1_diagnostics.csv'
        self.diagnostic_stream = self.diagnostic_path.open('w', newline='', encoding='utf-8')
        self.diagnostic_writer = csv.writer(self.diagnostic_stream)
        columns = ['monotonic_s', 'command_timestamp_us', 'adaptation_dt_s',
                   'running', 'compensation_enabled', 'reset_count', 'injected_thrust_N',
                   'predictor_f', 'predictor_mx', 'predictor_my', 'predictor_mz']
        for prefix, labels in (
                ('z', ('vx', 'vy', 'vz', 'p', 'q', 'r')),
                ('z_hat', ('vx', 'vy', 'vz', 'p', 'q', 'r')),
                ('prediction_error', ('vx', 'vy', 'vz', 'p', 'q', 'r')),
                ('sigma', ('f', 'mx', 'my', 'mz', 'fx_unmatched', 'fy_unmatched')),
                ('baseline', ('f', 'mx', 'my', 'mz')),
                ('adaptive_raw', ('f', 'mx', 'my', 'mz')),
                ('adaptive_applied', ('f', 'mx', 'my', 'mz')),
                ('requested', ('f', 'mx', 'my', 'mz')),
                ('sent', ('f', 'mx', 'my', 'mz')),
                ('normalized', ('thrust', 'mx', 'my', 'mz')),
                ('saturated', ('f', 'mx', 'my', 'mz'))):
            columns.extend(f'{prefix}_{label}' for label in labels)
        self.diagnostic_writer.writerow(columns)
        self.diagnostic_stream.flush()
        self.diagnostic_rows = 0
        self.get_logger().info(
            f'L1 enabled, apply={self.apply_compensation}; diagnostics: {self.diagnostic_path}')

    def reset_adaptation(self):
        if self.l1 is not None:
            if self.l1.z_hat is not None:
                self.reset_count += 1
            self.l1.reset()
            self.raw_adaptive[:] = 0
            self.applied_adaptive[:] = 0
        self.last_adaptation_time = None
        self.last_adaptation_stamp = None
        self.adaptation_dt = 0.0

    def flight_cb(self, msg):
        running = msg.data == 'running'
        if running != self.flight_running:
            self.reset_adaptation()
        self.flight_running = running

    def on_odometry_accepted(self, previous_stamp):
        # Share the base controller's validated sequence; no second receiver gate.
        stamp = self.base_odom_stamp
        if self.l1 is not None and stamp and previous_stamp and stamp < previous_stamp:
            self.reset_adaptation()
        self.odom_stamp = stamp
        self.odom_sequence = self.base_odom_sequence

    def timer_callback(self):
        if self.l1 is not None:
            now = time.monotonic()
            if (self.complete or self.last_odometry_time is None
                    or now - self.last_odometry_time > self.max_interval
                    or self.reference_received is None
                    or now - self.reference_received > 0.5):
                self.reset_adaptation()
        super().timer_callback()

    def compute_and_publish(self, msg, dt):
        if self.l1 is None:
            return super().compute_and_publish(msg, dt)
        self.prepare_reference(msg)
        thrust, moment, _ = self.geometric_control(
            np.asarray(msg.position), np.asarray(msg.velocity), np.asarray(msg.acceleration), dt)
        self.base_wrench = np.r_[thrust, moment]
        self.applied_adaptive[:] = 0
        now = time.monotonic()
        fresh = self.used_odom_sequence != self.odom_sequence
        if (self.flight_running and fresh
                and now - self.last_odometry_time <= self.max_interval):
            if (self.odom_stamp and self.last_adaptation_stamp
                    and self.odom_stamp > self.last_adaptation_stamp):
                interval = (self.odom_stamp - self.last_adaptation_stamp) * 1e-6
            else:
                interval = (self.dt_nominal if self.last_adaptation_time is None
                            else now - self.last_adaptation_time)
            if interval > self.max_interval:
                self.reset_adaptation()
                interval = self.dt_nominal
            try:
                self.raw_adaptive = self.l1.step(
                    np.r_[self.velocity, self.angular_velocity], self.rotation,
                    self.last_predictor_wrench, interval)
                self.adaptation_dt = interval
            except ValueError as error:
                self.get_logger().warning(f'L1 reset: {error}', throttle_duration_sec=2.)
                self.reset_adaptation()
            if self.apply_compensation:
                self.applied_adaptive = np.clip(
                    self.raw_adaptive, -self.compensation_limits, self.compensation_limits)
            self.last_adaptation_time = now
            self.last_adaptation_stamp = self.odom_stamp
        else:
            self.reset_adaptation()
        self.used_odom_sequence = self.odom_sequence
        self.base_used_sequence = self.base_odom_sequence
        requested = self.base_wrench + self.applied_adaptive
        self.publish_wrench(requested[0], requested[1:])
        self.write_diagnostics(requested + np.r_[self.injected_thrust, 0., 0., 0.])

    def write_diagnostics(self, requested):
        z = np.r_[self.velocity, self.angular_velocity]
        z_hat = self.l1.z_hat if self.l1.z_hat is not None else np.full(6, np.nan)
        saturated = ~np.isclose(requested, self.last_sent_wrench, rtol=1e-6, atol=1e-7)
        self.diagnostic_writer.writerow([
            time.monotonic(), self.last_command_timestamp, self.adaptation_dt,
            int(self.flight_running), int(self.apply_compensation),
            self.reset_count, self.injected_thrust, *self.last_predictor_wrench, *z, *z_hat, *self.l1.prediction_error, *self.l1.sigma,
            *self.base_wrench, *self.raw_adaptive, *self.applied_adaptive, *requested,
            *self.last_sent_wrench, *self.last_sent_normalized, *saturated.astype(int)])
        self.diagnostic_rows += 1
        if self.diagnostic_rows % 50 == 0:
            self.diagnostic_stream.flush()

    def destroy_node(self):
        if self.diagnostic_stream is not None:
            self.diagnostic_stream.close()
        return super().destroy_node()
