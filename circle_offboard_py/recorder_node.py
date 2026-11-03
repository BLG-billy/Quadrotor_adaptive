#!/usr/bin/env python3
"""Record and plot trajectory data independently of the control process."""

import csv
from datetime import datetime
import os
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from std_msgs.msg import String

from .trajectory_results import aligned_results, draw_results


class ExperimentRecorderNode(Node):
    def __init__(self):
        super().__init__('experiment_recorder')
        self.declare_parameter('fmu_namespace', 'sim')
        self.declare_parameter('experiment_name', 'geometric')
        self.declare_parameter(
            'results_dir', '~/ws_sensor_combined/src/circle_offboard_py/results')
        self.declare_parameter('show_plot', True)
        ns = str(self.get_parameter('fmu_namespace').value).strip('/')
        if not ns or ns == 'fmu':
            raise ValueError('fmu_namespace must identify an isolated SITL vehicle')
        name = str(self.get_parameter('experiment_name').value)
        name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name)
        self.results_dir = Path(str(self.get_parameter('results_dir').value)).expanduser()
        self.results_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        self.stem = self.results_dir / f'{name or "experiment"}_{stamp}'
        self.show_plot = bool(self.get_parameter('show_plot').value) and bool(
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        import matplotlib
        if not self.show_plot:
            matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
        if Path(font_path).is_file():
            font_manager.fontManager.addfont(font_path)
            plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        self.plt = plt
        if self.show_plot:
            plt.ion()
        self.fig, self.axes = plt.subplots(1, 3, figsize=(16, 5))
        if self.show_plot:
            plt.show(block=False)
        self.actual = []
        self.target = []
        self.circle_start = float('inf')
        self.start = time.monotonic()
        self.saved = False
        self.finished = False
        self.last_plot_update = 0.0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        version = getattr(VehicleLocalPosition, 'MESSAGE_VERSION', 0)
        suffix = f'_v{version}' if version else ''
        self.create_subscription(
            VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position{suffix}',
            self.position_cb, qos)
        self.create_subscription(
            TrajectorySetpoint, '/circle/reference_trajectory', self.target_cb, 10)
        self.create_subscription(
            String, '/circle/experiment_state', self.state_cb, 10)
        self.create_timer(0.05, self.update_plot)
        self.get_logger().info(f'实验结果将自动保存到 {self.results_dir}')

    def position_cb(self, msg):
        if (msg.xy_valid and msg.z_valid and not self.finished
                and np.all(np.isfinite([msg.x, msg.y, msg.z]))):
            self.actual.append((time.monotonic() - self.start, msg.x, msg.y, msg.z))

    def target_cb(self, msg):
        values = [*msg.position, *msg.velocity]
        if not self.finished and np.all(np.isfinite(values)):
            self.target.append((time.monotonic() - self.start, *values))

    def state_cb(self, msg):
        if msg.data == 'circle' and not np.isfinite(self.circle_start):
            self.circle_start = time.monotonic() - self.start
            self.get_logger().info('悬停结束，开始统计圆周误差（包含启动加速段）')
        if msg.data == 'complete' and not self.saved:
            self.finished = True
            self.save_result()

    def update_plot(self):
        if self.saved:
            if self.show_plot and self.plt.fignum_exists(self.fig.number):
                self.fig.canvas.flush_events()
            return
        now = time.monotonic()
        if now - self.last_plot_update < 0.5:
            if self.show_plot:
                self.fig.canvas.flush_events()
            return
        self.last_plot_update = now
        if len(self.actual) < 2 or len(self.target) < 2:
            if self.show_plot:
                self.fig.canvas.flush_events()
            return
        data = aligned_results(self.actual, self.target, self.circle_start)
        draw_results(self.axes, data, np.asarray(self.target))
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        if self.show_plot:
            self.fig.canvas.flush_events()

    def save_result(self):
        if self.saved:
            return
        self.finished = True
        if len(self.actual) < 2 or len(self.target) < 2:
            self.get_logger().warning('样本不足，未生成实验结果')
            return
        self.last_plot_update = 0.0
        self.update_plot()
        data = aligned_results(self.actual, self.target, self.circle_start)
        csv_path = self.stem.with_suffix('.csv')
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow([
                'time_s', 'actual_x_m', 'actual_y_m', 'actual_z_m',
                'target_x_m', 'target_y_m', 'target_z_m',
                'horizontal_error_m', 'circle_phase'])
            writer.writerows(data)
        self.fig.savefig(self.stem.with_suffix('.png'), dpi=150)
        self.saved = True
        self.get_logger().info(f'已保存 PNG 和 CSV：{self.stem}')

    def destroy_node(self):
        self.plt.close(self.fig)
        return super().destroy_node()
