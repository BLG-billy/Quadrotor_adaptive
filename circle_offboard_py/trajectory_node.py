#!/usr/bin/env python3
"""Publish the hover and circle reference as an independent ROS 2 node."""

import math
import time

from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String, Float64MultiArray

from .trajectory_reference import circle_reference, takeoff_reference


class CircleTrajectoryNode(Node):
    def __init__(self):
        super().__init__('circle_trajectory')
        self.declare_parameter('hover_height', 1.0)
        self.declare_parameter('takeoff_seconds', 2.0)
        self.declare_parameter('hover_seconds', 5.0)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('circle_laps', 2)
        self.declare_parameter('hover_only', False)
        self.declare_parameter('hover_experiment_seconds', 16.0)
        self.declare_parameter('circle_ramp_seconds', 2.0)
        self.height = float(self.get_parameter('hover_height').value)
        self.hover_seconds = float(self.get_parameter('hover_seconds').value)
        takeoff = float(self.get_parameter('takeoff_seconds').value)
        self.takeoff = min(takeoff, self.hover_seconds)
        self.radius = float(self.get_parameter('radius').value)
        self.speed = float(self.get_parameter('speed').value)
        self.laps = int(self.get_parameter('circle_laps').value)
        self.ramp = float(self.get_parameter('circle_ramp_seconds').value)
        if (not all(math.isfinite(value) for value in
                    (self.height, self.hover_seconds, takeoff, self.radius, self.speed, self.ramp))
                or self.height <= 0 or self.radius <= 0 or self.speed <= 0 or self.laps <= 0
                or self.ramp < 0 or self.hover_seconds < 0 or self.takeoff < 0):
            raise ValueError('Invalid circle parameters')
        self.declare_parameter('yaw', 0.0)
        self.yaw = float(self.get_parameter('yaw').value)
        if not math.isfinite(self.yaw):
            raise ValueError('yaw must be finite')
        self.omega = self.speed / self.radius
        self.circle_duration = (
            2.0 * math.pi * self.radius * self.laps / self.speed + 0.5 * self.ramp)
        self.start = None
        self.create_subscription(String, '/circle/flight_state', self.flight_callback, 10)
        self.hover_only = bool(self.get_parameter('hover_only').value)
        self.hover_duration = float(self.get_parameter('hover_experiment_seconds').value)
        if not math.isfinite(self.hover_duration) or self.hover_duration <= 0:
            raise ValueError('hover_experiment_seconds must be finite and positive')
        self.done = False
        self.reference_pub = self.create_publisher(
            TrajectorySetpoint, '/circle/reference_trajectory', 10)
        self.high_order_pub = self.create_publisher(
            Float64MultiArray, '/circle/reference_high_order', 1)
        self.state_pub = self.create_publisher(String, '/circle/experiment_state', 10)
        self.create_timer(0.01, self.publish_reference)

    def flight_callback(self, msg):
        if msg.data == 'running' and self.start is None:
            self.start = time.monotonic()

    def publish_reference(self):
        elapsed = 0.0 if self.start is None else time.monotonic() - self.start
        if self.hover_only and elapsed >= self.hover_duration:
            self.publish_state('complete')
            return
        if self.hover_only or elapsed < self.hover_seconds:
            p, v, a, jerk, snap = takeoff_reference(elapsed, self.height, self.takeoff)
            phase = 'hover'
        else:
            t = elapsed - self.hover_seconds
            if t >= self.circle_duration:
                self.done = True
                self.publish_state('complete')
                return
            p, v, a, jerk, snap = circle_reference(t, self.radius, self.omega, self.ramp)
            p[2] = -self.height
            phase = 'circle'
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position, msg.velocity, msg.acceleration = p.tolist(), v.tolist(), a.tolist()
        msg.jerk = jerk.tolist()
        msg.yawspeed = 0.0
        high_order = Float64MultiArray()
        high_order.data = [float(msg.timestamp), *p, *v, *a, *jerk, *snap,
                           self.yaw, 0.0, 0.0]
        self.high_order_pub.publish(high_order)
        msg.yaw = self.yaw
        self.reference_pub.publish(msg)
        self.publish_state(phase)

    def publish_state(self, value):
        msg = String()
        msg.data = value
        self.state_pub.publish(msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = CircleTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
