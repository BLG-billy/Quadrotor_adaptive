"""Manage PX4 mode transitions without computing control or trajectories."""
import time

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (OffboardControlMode, VehicleCommand,
                          VehicleStatus, VehicleThrustSetpoint, TrajectorySetpoint,
                          VehicleRatesSetpoint)
from std_msgs.msg import String


class FlightManagerNode(Node):
    def __init__(self):
        super().__init__('flight_manager')
        self.declare_parameter('fmu_namespace', 'sim')
        self.declare_parameter('enable_auto_arm', False)
        self.declare_parameter('control_mode', 'wrench')
        control_mode = self.get_parameter('control_mode').value
        if control_mode not in ('wrench', 'position', 'ctbr'):
            raise ValueError(f'Unknown control mode: {control_mode}')
        self.position_mode = control_mode == 'position'
        self.ctbr_mode = control_mode == 'ctbr'
        ns = str(self.get_parameter('fmu_namespace').value).strip('/')
        if not ns or ns == 'fmu':
            raise ValueError('fmu_namespace must identify an isolated SITL vehicle')
        self.prefix = f'/{ns}/fmu'
        self.auto_arm = bool(self.get_parameter('enable_auto_arm').value)
        self.status = None
        self.status_time = 0.0
        self.wrench_time = 0.0
        self.ready_since = None
        self.last_command = 0.0
        self.last_wait_log = float('-inf')
        self.complete = False
        self.mode_pub = self.create_publisher(
            OffboardControlMode, self.prefix + '/in/offboard_control_mode', 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, self.prefix + '/in/vehicle_command', 10)
        self.flight_pub = self.create_publisher(String, '/circle/flight_state', 10)
        version = getattr(VehicleStatus, 'MESSAGE_VERSION', 0)
        suffix = f'_v{version}' if version else ''
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            VehicleStatus, self.prefix + '/out/vehicle_status' + suffix, self.status_cb, qos)
        command_type = TrajectorySetpoint if self.position_mode else VehicleThrustSetpoint
        command_topic = ('/in/trajectory_setpoint' if self.position_mode
                         else '/in/vehicle_thrust_setpoint')
        if self.ctbr_mode:
            command_type = VehicleRatesSetpoint
            command_topic = '/in/vehicle_rates_setpoint'
        self.create_subscription(command_type, self.prefix + command_topic, self.wrench_cb, qos)
        self.create_subscription(String, '/circle/experiment_state', self.phase_cb, 10)
        self.create_timer(0.05, self.tick)

    def status_cb(self, msg):
        if self.status is None or (msg.arming_state, msg.nav_state) != (
                self.status.arming_state, self.status.nav_state):
            self.get_logger().info(
                f'PX4 status: armed={msg.arming_state}, mode={msg.nav_state}')
        self.status = msg
        self.status_time = time.monotonic()

    def wrench_cb(self, msg):
        self.wrench_time = time.monotonic()

    def phase_cb(self, msg):
        if msg.data == 'complete':
            self.complete = True

    def command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command, msg.param1, msg.param2 = command, p1, p2
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info(f'PX4 command sent: {command}')

    def tick(self):
        now = time.monotonic()
        if self.status is None or now - self.status_time > 3.0:
            self.ready_since = None
            self.log_wait(
                now, f'等待 {self.prefix}/out/vehicle_status 数据；'
                '请检查 PX4 uxrce_dds_client、MicroXRCEAgent、ROS_DOMAIN_ID 和话题版本')
            return
        offboard = self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        armed = self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        state = String()
        running = offboard and armed and now - self.wrench_time <= 0.5
        state.data = 'running' if running and not self.complete else 'waiting'
        self.flight_pub.publish(state)
        if self.complete:
            if offboard and armed and now - self.last_command >= 1.0:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
                self.last_command = now
            return
        if now - self.wrench_time > 0.5:
            self.ready_since = None
            self.log_wait(
                now, '已收到 PX4 状态，等待控制指令；'
                f'请检查 {self.prefix}/out/vehicle_odometry 和参考轨迹数据')
            return
        if self.ready_since is None:
            self.ready_since = now
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = self.position_mode
        msg.body_rate = self.ctbr_mode
        msg.thrust_and_torque = not (self.position_mode or self.ctbr_mode)
        self.mode_pub.publish(msg)
        if (self.auto_arm and not (offboard and armed)
                and now - self.ready_since >= 1.0 and now - self.last_command >= 1.0):
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_command = now

    def log_wait(self, now, message):
        if now - self.last_wait_log >= 5.0:
            self.get_logger().warning(message)
            self.last_wait_log = now
