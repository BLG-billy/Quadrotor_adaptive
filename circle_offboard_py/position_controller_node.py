"""Forward references to PX4's built-in position controller."""
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from std_msgs.msg import String


class PositionControllerNode(Node):
    def __init__(self):
        super().__init__('position_controller')
        self.declare_parameter('fmu_namespace', 'sim')
        ns = str(self.get_parameter('fmu_namespace').value).strip('/')
        if not ns or ns == 'fmu':
            raise ValueError('fmu_namespace must identify an isolated SITL vehicle')
        self.pub = self.create_publisher(
            TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', 10)
        self.complete = False
        self.create_subscription(
            TrajectorySetpoint, '/circle/reference_trajectory', self.reference_cb, 10)
        self.create_subscription(String, '/circle/experiment_state', self.phase_cb, 10)

    def reference_cb(self, msg):
        if not self.complete:
            self.pub.publish(msg)

    def phase_cb(self, msg):
        self.complete = msg.data == 'complete'
