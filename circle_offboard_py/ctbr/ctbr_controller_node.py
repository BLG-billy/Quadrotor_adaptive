"""Publish collective thrust and body rates, leaving rate control to PX4."""
import numpy as np
from px4_msgs.msg import VehicleRatesSetpoint

from ..geometric_controller_node import GeometricControllerNode
from .ctbr_control import CTBRControlLaw


class CTBRControllerNode(GeometricControllerNode, CTBRControlLaw):
    def __init__(self):
        super().__init__(node_name='ctbr_controller')
        if self.get_parameter('enable_l1').value:
            raise ValueError('L1Quad wrench compensation requires the geometric wrench controller')
        for axis, gain in zip('xyz', (4.0, 4.0, 3.0)):
            self.declare_parameter(f'ctbr_attitude_gain_{axis}', gain)
        self.ctbr_attitude_gain = self.parameter_vector('ctbr_attitude_gain')
        if (not np.all(np.isfinite(self.ctbr_attitude_gain))
                or np.any(self.ctbr_attitude_gain <= 0.0)):
            raise ValueError('ctbr_attitude_gain must be finite and positive')

    def create_control_publishers(self):
        self.rates_pub = self.create_publisher(
            VehicleRatesSetpoint, self.fmu_prefix + '/in/vehicle_rates_setpoint', 10)

    def compute_and_publish(self, msg, dt):
        self.prepare_reference(msg)
        thrust, rates = self.ctbr_control(
            np.asarray(msg.position), np.asarray(msg.velocity),
            np.asarray(msg.acceleration), dt)
        thrust += self.current_disturbance()
        thrust_command = self.actuator_map.collective(thrust)
        output = VehicleRatesSetpoint()
        output.timestamp = self.timestamp_us()
        output.roll, output.pitch, output.yaw = map(float, rates)
        output.thrust_body = [0.0, 0.0, -thrust_command]
        output.reset_integral = False
        self.rates_pub.publish(output)
        self.last_command_timestamp = output.timestamp


def main(args=None):
    from ..node_runner import main as run_node
    run_node(role='ctbr', args=args)
