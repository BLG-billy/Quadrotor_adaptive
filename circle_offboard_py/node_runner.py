"""Run exactly one experiment node per process."""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException


def main(role=None, args=None):
    if role is None:
        role = sys.argv[1]
        args = sys.argv[2:]
    rclpy.init(args=args)
    if role == 'controller':
        from .l1_geometric_controller_node import L1GeometricControllerNode
        node = L1GeometricControllerNode()
    elif role == 'ctbr':
        from .ctbr.ctbr_controller_node import CTBRControllerNode
        node = CTBRControllerNode()
    elif role == 'position':
        from .position_controller_node import PositionControllerNode
        node = PositionControllerNode()
    elif role == 'trajectory':
        from .trajectory_node import CircleTrajectoryNode
        node = CircleTrajectoryNode()
    elif role == 'flight':
        from .flight_manager_node import FlightManagerNode
        node = FlightManagerNode()
    elif role == 'recorder':
        from .recorder_node import ExperimentRecorderNode
        node = ExperimentRecorderNode()
    else:
        raise ValueError(role)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if role == 'recorder':
            node.save_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def flight_main(args=None):
    main(role='flight', args=args)


def recorder_main(args=None):
    main(role='recorder', args=args)


if __name__ == '__main__':
    main()
