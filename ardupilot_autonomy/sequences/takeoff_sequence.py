# takeoff_sequence.py - both standalone AND as a service
# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import sys
import time

class TakeoffSequence(Node):
    def __init__(self):
        super().__init__('takeoff_sequence')

        # Declare parameter
        self.declare_parameter('takeoff_altitude', 5.0)

        # Create service clients
        self.lock_client = self.create_client(Trigger, '/vehicle/acquire_lock')
        self.guided_client = self.create_client(Trigger, '/vehicle/set_guided_mode')
        self.arm_client = self.create_client(Trigger, '/vehicle/arm')
        self.takeoff_client = self.create_client(Trigger, '/vehicle/takeoff')
        self.abort_client = self.create_client(Trigger, '/vehicle/abort')
        # Parameter client to set parameter on vehicle_interface node
        self.param_client = self.create_client(
            SetParameters,
            '/vehicle_interface/set_parameters'
        )
        self.release_client = self.create_client(Trigger, '/vehicle/release_lock')

    def execute(self):
        altitude = self.get_parameter('takeoff_altitude').value
        self.get_logger().info(f'🚀 Takeoff to {altitude}m')

        self.get_logger().info('Step 1: GUIDED mode')
        self.guided_client.call_async(Trigger.Request())
        time.sleep(2.0)

        self.get_logger().info('Step 2: Arm')
        self.arm_client.call_async(Trigger.Request())
        time.sleep(3.0)

        # Before takeoff
        altitude = self.get_parameter('takeoff_altitude').value
        self.set_vehicle_altitude_parameter(altitude)

        self.get_logger().info('Step 3: Takeoff')
        self.takeoff_client.call_async(Trigger.Request())

        self.get_logger().info('✅ Done')
        return True

    def set_vehicle_altitude_parameter(self, altitude):
        """Set takeoff_altitude parameter on vehicle_interface node"""
        param = Parameter()
        param.name = 'takeoff_altitude'
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=altitude
        )

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        return future.result().results[0].successful if future.result() else False


def main():
    rclpy.init()
    node = TakeoffSequence()

    # Run once and exit
    success = node.execute()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()