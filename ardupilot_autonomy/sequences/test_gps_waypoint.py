#!/usr/bin/env python3
"""Test GPS waypoint - 5m north, 5m altitude"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import time


class WaypointTestGPS(Node):
    def __init__(self):
        super().__init__('waypoint_test_gps')

        # Service clients
        self.goto_client = self.create_client(Trigger, '/vehicle/goto_position')
        self.param_client = self.create_client(SetParameters, '/vehicle_interface/set_parameters')

        # Wait for services
        while not self.goto_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for goto_position service...')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')

        self.get_logger().info('✅ GPS waypoint test ready')

    def set_param(self, name, value):
        """Set parameter on vehicle_interface"""
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)

        request = SetParameters.Request()
        request.parameters = [param]

        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def execute(self):
        # Set waypoint: 5m north, 0m east, 5m altitude
        self.get_logger().info('Setting parameters on /vehicle_interface...')
        self.set_param('goto_north', 0.0)
        self.set_param('goto_east', 0.0)
        self.set_param('goto_up', 5.0)
        self.set_param('goto_yaw', 225.0)

        self.get_logger().info('📍 Commanding GPS waypoint: 5m North, 5m altitude')

        # Call service
        request = Trigger.Request()
        future = self.goto_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('✅ GPS waypoint commanded')
        else:
            self.get_logger().error('❌ GPS waypoint failed')


def main():
    rclpy.init()
    node = WaypointTestGPS()

    time.sleep(1)
    node.execute()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()