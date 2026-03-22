#!/usr/bin/env python3
"""
Scan Sequence GPS - Spiral pattern using GPS setpoints (for old firmware)
Identical to scan_sequence but calls /vehicle/goto_position instead of /vehicle/goto_neu
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
import math
import time
import sys


class ScanSequenceGPS(Node):
    def __init__(self):
        super().__init__('scan_sequence_gps')

        # Parameters
        self.declare_parameter('scan_radius', 10.0)
        self.declare_parameter('waypoint_spacing', 2.5)
        self.declare_parameter('waypoint_threshold', 1.0)
        self.declare_parameter('spiral_turns', 3)

        # Service client - ONLY CHANGE: goto_position instead of goto_neu
        self.goto_client = self.create_client(Trigger, '/vehicle/goto_position')

        # Parameter client
        self.param_client = self.create_client(
            SetParameters,
            '/vehicle_interface/set_parameters'
        )

        self.local_north = 0.0
        self.local_east = 0.0
        self.local_up = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to local pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        # Wait for services
        self.get_logger().info('Waiting for services...')
        self.goto_client.wait_for_service(timeout_sec=10.0)
        self.param_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready!')

        # Wait for position
        self.get_logger().info('Waiting for position data...')
        while self.local_up == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'✅ Position locked - altitude: {self.local_up:.2f}m')


    def calculate_distance(self, north1, east1, north2, east2):
        return math.sqrt((north2 - north1) ** 2 + (east2 - east1) ** 2)


    def pose_callback(self, msg):
        self.local_east = msg.pose.position.x
        self.local_north = msg.pose.position.y
        self.local_up = msg.pose.position.z


    def set_goto_parameters(self, north, east, up, yaw):
        """Set goto parameters - VI converts to GPS internally"""
        params = [
            Parameter(
                name='goto_north',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=north)
            ),
            Parameter(
                name='goto_east',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=east)
            ),
            Parameter(
                name='goto_up',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=up)
            ),
            Parameter(
                name='goto_yaw',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw)
            ),
        ]

        request = SetParameters.Request()
        request.parameters = params

        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            result = future.result().results[0]
            return result.successful
        return False


    def generate_spiral_waypoints(self):
        """Generate spiral waypoints in NED frame"""
        radius = self.get_parameter('scan_radius').value
        altitude = self.local_up
        spacing = self.get_parameter('waypoint_spacing').value
        turns = self.get_parameter('spiral_turns').value

        waypoints = []
        total_angle = turns * 2 * math.pi
        num_points = int((turns * 2 * math.pi * radius) / spacing)

        self.get_logger().info(f'Generating {num_points} waypoints for {turns} spiral turns')

        min_radius = 2.0

        for i in range(num_points):
            r = min_radius + (radius - min_radius) * (i / num_points)
            theta = total_angle * (i / num_points)

            north = r * math.cos(theta)
            east = r * math.sin(theta)

            yaw_rad = theta + math.pi / 2
            yaw_deg = math.degrees(yaw_rad)

            while yaw_deg > 180:
                yaw_deg -= 360
            while yaw_deg < -180:
                yaw_deg += 360

            waypoints.append((north, east, altitude, yaw_deg))

        # Final waypoint
        final_yaw = math.degrees(total_angle + math.pi / 2)
        while final_yaw > 180:
            final_yaw -= 360
        waypoints.append((radius, 0.0, altitude, final_yaw))

        return waypoints


    def wait_for_waypoint(self, target_north, target_east):
        threshold = self.get_parameter('waypoint_threshold').value
        timeout = 30.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            distance = self.calculate_distance(
                self.local_north, self.local_east,
                target_north, target_east
            )

            self.get_logger().info(
                f'Distance to waypoint: {distance:.2f}m (threshold: {threshold}m)',
                throttle_duration_sec=1.0
            )

            if distance < threshold:
                self.get_logger().info(f'✓ Waypoint reached')
                return True

            time.sleep(0.2)

        self.get_logger().warn('⚠ Waypoint timeout')
        return False


    def execute(self):
        self.get_logger().info('🔍 Starting GPS spiral scan sequence')

        waypoints = self.generate_spiral_waypoints()
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')

        for i, (north, east, alt, yaw) in enumerate(waypoints):
            self.get_logger().info(
                f'Waypoint {i + 1}/{len(waypoints)}: N={north:.1f}, E={east:.1f}, Yaw={yaw:.0f}°'
            )

            if not self.set_goto_parameters(north, east, alt, yaw):
                self.get_logger().error('Failed to set goto parameters')
                return False

            self.goto_client.call_async(Trigger.Request())

            if not self.wait_for_waypoint(north, east):
                self.get_logger().error('Failed to reach waypoint - aborting scan')
                return False

            time.sleep(0.5)

        self.get_logger().info('✅ GPS spiral scan complete!')
        return True


def main():
    print("🔍 Starting GPS scan sequence...")
    rclpy.init()

    node = ScanSequenceGPS()
    success = node.execute()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()