#!/usr/bin/env python3
"""
Scan Sequence - Spiral pattern coverage with forward-facing orientation
Fixes:
  1. Waypoints offset by current position at scan start (EKF origin independence)
  2. Continuous setpoint streaming at 10Hz until waypoint reached
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


class ScanSequence(Node):
    def __init__(self):
        super().__init__('scan_sequence')

        # ========================================
        # PARAMETERS - Easy to tune
        # ========================================
        self.declare_parameter('scan_radius', 10.0)
        self.declare_parameter('waypoint_spacing', 2.5)
        self.declare_parameter('waypoint_threshold', 1.0)
        self.declare_parameter('spiral_turns', 3)

        # Service clients
        self.goto_client = self.create_client(Trigger, '/vehicle/goto_neu')
        self.param_client = self.create_client(
            SetParameters,
            '/vehicle_interface/set_parameters'
        )

        # Direct setpoint publisher for continuous streaming
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
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

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )

        self.get_logger().info('Waiting for services...')
        self.goto_client.wait_for_service(timeout_sec=10.0)
        self.param_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready!')

        self.get_logger().info('Waiting for position data...')
        while self.local_up == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f'✅ Position locked - altitude: {self.local_up:.2f}m')

    def pose_callback(self, msg):
        self.local_east = msg.pose.position.x
        self.local_north = msg.pose.position.y
        self.local_up = msg.pose.position.z

    def calculate_distance(self, north1, east1, north2, east2):
        return math.sqrt((north2 - north1) ** 2 + (east2 - east1) ** 2)

    def publish_setpoint(self, north, east, up, yaw_deg):
        """Publish a single setpoint to MAVROS local position topic"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = east   # ENU: x = East
        msg.pose.position.y = north  # ENU: y = North
        msg.pose.position.z = up

        yaw_enu_deg = 90.0 - yaw_deg
        yaw_rad = math.radians(yaw_enu_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)

        self.setpoint_pub.publish(msg)

    def set_goto_parameters(self, north, east, up, yaw):
        """Set goto parameters on vehicle_interface node"""
        params = [
            Parameter(name='goto_north',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=north)),
            Parameter(name='goto_east',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=east)),
            Parameter(name='goto_up',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=up)),
            Parameter(name='goto_yaw',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw)),
        ]

        request = SetParameters.Request()
        request.parameters = params
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            return future.result().results[0].successful
        return False

    def generate_spiral_waypoints(self, start_north, start_east):
        """
        Generate spiral waypoints offset from current position.
        Returns list of absolute (north, east, altitude, yaw) tuples.
        """
        radius  = self.get_parameter('scan_radius').value
        altitude = self.local_up
        spacing = self.get_parameter('waypoint_spacing').value
        turns   = self.get_parameter('spiral_turns').value

        total_angle = turns * 2 * math.pi
        num_points  = int((turns * 2 * math.pi * radius) / spacing)
        min_radius  = 2.0

        self.get_logger().info(f'Generating {num_points} waypoints for {turns} spiral turns')
        self.get_logger().info(f'Scan origin offset: N={start_north:.2f}, E={start_east:.2f}')

        waypoints = []

        for i in range(num_points):
            r     = min_radius + (radius - min_radius) * (i / num_points)
            theta = total_angle * (i / num_points)

            # Relative NED offset from scan origin
            rel_north = r * math.cos(theta)
            rel_east  = r * math.sin(theta)

            # Absolute position in EKF frame
            north = start_north + rel_north
            east  = start_east  + rel_east

            yaw_deg = math.degrees(theta + math.pi / 2)
            while yaw_deg >  180: yaw_deg -= 360
            while yaw_deg < -180: yaw_deg += 360

            waypoints.append((north, east, altitude, yaw_deg))

        # Final waypoint
        final_yaw = math.degrees(total_angle + math.pi / 2)
        while final_yaw >  180: final_yaw -= 360
        waypoints.append((start_north + radius, start_east, altitude, final_yaw))

        return waypoints

    def wait_for_waypoint(self, target_north, target_east, target_up, target_yaw):
        """Wait until waypoint reached, streaming setpoint continuously at 10Hz"""
        threshold  = self.get_parameter('waypoint_threshold').value
        timeout    = 30.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            # Spin to get fresh pose
            rclpy.spin_once(self, timeout_sec=0.05)

            # Stream setpoint continuously
            self.publish_setpoint(target_north, target_east, target_up, target_yaw)

            distance = self.calculate_distance(
                self.local_north, self.local_east,
                target_north, target_east
            )

            self.get_logger().info(
                f'Distance to waypoint: {distance:.2f}m (threshold: {threshold}m)',
                throttle_duration_sec=1.0
            )

            if distance < threshold:
                self.get_logger().info('✓ Waypoint reached')
                return True

            time.sleep(0.1)  # 10Hz

        self.get_logger().warn('⚠ Waypoint timeout')
        return False

    def execute(self):
        """Execute spiral scan pattern"""
        self.get_logger().info('🔍 Starting spiral scan sequence')

        # Capture current position as scan origin
        start_north = self.local_north
        start_east  = self.local_east
        self.get_logger().info(f'Scan origin: N={start_north:.2f}, E={start_east:.2f}')

        waypoints = self.generate_spiral_waypoints(start_north, start_east)
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')

        for i, (north, east, alt, yaw) in enumerate(waypoints):
            self.get_logger().info(
                f'Waypoint {i+1}/{len(waypoints)}: N={north:.1f}, E={east:.1f}, Yaw={yaw:.0f}°'
            )

            # Set params and trigger via vehicle_interface (first command)
            if not self.set_goto_parameters(north, east, alt, yaw):
                self.get_logger().error('Failed to set goto parameters')
                return False

            self.goto_client.call_async(Trigger.Request())

            # Wait — continuously streaming the same setpoint until reached
            if not self.wait_for_waypoint(north, east, alt, yaw):
                self.get_logger().error('Failed to reach waypoint - aborting scan')
                return False

            time.sleep(0.5)

        self.get_logger().info('✅ Spiral scan complete!')
        return True


def main():
    print("🔍 Starting scan sequence...")
    rclpy.init()

    node = ScanSequence()
    success = node.execute()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
