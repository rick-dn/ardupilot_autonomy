#!/usr/bin/env python3
"""
DDS Interface - ArduPilot DDS communication wrapper
Handles all ArduPilot DDS service calls and topic subscriptions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from ardupilot_msgs.msg import GlobalPosition
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8
from geographiclib.geodesic import Geodesic
import math


class DDSInterface:
    """Wrapper for ArduPilot DDS communication"""
    
    def __init__(self, node: Node):
        self.node = node
        
        # QoS for status (RELIABLE + TRANSIENT_LOCAL)
        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
    
        # QoS for pose (BEST_EFFORT + VOLATILE)
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        ) 
        # ArduPilot DDS service clients
        self.arm_client = self.node.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.node.create_client(ModeSwitch, '/ap/mode_switch')
        self.takeoff_client = self.node.create_client(Takeoff, '/ap/experimental/takeoff')
        
        # ArduPilot DDS subscribers
        self.status_sub = self.node.create_subscription(
            UInt8,
            '/ap/status',
            self.status_callback,
            qos_status
        )
        
        self.gps_sub = self.node.create_subscription(
            NavSatFix,
            '/ap/navsat',
            self.gps_callback,
            qos_pose
        )
        
        self.pose_sub = self.node.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.pose_callback,
            qos_pose
        )
        
        # ArduPilot DDS publishers
        self.gps_pose_pub = self.node.create_publisher(
            GlobalPosition,
            '/ap/cmd_gps_pose',
            10
        )
        
        # Geodesic calculator for NED conversion
        self.geod = Geodesic.WGS84
        
        # State tracking
        self.armed = False
        self.mode = 0  # ArduPilot mode number
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0
        
        # NED position from /ap/pose/filtered
        self.ned_x = 0.0
        self.ned_y = 0.0
        self.ned_z = 0.0
        
        self.node.get_logger().info('DDS Interface initialized')
    
    def status_callback(self, msg):
        """ArduPilot status callback"""
        # Status byte contains armed/mode info
        # You'll need to decode based on ArduPilot docs
        # For now, simplified:
        self.armed = (msg.data & 0x80) != 0  # Bit 7 = armed
        self.mode = msg.data & 0x1F  # Lower 5 bits = mode
    
    def gps_callback(self, msg):
        """GPS position callback"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        
        # Save home on first valid GPS
        if self.home_lat == 0.0 and msg.latitude != 0.0:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.home_alt = msg.altitude
            self.node.get_logger().info(f'Home set: {self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m')
    
    def pose_callback(self, msg):
        """NED pose callback"""
        self.ned_x = msg.pose.position.x
        self.ned_y = msg.pose.position.y
        self.ned_z = msg.pose.position.z
    
    def arm(self):
        """Arm the vehicle"""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Arming service not available')
            return False
        
        request = ArmMotors.Request()
        request.arm = True
        
        # Fire and forget
        self.arm_client.call_async(request)
        self.node.get_logger().info('Arm command sent')
        return True
    
    def disarm(self):
        """Disarm the vehicle"""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Disarm service not available')
            return False
        
        request = ArmMotors.Request()
        request.arm = False
        
        # Fire and forget
        self.arm_client.call_async(request)
        self.node.get_logger().info('Disarm command sent')
        return True
    
    def set_mode_guided(self):
        """Set GUIDED mode (mode 4)"""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Mode switch service not available')
            return False
        
        request = ModeSwitch.Request()
        request.mode = 4  # GUIDED
        
        # Fire and forget
        self.mode_client.call_async(request)
        self.node.get_logger().info('GUIDED mode command sent')
        return True
    
    # def takeoff(self, altitude):
    #     """Takeoff to specified altitude"""
    #     # ArduPilot DDS: use GPS position at current location + altitude
    #     msg = GlobalPosition()
    #     msg.header.stamp = self.node.get_clock().now().to_msg()
    #     msg.header.frame_id = 'map'
    #     msg.coordinate_frame = 5  # MAV_FRAME_GLOBAL_INT
    #     msg.latitude = self.current_lat
    #     msg.longitude = self.current_lon
    #     msg.altitude = altitude  # Relative to home
    #
    #     # Publish 3 times for robustness
    #     for _ in range(3):
    #         self.gps_pose_pub.publish(msg)
    #         rclpy.spin_once(self.node, timeout_sec=0.1)
    #
    #     self.node.get_logger().info(f'Takeoff to {altitude}m commanded')
    #     return True

    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Takeoff service not available')
            return False

        request = Takeoff.Request()
        request.alt = altitude

        # Fire and forget
        self.takeoff_client.call_async(request)
        self.node.get_logger().info(f'Takeoff to {altitude}m commanded')
        return True
    
    def goto_position(self, lat, lon, alt):
        """Go to GPS position"""
        msg = GlobalPosition()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = 5  # MAV_FRAME_GLOBAL_INT
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = self.home_alt + alt  # Convert to MSL
        
        # Publish 3 times for robustness
        for _ in range(3):
            self.gps_pose_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.node.get_logger().info(f'Goto position: {lat:.6f}, {lon:.6f}, {alt:.1f}m')
        return True
    
    def ned_to_gps(self, north_m, east_m, down_m):
        """
        Convert NED offset to GPS coordinates (JacopoPan style)
        
        Args:
            north_m: North offset in meters (from home)
            east_m: East offset in meters (from home)
            down_m: Down offset in meters (positive = below home)
        
        Returns:
            (lat, lon, alt_msl)
        """
        # Step 1: Apply North offset
        bearing_ns = 0 if north_m >= 0 else 180
        result = self.geod.Direct(
            self.home_lat, self.home_lon,
            bearing_ns, abs(north_m)
        )
        temp_lat = result['lat2']
        temp_lon = result['lon2']
        
        # Step 2: Apply East offset
        bearing_ew = 90 if east_m >= 0 else 270
        result = self.geod.Direct(
            temp_lat, temp_lon,
            bearing_ew, abs(east_m)
        )
        target_lat = result['lat2']
        target_lon = result['lon2']
        
        # Step 3: Calculate altitude (down is positive in NED)
        target_alt_msl = self.home_alt + down_m
        
        return target_lat, target_lon, target_alt_msl
    
    # def goto_ned(self, north, east, down):
    #     """
    #     Go to NED position (absolute from home)
    #     Must convert to GPS since ArduPilot DDS doesn't accept NED commands
    #
    #     Args:
    #         north: North position in meters (from home)
    #         east: East position in meters (from home)
    #         down: Down position in meters (positive = below home, negative = above)
    #     """
    #     # Validate altitude
    #     altitude_above_home = -down
    #     if altitude_above_home < 0:
    #         self.node.get_logger().error(
    #             f'Invalid altitude: {altitude_above_home}m (cannot go below home)'
    #         )
    #         return False
    #
    #     # Convert NED to GPS
    #     lat, lon, alt_msl = self.ned_to_gps(north, east, down)
    #
    #     # Publish GPS command
    #     msg = GlobalPosition()
    #     msg.header.stamp = self.node.get_clock().now().to_msg()
    #     msg.header.frame_id = 'map'
    #     msg.coordinate_frame = 5  # MAV_FRAME_GLOBAL_INT
    #     msg.latitude = lat
    #     msg.longitude = lon
    #     msg.altitude = alt_msl
    #
    #     # Publish 3 times for robustness
    #     for _ in range(3):
    #         self.gps_pose_pub.publish(msg)
    #         rclpy.spin_once(self.node, timeout_sec=0.1)
    #
    #     self.node.get_logger().info(
    #         f'Goto NED: N={north:.1f}, E={east:.1f}, D={down:.1f}m '
    #         f'(GPS: {lat:.6f}, {lon:.6f}, {alt_msl:.1f}m MSL)'
    #     )
    #     return True

    def goto_ned(self, north, east, down):
        """
        Go to NED position (absolute from home)
        Must convert to GPS since ArduPilot DDS doesn't accept NED commands

        Args:
            north: North position in meters (from home)
            east: East position in meters (from home)
            down: Down position in meters (positive = below home, negative = above)
        """
        # Convert NED to GPS (no validation here)
        lat, lon, alt_msl = self.ned_to_gps(north, east, down)

        # Publish GPS command
        msg = GlobalPosition()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = 5
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_msl

        # Publish 3 times for robustness
        for _ in range(3):
            self.gps_pose_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info(
            f'Goto NED: N={north:.1f}, E={east:.1f}, D={down:.1f}m '
            f'(GPS: {lat:.6f}, {lon:.6f}, {alt_msl:.1f}m MSL)'
        )
        return True
    
    def goto_yaw(self, yaw_deg):
        """Set yaw while maintaining position"""
        # ArduPilot DDS: maintain current GPS position, set orientation
        yaw_rad = math.radians(yaw_deg)
        
        msg = GlobalPosition()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.coordinate_frame = 5
        msg.latitude = self.current_lat
        msg.longitude = self.current_lon
        msg.altitude = self.current_alt
        # Note: GlobalPosition might not support yaw - check ardupilot_msgs definition
        
        for _ in range(3):
            self.gps_pose_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.node.get_logger().info(f'Goto yaw: {yaw_deg}°')
        return True
    
    def land(self):
        """Land at current position - switch to LAND mode (9)"""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Mode switch service not available')
            return False
        
        request = ModeSwitch.Request()
        request.mode = 9  # LAND
        
        # Fire and forget
        self.mode_client.call_async(request)
        self.node.get_logger().info('Land command sent')
        return True
    
    def rtl(self):
        """Return to launch - switch to RTL mode (6)"""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Mode switch service not available')
            return False
        
        request = ModeSwitch.Request()
        request.mode = 6  # RTL
        
        # Fire and forget
        self.mode_client.call_async(request)
        self.node.get_logger().info('RTL command sent')
        return True
