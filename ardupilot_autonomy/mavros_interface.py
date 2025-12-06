#!/usr/bin/env python3
"""
MAVROS Interface - MAVROS communication wrapper
Handles all MAVROS service calls and topic subscriptions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from geographiclib.geodesic import Geodesic

import math


class MavrosInterface:
    """Wrapper for MAVROS communication"""
    
    def __init__(self, node: Node):
        self.node = node

        # Geodesic calculator for lat/lon conversions
        self.geod = Geodesic.WGS84
        
        # QoS profile for subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for subscribers
        qos_profile_global = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Changed from RELIABLE
            durability=DurabilityPolicy.VOLATILE,  # Changed from TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # MAVROS service clients
        self.arm_client = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.node.create_client(CommandTOL, '/mavros/cmd/land')
        
        # MAVROS subscribers
        self.state_sub = self.node.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        self.gps_sub = self.node.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_global
        )
        
        # MAVROS publisher
        self.setpoint_pub = self.node.create_publisher(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            10
        )

        # MAVROS publisher for local NED positions
        self.setpoint_local_pub = self.node.create_publisher(
        PoseStamped,
        '/mavros/setpoint_position/local',
        10
        )

        # MAVROS publisher for velocity control (body frame only)
        self.vel_pub = self.node.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )

        # MAVROS publisher for acceleration control
        self.accel_pub = self.node.create_publisher(
            Vector3Stamped,
            '/mavros/setpoint_accel/accel',
            10
        )
        
        # State tracking
        self.armed = False
        self.mode = ""
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0
        self.home_lat = 0.0
        self.home_lon = 0.0
        self.home_alt = 0.0

        # Track current local position
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.local_yaw = 0.0

        # Velocity control state
        self.velocity_control_active = False
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_vyaw = 0.0

        # Acceleration control state
        self.accel_control_active = False
        self.target_ax = 0.0
        self.target_ay = 0.0
        self.target_az = 0.0

        # Timer for continuous velocity publishing (10Hz)
        self.velocity_timer = self.node.create_timer(0.1, self.publish_velocity)
        self.accel_timer = self.node.create_timer(0.1, self.publish_accel)
        
        self.node.get_logger().info('MAVROS Interface initialized')
    
    def state_callback(self, msg):
        """MAVROS state callback"""
        # self.node.get_logger().info(f'📥 mavros_interface state_callback: msg.mode="{msg.mode}", msg.armed={msg.armed}')
        self.armed = msg.armed
        self.mode = msg.mode
        self.system_status = msg.system_status  # ADD THIS
        # self.node.get_logger().info(f'📥 After assignment: self.mode="{self.mode}"')

        # self.node.get_logger().info(f'🔔 State callback fired: armed={self.armed}, mode={self.mode}')

    def ned_to_gps(self, north, east):
        """
        Convert NED offsets to GPS coordinates
        Args:
            north: North offset in meters from home
            east: East offset in meters from home
        Returns:
            (latitude, longitude)
        """
        # North-South offset (bearing 0° for north, 180° for south)
        bearing_ns = 0 if north >= 0 else 180
        result = self.geod.Direct(
            self.home_lat, self.home_lon, bearing_ns, abs(north)
        )
        temp_lat = result['lat2']
        temp_lon = result['lon2']

        # East-West offset (bearing 90° for east, 270° for west)
        bearing_ew = 90 if east >= 0 else 270
        result = self.geod.Direct(
            temp_lat, temp_lon, bearing_ew, abs(east)
        )
        final_lat = result['lat2']
        final_lon = result['lon2']

        return final_lat, final_lon
    
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
            self.node.get_logger().info(f'Home set: {self.home_lat:.6f}, {self.home_lon:.6f}')
    
    def arm(self):
        """Arm the vehicle"""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Arming service not available')
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        # Fire and forget - JacopoPan style
        self.arm_client.call_async(request)
        self.node.get_logger().info('Arm command sent')
        return True
    
    def disarm(self):
        """Disarm the vehicle"""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Disarm service not available')
            return False
        
        request = CommandBool.Request()
        request.value = False
        
        # Fire and forget - JacopoPan style
        self.arm_client.call_async(request)
        self.node.get_logger().info('Disarm command sent')
        return True

    def set_mode(self, mode_name):
        """Set flight mode (fire-and-forget)"""
        request = SetMode.Request()
        request.custom_mode = mode_name
        self.mode_client.call_async(request)
        self.node.get_logger().info(f'{mode_name} mode command sent')
        return True

    def set_mode_guided(self):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            return False
        
        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        
        # Fire and forget - JacopoPan style
        self.mode_client.call_async(request)
        self.node.get_logger().info('GUIDED mode command sent')
        return True
    
    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Takeoff service not available')
            return False
        
        request = CommandTOL.Request()
        request.altitude = altitude
        
        # Fire and forget - JacopoPan style
        self.takeoff_client.call_async(request)
        self.node.get_logger().info(f'Takeoff to {altitude}m commanded')
        return True
    
    def goto_position(self, north, east, alt, yaw_deg):
        """Go to GPS position"""

        # Convert NED offsets to GPS
        lat, lon = self.ned_to_gps(north, east)

        msg = GeoPoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon
        msg.pose.position.altitude = self.home_alt + alt

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        
        # Publish 3 times for robustness
        for _ in range(3):
            self.setpoint_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self.node.get_logger().info(f'Goto position: {lat:.6f}, {lon:.6f}, {alt:.1f}m, Yaw={yaw_deg}°')
        return True

    def goto_neu(self, north, east, up, yaw_deg):
        """Go to NED position - publish directly to local frame"""
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = east
        msg.pose.position.y = north
        msg.pose.position.z = up

        # Set orientation (yaw) - always set, no checks
        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)

        # Publish 3 times for robustness
        for _ in range(3):
            self.setpoint_local_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info(f'Goto NED: N={north:.1f}, E={east:.1f}, Up={up:.1f}m, Yaw={yaw_deg}°')
        return True

    def local_pose_callback(self, msg):
        """Local position callback"""
        self.local_x = msg.pose.position.x
        self.local_y = msg.pose.position.y
        self.local_z = msg.pose.position.z

        # Extract yaw from quaternion
        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        self.local_yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def set_velocity(self, vx, vy, vz, yaw_rate):
        """
        Set velocity in body frame
        Args:
            vx: Forward velocity (m/s)
            vy: Right velocity (m/s)
            vz: Down velocity (m/s, positive = down)
            yaw_rate: Yaw rate (rad/s)
        """
        self.target_vx = vx
        self.target_vy = vy
        self.target_vz = vz
        self.target_vyaw = yaw_rate
        self.velocity_control_active = True

        self.node.get_logger().info(
            f'Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}'
        )

    def stop_velocity(self):
        """Stop velocity control"""
        self.velocity_control_active = False
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_vyaw = 0.0
        self.node.get_logger().info('Velocity control stopped')

    def publish_velocity(self):
        """Publish velocity setpoint at 10Hz"""
        if not self.velocity_control_active:
            return

        msg = Twist()
        msg.linear.x = self.target_vx
        msg.linear.y = self.target_vy
        msg.linear.z = self.target_vz
        msg.angular.z = self.target_vyaw
        self.vel_pub.publish(msg)

    def set_acceleration(self, ax, ay, az):
        """
        Set acceleration in NED frame
        Args:
            ax: North acceleration (m/s²)
            ay: East acceleration (m/s²)
            az: Down acceleration (m/s², positive = down)
        """
        self.target_ax = ax
        self.target_ay = ay
        self.target_az = az
        self.accel_control_active = True

        self.node.get_logger().info(
            f'Acceleration: ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}'
        )

    def stop_acceleration(self):
        """Stop acceleration control"""
        self.accel_control_active = False
        self.target_ax = 0.0
        self.target_ay = 0.0
        self.target_az = 0.0
        self.node.get_logger().info('Acceleration control stopped')

    def publish_accel(self):
        """Publish acceleration setpoint at 10Hz"""
        if not self.accel_control_active:
            return

        msg = Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = self.target_ax
        msg.vector.y = self.target_ay
        msg.vector.z = self.target_az
        self.accel_pub.publish(msg)
    
    def land(self):
        """Land at current position"""
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Land service not available')
            return False
        
        request = CommandTOL.Request()
        
        # Fire and forget - JacopoPan style
        self.land_client.call_async(request)
        self.node.get_logger().info('Land command sent')
        return True
    
    def rtl(self):
        """Return to launch"""
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error('Set mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = "RTL"
        
        # Fire and forget - JacopoPan style
        self.mode_client.call_async(request)
        self.node.get_logger().info('RTL mode set')
        return True
