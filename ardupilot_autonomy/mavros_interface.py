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
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from geographiclib.geodesic import Geodesic
from mavros_msgs.msg import HomePosition, PositionTarget, RCIn, GPSRAW, VfrHud, StatusText
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

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

        # Create specific QoS for home (match MAVROS publisher)
        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # MAVROS service clients
        self.arm_client = self.node.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.node.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.node.create_client(CommandTOL, '/mavros/cmd/land')

        # MAVROS subscribers
        self.home_sub = self.node.create_subscription(
            HomePosition,
            '/mavros/home_position/home',
            self.home_callback,
            home_qos
        )

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

        self.velocity_sub = self.node.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_body',
            self.velocity_callback,
            qos_profile_global
        )

        self.imu_sub = self.node.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile_global
        )

        self.rc_sub = self.node.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            qos_profile_global
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile_global
        )

        self.battery_sub = self.node.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile_global
        )

        self.gps_vel_sub = self.node.create_subscription(
            TwistStamped,
            '/mavros/global_position/raw/gps_vel',
            self.gps_vel_callback,
            qos_profile_global
        )

        self.gpsraw_sub = self.node.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.gpsraw_callback,
            qos_profile_global
        )

        self.compass_hdg_sub = self.node.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.compass_hdg_callback,
            qos_profile_global
        )

        self.vfr_hud_sub = self.node.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_hud_callback,
            qos_profile_global
        )

        self.statustext_sub = self.node.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.statustext_callback,
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

        # MAVROS publisher for body-frame position setpoints
        self.setpoint_raw_local_pub = self.node.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
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

        # Current body-frame velocity (from /mavros/local_position/velocity_body)
        # RFU convention: right/forward/up, matching goto_body()
        self.vel_forward = 0.0
        self.vel_right = 0.0
        self.vel_up = 0.0
        self.vel_yaw_rate = 0.0

        # IMU status (orientation is ENU quaternion; gyro/accel are body-frame RFU)
        self.imu_qw = 1.0
        self.imu_qx = 0.0
        self.imu_qy = 0.0
        self.imu_qz = 0.0
        self.imu_gyro_forward = 0.0
        self.imu_gyro_right = 0.0
        self.imu_gyro_up = 0.0
        self.imu_accel_forward = 0.0
        self.imu_accel_right = 0.0
        self.imu_accel_up = 0.0

        # RC status
        self.rc_channels = []
        self.rc_rssi = 0

        # Combined local odometry: pose is ENU (east/north/up), twist is body-frame RFU
        self.odom_east = 0.0
        self.odom_north = 0.0
        self.odom_up = 0.0
        self.odom_qw = 1.0
        self.odom_qx = 0.0
        self.odom_qy = 0.0
        self.odom_qz = 0.0
        self.odom_vel_forward = 0.0
        self.odom_vel_right = 0.0
        self.odom_vel_up = 0.0
        self.odom_yaw_rate = 0.0

        # Battery status
        self.battery_voltage = 0.0
        self.battery_current = 0.0
        self.battery_percentage = 0.0

        # GPS-derived velocity (ENU)
        self.gps_vel_east = 0.0
        self.gps_vel_north = 0.0
        self.gps_vel_up = 0.0

        # Raw GPS status
        self.gps_fix_type = 0
        self.gps_satellites_visible = 0
        self.gps_eph = 0
        self.gps_epv = 0

        # Heading, CCW positive from East (converted from MAVLink's CW-from-North compass bearing)
        self.heading_deg = 0.0

        # VFR HUD
        self.vfr_airspeed = 0.0
        self.vfr_groundspeed = 0.0
        self.vfr_heading_deg = 0.0
        self.vfr_throttle = 0.0
        self.vfr_altitude = 0.0
        self.vfr_climb = 0.0

        # Last STATUSTEXT received
        self.last_statustext = ""
        self.last_statustext_severity = 0

        # Track current local position
        self.local_x = 0.0
        self.local_y = 0.0
        self.local_z = 0.0
        self.local_yaw = 0.0

        # Velocity control state
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_vyaw = 0.0

        # Acceleration control state
        self.target_ax = 0.0
        self.target_ay = 0.0
        self.target_az = 0.0

        # Setpoint streaming timers are created on demand by set_velocity() /
        # set_acceleration() and destroyed by their stop_*() counterparts.
        # Nothing is published until something is actually commanded - a
        # standing stream of zero setpoints would otherwise hold the vehicle
        # in GUIDED and override commands issued from elsewhere.
        self.velocity_timer = None
        self.accel_timer = None

        self.node.get_logger().info('MAVROS Interface initialized')

    def state_callback(self, msg):
        """MAVROS state callback"""
        # self.node.get_logger().info(f'📥 mavros_interface state_callback: msg.mode="{msg.mode}", msg.armed={msg.armed}')
        self.armed = msg.armed
        self.mode = msg.mode
        self.system_status = msg.system_status  # ADD THIS
        # self.node.get_logger().info(f'📥 After assignment: self.mode="{self.mode}"')

        # self.node.get_logger().info(f'🔔 State callback fired: armed={self.armed}, mode={self.mode}')

    def home_callback(self, msg):
        """Get ArduPilot's actual home position"""
        # if self.home_lat == 0.0:
        self.home_lat = msg.geo.latitude
        self.home_lon = msg.geo.longitude
        self.home_alt = msg.geo.altitude
        self.node.get_logger().info(
            f'✅ Home: {self.home_lat:.6f}, {self.home_lon:.6f}, {self.home_alt:.1f}m AMSL'
        )

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

        # # Save home on first valid GPS
        # if self.home_lat == 0.0 and msg.latitude != 0.0:
        #     self.home_lat = msg.latitude
        #     self.home_lon = msg.longitude
        #     self.home_alt = msg.altitude
        #     self.node.get_logger().info(f'Home set: {self.home_lat:.6f}, {self.home_lon:.6f}')

    def velocity_callback(self, msg):
        """
        Current body-frame velocity callback.
        MAVROS publishes this in base_link (FLU: x=forward, y=left, z=up);
        convert to RFU to match goto_body()'s convention (right = -left).
        """
        self.vel_forward = msg.twist.linear.x
        self.vel_right = -msg.twist.linear.y
        self.vel_up = msg.twist.linear.z
        self.vel_yaw_rate = msg.twist.angular.z

    def imu_callback(self, msg):
        """
        IMU status callback. Orientation is ENU (matches command quaternions).
        Angular velocity/linear acceleration are body-frame FLU from MAVROS;
        convert to RFU to match goto_body()'s convention (right = -left).
        """
        self.imu_qw = msg.orientation.w
        self.imu_qx = msg.orientation.x
        self.imu_qy = msg.orientation.y
        self.imu_qz = msg.orientation.z
        self.imu_gyro_forward = msg.angular_velocity.x
        self.imu_gyro_right = -msg.angular_velocity.y
        self.imu_gyro_up = msg.angular_velocity.z
        self.imu_accel_forward = msg.linear_acceleration.x
        self.imu_accel_right = -msg.linear_acceleration.y
        self.imu_accel_up = msg.linear_acceleration.z

    def rc_callback(self, msg):
        """RC input status callback"""
        self.rc_channels = list(msg.channels)
        self.rc_rssi = msg.rssi

    def odom_callback(self, msg):
        """
        Combined local odometry callback.
        Pose is ENU (map frame). Twist is body-frame FLU from MAVROS;
        convert to RFU to match goto_body()'s convention (right = -left).
        """
        self.odom_east = msg.pose.pose.position.x
        self.odom_north = msg.pose.pose.position.y
        self.odom_up = msg.pose.pose.position.z
        self.odom_qw = msg.pose.pose.orientation.w
        self.odom_qx = msg.pose.pose.orientation.x
        self.odom_qy = msg.pose.pose.orientation.y
        self.odom_qz = msg.pose.pose.orientation.z
        self.odom_vel_forward = msg.twist.twist.linear.x
        self.odom_vel_right = -msg.twist.twist.linear.y
        self.odom_vel_up = msg.twist.twist.linear.z
        self.odom_yaw_rate = msg.twist.twist.angular.z

    def battery_callback(self, msg):
        """Battery status callback"""
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current
        self.battery_percentage = msg.percentage

    def gps_vel_callback(self, msg):
        """GPS-derived velocity (ENU) callback"""
        self.gps_vel_east = msg.twist.linear.x
        self.gps_vel_north = msg.twist.linear.y
        self.gps_vel_up = msg.twist.linear.z

    def gpsraw_callback(self, msg):
        """Raw GPS status callback"""
        self.gps_fix_type = msg.fix_type
        self.gps_satellites_visible = msg.satellites_visible
        self.gps_eph = msg.eph
        self.gps_epv = msg.epv

    def compass_hdg_callback(self, msg):
        """
        Compass heading callback. MAVLink reports this as a bearing
        (deg, CW from North); convert to yaw_deg convention (CCW from East).
        """
        self.heading_deg = (90.0 - msg.data) % 360.0

    def vfr_hud_callback(self, msg):
        """
        VFR HUD callback. msg.heading is a bearing (deg, CW from North),
        same as compass_hdg; convert to yaw_deg convention (CCW from East).
        """
        self.vfr_airspeed = msg.airspeed
        self.vfr_groundspeed = msg.groundspeed
        self.vfr_heading_deg = (90.0 - msg.heading) % 360.0
        self.vfr_throttle = msg.throttle
        self.vfr_altitude = msg.altitude
        self.vfr_climb = msg.climb

    def statustext_callback(self, msg):
        """FC STATUSTEXT callback"""
        self.last_statustext = msg.text
        self.last_statustext_severity = msg.severity
        self.node.get_logger().info(f'FC STATUSTEXT [{msg.severity}]: {msg.text}')

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

    def goto_global(self, lon, lat, alt, yaw_deg):
        """Go to GPS position (lon, lat, alt relative to home, yaw CCW positive)"""

        target_amsl = self.home_alt + alt

        msg = GeoPoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.pose.position.latitude = lat
        msg.pose.position.longitude = lon

        msg.pose.position.altitude = target_amsl

        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)

        self.node.get_logger().info(f'Sending AMSL: {target_amsl:.1f}m (home={self.home_alt:.1f}m + rel={alt:.1f}m)')

        # Publish 3 times for robustness
        for _ in range(3):
            self.setpoint_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info(f'Goto position: lon={lon:.6f}, lat={lat:.6f}, alt={alt:.1f}m, AMSL={target_amsl:.1f}m, Yaw={yaw_deg}°')
        return True

    def goto_local(self, east, north, up, yaw_deg):
        """Go to ENU position - publish directly to local frame"""
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

        self.node.get_logger().info(f'Goto ENU: E={east:.1f}, N={north:.1f}, Up={up:.1f}m, Yaw={yaw_deg}°')
        return True

    def goto_body(self, right, forward, up, yaw_deg):
        """
        Go to a position offset in the body frame (relative to current pose).
        External convention matches ENU: right/forward/up, yaw CCW positive.
        Published as MAV_FRAME_BODY_OFFSET_NED via /mavros/setpoint_raw/local,
        which internally expects baselink axes (x=forward, y=left, z=up).
        """
        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = PositionTarget.FRAME_BODY_OFFSET_NED

        msg.position.x = forward
        msg.position.y = -right
        msg.position.z = up

        yaw_rad = math.radians(yaw_deg)
        msg.yaw = yaw_rad

        # Ignore velocity, acceleration and yaw-rate - position + yaw only
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # Publish 3 times for robustness
        for _ in range(3):
            self.setpoint_raw_local_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info(f'Goto body: R={right:.1f}, F={forward:.1f}, Up={up:.1f}m, Yaw={yaw_deg}°')
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

    def set_velocity(self, east, north, up, yaw_rate):
        """
        Set velocity in ENU frame (matches goto_local's convention)
        Args:
            east: East velocity (m/s)
            north: North velocity (m/s)
            up: Up velocity (m/s)
            yaw_rate: Yaw rate, CCW positive (rad/s)
        """
        self.target_vx = east
        self.target_vy = north
        self.target_vz = up
        self.target_vyaw = yaw_rate

        if self.velocity_timer is None:
            self.velocity_timer = self.node.create_timer(0.1, self.publish_velocity)

        self.node.get_logger().info(
            f'Velocity: E={east:.2f}, N={north:.2f}, U={up:.2f}, yaw_rate={yaw_rate:.2f}'
        )

    def stop_velocity(self):
        """Stop velocity control and stop streaming setpoints"""
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_vyaw = 0.0

        # Publish the zeroed setpoint explicitly before tearing the timer down,
        # so the last thing on the wire is a commanded stop rather than the
        # previous non-zero velocity left to expire on ArduPilot's timeout.
        if self.velocity_timer is not None:
            self.publish_velocity()
            self.node.destroy_timer(self.velocity_timer)
            self.velocity_timer = None

        self.node.get_logger().info('Velocity control stopped')

    def publish_velocity(self):
        """Publish velocity setpoint at 10Hz"""
        msg = Twist()
        msg.linear.x = self.target_vx
        msg.linear.y = self.target_vy
        msg.linear.z = self.target_vz
        msg.angular.z = self.target_vyaw
        self.vel_pub.publish(msg)

    def set_acceleration(self, east, north, up):
        """
        Set acceleration in ENU frame (matches goto_local's convention)
        Args:
            east: East acceleration (m/s²)
            north: North acceleration (m/s²)
            up: Up acceleration (m/s²)
        """
        self.target_ax = east
        self.target_ay = north
        self.target_az = up

        if self.accel_timer is None:
            self.accel_timer = self.node.create_timer(0.1, self.publish_accel)

        self.node.get_logger().info(
            f'Acceleration: E={east:.2f}, N={north:.2f}, U={up:.2f}'
        )

    def stop_acceleration(self):
        """Stop acceleration control and stop streaming setpoints"""
        self.target_ax = 0.0
        self.target_ay = 0.0
        self.target_az = 0.0

        # Same reasoning as stop_velocity(): command the stop explicitly
        # before the stream ends.
        if self.accel_timer is not None:
            self.publish_accel()
            self.node.destroy_timer(self.accel_timer)
            self.accel_timer = None

        self.node.get_logger().info('Acceleration control stopped')

    def publish_accel(self):
        """Publish acceleration setpoint at 10Hz"""
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

    @staticmethod
    def _yaw_from_quat(qw, qx, qy, qz):
        """Yaw in degrees, CCW positive from East, from an ENU quaternion"""
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return math.degrees(yaw_rad)

    def get_telemetry(self):
        """
        Uniform telemetry snapshot, grouped by category.
        Convention throughout: ENU (east/north/up) for world-frame quantities,
        RFU (right/forward/up) for body-frame quantities, yaw CCW positive.
        """
        return {
            'state': {
                'armed': self.armed,
                'mode': self.mode,
            },
            'home': {
                'lat': self.home_lat,
                'lon': self.home_lon,
                'alt': self.home_alt,
            },
            'global_position': {
                'lat': self.current_lat,
                'lon': self.current_lon,
                'alt': self.current_alt,
            },
            'local_position': {
                'east': self.odom_east,
                'north': self.odom_north,
                'up': self.odom_up,
                'yaw_deg': self._yaw_from_quat(self.odom_qw, self.odom_qx, self.odom_qy, self.odom_qz),
            },
            'velocity_body': {
                'right': self.vel_right,
                'forward': self.vel_forward,
                'up': self.vel_up,
                'yaw_rate': self.vel_yaw_rate,
            },
            'velocity_body_odom': {
                'right': self.odom_vel_right,
                'forward': self.odom_vel_forward,
                'up': self.odom_vel_up,
                'yaw_rate': self.odom_yaw_rate,
            },
            'velocity_gps': {
                'east': self.gps_vel_east,
                'north': self.gps_vel_north,
                'up': self.gps_vel_up,
            },
            'imu': {
                'yaw_deg': self._yaw_from_quat(self.imu_qw, self.imu_qx, self.imu_qy, self.imu_qz),
                'gyro_right': self.imu_gyro_right,
                'gyro_forward': self.imu_gyro_forward,
                'gyro_up': self.imu_gyro_up,
                'accel_right': self.imu_accel_right,
                'accel_forward': self.imu_accel_forward,
                'accel_up': self.imu_accel_up,
            },
            'battery': {
                'voltage': self.battery_voltage,
                'current': self.battery_current,
                'percentage': self.battery_percentage,
            },
            'gps_status': {
                'fix_type': self.gps_fix_type,
                'satellites_visible': self.gps_satellites_visible,
                'eph': self.gps_eph,
                'epv': self.gps_epv,
            },
            'heading_deg': self.heading_deg,
            'vfr_hud': {
                'airspeed': self.vfr_airspeed,
                'groundspeed': self.vfr_groundspeed,
                'heading_deg': self.vfr_heading_deg,
                'throttle': self.vfr_throttle,
                'altitude': self.vfr_altitude,
                'climb': self.vfr_climb,
            },
            'rc': {
                'channels': self.rc_channels,
                'rssi': self.rc_rssi,
            },
            'statustext': {
                'text': self.last_statustext,
                'severity': self.last_statustext_severity,
            },
        }
