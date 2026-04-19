#!/usr/bin/env python3
"""
Waypoint Navigation Test v0.5.0
Based on scan_sequence_v0.1.0.py pattern.

Sequence:
  Move 10m North / East / South / West with yaw aligned to direction of travel.
  Waits until distance < 2m then velocity < 0.5 m/s before triggering next waypoint.

Logs NED offset + compass heading every 2 seconds. Never crashes on log failure.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TwistStamped
import math
import time
import sys


class YawTestSequence(Node):

    def __init__(self):
        super().__init__('yaw_test_sequence')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- state ---
        self.local_north = 0.0
        self.local_east = 0.0
        self.local_up = 0.0
        self.compass_hdg = None
        self.local_vx = 0.0
        self.local_vy = 0.0

        # --- subscriptions ---
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self._hdg_cb, qos)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self._vel_cb, qos)

        # --- service clients ---
        self.goto_client = self.create_client(Trigger, '/vehicle/goto_neu')
        self.param_client = self.create_client(
            SetParameters, '/vehicle_interface/set_parameters')

        # --- wait for services ---
        self.get_logger().info('Waiting for services...')
        self.goto_client.wait_for_service(timeout_sec=10.0)
        self.param_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready!')

        # --- wait for first pose ---
        self.get_logger().info('Waiting for position data...')
        while self.local_up == 0.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'✅ Position locked - altitude: {self.local_up:.2f}m')

        # capture origin
        self.origin_north = self.local_north
        self.origin_east = self.local_east
        self.origin_up = self.local_up

    def _pose_cb(self, msg):
        self.local_east = msg.pose.position.x
        self.local_north = msg.pose.position.y
        self.local_up = msg.pose.position.z

    def _hdg_cb(self, msg):
        self.compass_hdg = msg.data

    def _vel_cb(self, msg):
        self.local_vx = msg.twist.linear.x
        self.local_vy = msg.twist.linear.y

    def _horizontal_speed(self):
        return math.sqrt(self.local_vx ** 2 + self.local_vy ** 2)

    def _distance_to(self, target_n, target_e):
        return math.sqrt(
            (self.local_north - target_n) ** 2 +
            (self.local_east - target_e) ** 2
        )

    def _wait_for_waypoint(self, target_n, target_e, timeout=60.0):
        """Wait until within 2m of target, then wait for velocity < 0.5 m/s."""
        start = time.time()

        # Step 1: wait for distance < 2m
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            dist = self._distance_to(target_n, target_e)
            self._log()
            if dist < 2.0:
                self.get_logger().info(f'[nav_test] Within 2m — waiting for stop')
                break
            time.sleep(0.5)
        else:
            self.get_logger().warn('[nav_test] ⚠ Waypoint timeout')
            return False

        # Step 2: wait for velocity < 0.5 m/s
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            speed = self._horizontal_speed()
            self._log()
            if speed < 0.5:
                self.get_logger().info(f'[nav_test] ✅ Waypoint reached — speed {speed:.2f} m/s')
                return True
            time.sleep(0.5)

        self.get_logger().warn('[nav_test] ⚠ Velocity timeout')
        return False

    def _log(self):
        try:
            hdg_str = f'{self.compass_hdg:.1f}°' if self.compass_hdg is not None else 'N/A'
            dn = self.local_north - self.origin_north
            de = self.local_east - self.origin_east
            self.get_logger().info(
                f'[yaw_test] NED offset: N={dn:.2f}m  E={de:.2f}m  '
                f'Up={self.local_up:.2f}m  Heading={hdg_str}'
            )
        except Exception:
            pass

    def _set_params(self, north, east, up, yaw):
        params = [
            Parameter(name='goto_north',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(north))),
            Parameter(name='goto_east',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(east))),
            Parameter(name='goto_up',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(up))),
            Parameter(name='goto_yaw',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(yaw))),
        ]
        req = SetParameters.Request()
        req.parameters = params
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() is not None

    def _goto(self, north, east, up, yaw_ned_deg):
        self._set_params(north, east, up, yaw_ned_deg)
        self.goto_client.call_async(Trigger.Request())
        self.get_logger().info(
            f'[yaw_test] Goto: N={north:.1f}  E={east:.1f}  Up={up:.1f}  Yaw={yaw_ned_deg:.0f}° NED'
        )

    def _calc_yaw(self, from_n, from_e, to_n, to_e):
        """Calculate NED yaw (degrees) from one point to another."""
        dn = to_n - from_n
        de = to_e - from_e
        yaw = math.degrees(math.atan2(de, dn))
        if yaw < 0:
            yaw += 360.0
        return yaw

    def _sleep_log(self, seconds, label=''):
        if label:
            self.get_logger().info(f'[yaw_test] {label}')
        steps = int(seconds / 2)
        for _ in range(steps):
            rclpy.spin_once(self, timeout_sec=0.1)
            self._log()
            time.sleep(2.0)

    def execute(self):
        on = self.origin_north
        oe = self.origin_east
        ou = self.origin_up

        self.get_logger().info('[nav_test] ===== Sequence started =====')

        # Waypoint moves with yaw aligned to direction of travel
        waypoints = [
            ('North', on + 10.0, oe),
            ('East',  on,        oe + 10.0),
            ('South', on - 10.0, oe),
            ('West',  on,        oe - 10.0),
        ]

        prev_n, prev_e = on, oe
        for direction, tn, te in waypoints:
            yaw = self._calc_yaw(prev_n, prev_e, tn, te)
            self.get_logger().info(f'[nav_test] Move {direction} 10m facing {yaw:.0f}°')
            self._goto(tn, te, ou, yaw)
            self._wait_for_waypoint(tn, te)
            prev_n, prev_e = tn, te

        self.get_logger().info('[nav_test] ===== Sequence complete =====')
        return True


def main():
    print('[nav_test] Starting...')
    rclpy.init()
    node = YawTestSequence()
    success = node.execute()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
