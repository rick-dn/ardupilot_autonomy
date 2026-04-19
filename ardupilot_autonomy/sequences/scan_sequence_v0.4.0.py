#!/usr/bin/env python3
"""
Yaw + Position Test Sequence v0.4.0
Based on scan_sequence_v0.1.0.py pattern.

Sequence:
  1. Yaw North / East / South / West (5s each, hold position)
  2. Move 10m North / East / South / West (10s each, yaw calculated from direction of travel)

Logs NED offset + compass heading every 1 second. Never crashes on log failure.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
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

        # --- subscriptions ---
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_cb, qos)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self._hdg_cb, qos)

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

        self.get_logger().info('[yaw_test] ===== Sequence started =====')

        # Phase 1: yaw in place (5s each)
        for direction, yaw in [('North', 0), ('East', 90), ('South', 180), ('West', 270)]:
            self.get_logger().info(f'[yaw_test] Phase 1: Yaw {direction} ({yaw}° NED)')
            self._goto(on, oe, ou, yaw)
            self._sleep_log(5)

        # Return to origin
        self.get_logger().info('[yaw_test] Return to origin')
        self._goto(on, oe, ou, 0.0)
        self._sleep_log(5)

        # Phase 2: position moves, yaw calculated from direction of travel (10s each)
        waypoints = [
            ('North', on + 10.0, oe),
            ('East',  on,        oe + 10.0),
            ('South', on - 10.0, oe),
            ('West',  on,        oe - 10.0),
        ]

        prev_n, prev_e = on, oe
        for direction, tn, te in waypoints:
            yaw = self._calc_yaw(prev_n, prev_e, tn, te)
            self.get_logger().info(f'[yaw_test] Phase 2: Move {direction} 10m facing {yaw:.0f}°')
            self._goto(tn, te, ou, yaw)
            self._sleep_log(10)
            prev_n, prev_e = tn, te

        self.get_logger().info('[yaw_test] ===== Sequence complete =====')
        return True


def main():
    print('[yaw_test] Starting...')
    rclpy.init()
    node = YawTestSequence()
    success = node.execute()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
