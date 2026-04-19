#!/usr/bin/env python3
"""
Diamond Velocity Test v0.2.0
Sends 4 velocity commands in a diamond pattern (NE/SE/SW/NW).
8 seconds per leg at 1 m/s, then waits for speed < 0.3 m/s before next.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time
import sys


class DiamondVelocityTest(Node):

    def __init__(self):
        super().__init__('diamond_velocity_test')

        from ardupilot_autonomy.mavros_interface import MavrosInterface
        self.mavros = MavrosInterface(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.local_vx = 0.0
        self.local_vy = 0.0

        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self._vel_cb, qos)

    def _vel_cb(self, msg):
        self.local_vx = msg.twist.linear.x
        self.local_vy = msg.twist.linear.y

    def _horizontal_speed(self):
        return math.sqrt(self.local_vx ** 2 + self.local_vy ** 2)

    def _spin(self, duration):
        start = time.time()
        while time.time() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_stop(self, timeout=30.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            speed = self._horizontal_speed()
            self.get_logger().info(f'[diamond] Speed: {speed:.2f} m/s')
            if speed < 0.3:
                self.get_logger().info('[diamond] ✅ Stopped')
                return
            time.sleep(0.5)
        self.get_logger().warn('[diamond] ⚠ Stop timeout — continuing')

    def execute(self):
        vel = 1.0
        fire_duration = 8

        legs = [
            ('NE',  vel,  vel),
            ('SE', -vel,  vel),
            ('SW', -vel, -vel),
            ('NW',  vel, -vel),
        ]

        self.get_logger().info('[diamond] ===== Sequence started =====')

        for direction, vx, vy in legs:
            self.get_logger().info(f'[diamond] Moving {direction}')
            self.mavros.set_velocity(vx, vy, 0.0, 0.0)
            self._spin(fire_duration)
            self.mavros.stop_velocity()
            self._wait_for_stop()

        self.get_logger().info('[diamond] ===== Sequence complete =====')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = DiamondVelocityTest()
    success = node.execute()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()