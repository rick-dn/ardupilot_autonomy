#!/usr/bin/env python3
"""
Goto local - fly to one ENU waypoint relative to the EKF origin, then exit.

Assumes the vehicle is already airborne and in GUIDED (run takeoff first).

Standalone by design: no shared base class, no helper module. Everything this
sequence needs is in this file, so it reads top to bottom and editing it
cannot affect another sequence.

Interrupting with ctrl-C stops the script but NOT the vehicle: the flight
controller keeps flying toward the last setpoint it was given.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from udl_aa_msgs.msg import VehicleCommand, VehicleTelemetry

COMMAND_TOPIC = '/udl_dev/command'
TELEMETRY_TOPIC = '/udl_dev/telemetry'

# ENU target, metres from the EKF origin. yaw_deg is CCW positive from East,
# matching the convention used throughout mavros_adapter.
TARGET_EAST_M = 10.0
TARGET_NORTH_M = 0.0
TARGET_UP_M = 5.0
TARGET_YAW_DEG = 0.0

ARRIVE_TOLERANCE_M = 1.0
LINK_TIMEOUT_S = 15.0
ARRIVE_TIMEOUT_S = 90.0


class GotoLocal(Node):

    def __init__(self):
        super().__init__('goto_local')

        self.tel = None

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand, COMMAND_TOPIC, command_qos)
        self.create_subscription(
            VehicleTelemetry, TELEMETRY_TOPIC, self._telemetry_callback,
            telemetry_qos)

    def _telemetry_callback(self, msg):
        self.tel = msg

    def send(self, command, **params):
        """Fire one command at vehicle_node. Nothing acknowledges it."""
        msg = VehicleCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.sequence = 'goto_local'
        msg.token = 0
        msg.command = command
        msg.param_names = list(params.keys())
        msg.param_values = [float(v) for v in params.values()]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'-> {command} {params if params else ""}')

    def wait_until(self, predicate, timeout_s, what):
        """
        Poll telemetry until predicate holds. The predicate is only called
        once telemetry exists, so it can read self.tel without guarding.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.tel is not None and predicate():
                return True
            time.sleep(0.1)
        self.get_logger().error(f'timed out after {timeout_s}s waiting for {what}')
        return False

    def _distance_to_target(self):
        lp = self.tel.local_position
        return math.sqrt(
            (lp.east - TARGET_EAST_M) ** 2
            + (lp.north - TARGET_NORTH_M) ** 2
            + (lp.up - TARGET_UP_M) ** 2
        )

    def run(self):
        # mode == '' is vehicle_node reporting no live FC link.
        if not self.wait_until(
                lambda: self.tel.state.mode != '', LINK_TIMEOUT_S, 'FC link'):
            return

        if not self.tel.state.armed:
            self.get_logger().error('not armed - run takeoff first')
            return

        self.send(
            'GOTO_LOCAL',
            east=TARGET_EAST_M,
            north=TARGET_NORTH_M,
            up=TARGET_UP_M,
            yaw_deg=TARGET_YAW_DEG,
        )

        if not self.wait_until(
                lambda: self._distance_to_target() <= ARRIVE_TOLERANCE_M,
                ARRIVE_TIMEOUT_S,
                f'arrival at E={TARGET_EAST_M} N={TARGET_NORTH_M} U={TARGET_UP_M}'):
            return

        lp = self.tel.local_position
        self.get_logger().info(
            f'arrived, E={lp.east:.2f} N={lp.north:.2f} U={lp.up:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GotoLocal()
    # Spinning on a daemon thread is what lets run() read as linear flight
    # logic while telemetry keeps arriving in the background.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        print('goto_local interrupted - vehicle keeps its last setpoint')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
