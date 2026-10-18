#!/usr/bin/env python3
"""
Takeoff - GUIDED, arm, climb to TAKEOFF_ALT_M, exit once there.

Standalone by design: no shared base class, no helper module. Everything this
sequence needs is in this file, so it reads top to bottom and editing it
cannot affect another sequence. The cost is that send() and wait_until() are
repeated per file - keep them in step if you change the topics.

Interrupting with ctrl-C stops the script but NOT the vehicle: the flight
controller keeps flying the last setpoint it was given.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from udl_aa_msgs.msg import VehicleCommand, VehicleTelemetry

COMMAND_TOPIC = '/udl_dev/command'
TELEMETRY_TOPIC = '/udl_dev/telemetry'

TAKEOFF_ALT_M = 5.0
ARRIVE_TOLERANCE_M = 0.5
LINK_TIMEOUT_S = 15.0
ACK_TIMEOUT_S = 15.0
CLIMB_TIMEOUT_S = 60.0


class Takeoff(Node):

    def __init__(self):
        super().__init__('takeoff')

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
        msg.sequence = 'takeoff'
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

    def run(self):
        # mode == '' is vehicle_node reporting no live FC link - it is the one
        # telemetry value no real flight controller ever produces.
        if not self.wait_until(
                lambda: self.tel.state.mode != '', LINK_TIMEOUT_S, 'FC link'):
            return

        self.send('SET_MODE_GUIDED')
        if not self.wait_until(
                lambda: self.tel.state.mode == 'GUIDED', ACK_TIMEOUT_S, 'GUIDED'):
            return

        self.send('ARM')
        if not self.wait_until(
                lambda: self.tel.state.armed, ACK_TIMEOUT_S, 'armed'):
            return

        self.send('TAKEOFF', altitude=TAKEOFF_ALT_M)
        target = TAKEOFF_ALT_M - ARRIVE_TOLERANCE_M
        if not self.wait_until(
                lambda: self.tel.local_position.up >= target,
                CLIMB_TIMEOUT_S, f'{TAKEOFF_ALT_M} m'):
            return

        self.get_logger().info(
            f'takeoff complete, up={self.tel.local_position.up:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = Takeoff()
    # Spinning on a daemon thread is what lets run() read as linear flight
    # logic while telemetry keeps arriving in the background.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        print('takeoff interrupted - vehicle keeps its last setpoint')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
