#!/usr/bin/env python3
"""
Land - hand the vehicle to the flight controller's LAND mode, wait for it to
settle and disarm, then exit.

Standalone by design: no shared base class, no helper module. Everything this
sequence needs is in this file, so it reads top to bottom and editing it
cannot affect another sequence.

Unlike the other sequences, interrupting this one with ctrl-C is harmless:
LAND is a flight-controller mode, not a streamed setpoint, so the descent
continues whether or not this script is alive.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from udl_aa_msgs.msg import VehicleCommand, VehicleTelemetry

COMMAND_TOPIC = '/udl_dev/command'
TELEMETRY_TOPIC = '/udl_dev/telemetry'

LINK_TIMEOUT_S = 15.0
MODE_TIMEOUT_S = 15.0
DESCENT_TIMEOUT_S = 120.0


class Land(Node):

    def __init__(self):
        super().__init__('land')

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
        msg.sequence = 'land'
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
        # mode == '' is vehicle_node reporting no live FC link.
        if not self.wait_until(
                lambda: self.tel.state.mode != '', LINK_TIMEOUT_S, 'FC link'):
            return

        self.send('LAND')
        if not self.wait_until(
                lambda: self.tel.state.mode == 'LAND', MODE_TIMEOUT_S, 'LAND mode'):
            return

        # Disarm is the end of the descent, not touchdown: ArduPilot disarms
        # itself once it has settled. in_air is checked too because it is
        # altitude-latched, so it can still read true at the moment of disarm.
        if not self.wait_until(
                lambda: not self.tel.state.armed and not self.tel.state.in_air,
                DESCENT_TIMEOUT_S, 'touchdown and disarm'):
            return

        self.get_logger().info(
            f'landed, up={self.tel.local_position.up:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = Land()
    # Spinning on a daemon thread is what lets run() read as linear flight
    # logic while telemetry keeps arriving in the background.
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        print('land interrupted - descent continues, LAND is an FC mode')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
