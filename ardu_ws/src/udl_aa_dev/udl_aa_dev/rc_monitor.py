#!/usr/bin/env python3
"""
RC Monitor - flight only. Turns RC switch flicks into sequence launches.

Carried over unchanged from the flight-tested version running on the Pi
(~/ardu_ws/src/ardupilot_autonomy/ardupilot_autonomy/rc_monitor.py). Only the
two spawned executable names differ, to match this package's sequences.

Behaviour is deliberately NOT modernised - it is the version that has flown.
Four known quirks come with it, to be addressed when real flight is in scope:

  1. last_switch* start at 1000, so booting with a switch already high reads
     as a rising edge and launches immediately.
  2. An empty channels array is not distinguished from a low switch.
  3. A falling edge does nothing - there is no abort.
  4. Popen handles are not kept, so two flicks start two sequences, both
     commanding the vehicle.

Note this subscribes to /mavros/rc/in directly rather than going through
vehicle_node, so on the flight launch vehicle_node is not the only MAVROS
client. It is read-only, so it does not contend for control.
"""

import subprocess

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn


class RCMonitor(Node):
    def __init__(self):
        super().__init__('rc_monitor')

        # Subscribe to RC channels
        self.rc_sub = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            10
        )

        # Track switch states to detect flicks (rising edge)
        self.last_switch2 = 1000
        self.last_switch4 = 1000

        self.get_logger().info('RC Monitor started')
        self.get_logger().info('  channels[7] > 1800: Trigger takeoff')
        self.get_logger().info('  channels[5] > 1800: Trigger land')

    def rc_callback(self, msg):
        if len(msg.channels) < 8:
            return

        switch2 = msg.channels[7]  # Switch 2
        switch4 = msg.channels[5]  # Switch 4

        # Detect rising edge on switch2 (switch flick to ON)
        if switch2 > 1800 and self.last_switch2 <= 1800:
            self.get_logger().info('🎮 channels[7] flicked ON - Triggering takeoff')
            subprocess.Popen(['ros2', 'run', 'udl_aa_dev', 'takeoff'])

        # Detect rising edge on switch4 (switch flicked to ON)
        if switch4 > 1800 and self.last_switch4 <= 1800:
            self.get_logger().info('🎮 channels[5] flicked ON - Triggering land')
            subprocess.Popen(['ros2', 'run', 'udl_aa_dev', 'land'])

        self.last_switch2 = switch2
        self.last_switch4 = switch4


def main(args=None):
    rclpy.init(args=args)
    node = RCMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down rc monitor')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
