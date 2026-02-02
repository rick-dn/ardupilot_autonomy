# rc_monitor.py
# !/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
import subprocess


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
        self.last_ch5 = 1000
        self.last_ch6 = 1000

        self.get_logger().info('RC Monitor started')
        self.get_logger().info('  CH5 > 1800: Trigger takeoff')
        self.get_logger().info('  CH6 > 1800: Trigger land')

    def rc_callback(self, msg):
        if len(msg.channels) < 6:
            return

        ch5 = msg.channels[5]  # Switch 2
        ch6 = msg.channels[6]  # Switch 4

        # Detect rising edge on CH5 (switch flick to ON)
        if ch5 > 1800 and self.last_ch5 <= 1800:
            self.get_logger().info('🎮 CH5 flicked ON - Triggering takeoff')
            subprocess.Popen(['ros2', 'run', 'ardupilot_autonomy', 'takeoff_sequence'])

        # Detect rising edge on CH6
        if ch6 > 1800 and self.last_ch6 <= 1800:
            self.get_logger().info('🎮 CH6 flicked ON - Triggering land')
            subprocess.Popen(['ros2', 'run', 'ardupilot_autonomy', 'scan_sequence'])

        self.last_ch5 = ch5
        self.last_ch6 = ch6


def main():
    rclpy.init()
    node = RCMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()