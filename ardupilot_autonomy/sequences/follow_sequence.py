# ardupilot_autonomy/sequences/follow_sequence.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from ardupilot_autonomy.mavros_interface import MavrosInterface


class FollowSequence(Node):

    def __init__(self):
        super().__init__('follow_sequence')

        # Use your MavrosInterface wrapper
        self.mavros = MavrosInterface(self)

        # Subscribe to vision target
        self.sub = self.create_subscription(
            Point,
            '/vision/target',
            self.target_callback,
            10
        )

        self.get_logger().info('Follow sequence started')

    def target_callback(self, msg):
        """Called at 15 Hz - calculate velocity from target position"""

        # P gains (tune these)
        Kp_lateral = 0.5  # For centering horizontally
        Kp_vertical = 0.5  # For centering vertically
        Kp_distance = 0.3  # For maintaining distance (bbox size)

        # Target errors (normalized coordinates from -1 to 1)
        error_x = msg.x  # Horizontal offset from center
        error_y = msg.y  # Vertical offset from center

        # Calculate body-frame velocities
        vy = Kp_lateral * error_x  # Right/left to center target
        vz = -Kp_vertical * error_y  # Up/down to center target (invert for NED)
        vx = 0.5  # Constant forward for now

        self.get_logger().info(
            f'Target x={msg.x:.2f}, y={msg.y:.2f} → vy={vy:.2f}, vz={vz:.2f}'
        )

        # Send velocity command
        self.mavros.set_velocity(vx, vy, vz, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FollowSequence()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()