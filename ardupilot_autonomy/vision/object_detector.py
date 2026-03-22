# ardupilot_autonomy/vision/object_detector.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math


class ObjectDetector(Node):
    """Mock object detector - publishes steady circular motion"""

    def __init__(self):
        super().__init__('object_detector')

        # Publisher: normalized bbox center coordinates
        # x, y in range [-1.0, 1.0] where (0,0) = image center
        # z = bbox area as fraction of image [0.0, 1.0]
        self.pub = self.create_publisher(Point, '/vision/target', 10)

        # Circular motion parameters
        self.radius = 0.3  # 30% of image half-width
        self.angular_velocity = 0.5  # rad/s
        self.bbox_area = 0.1  # 10% of image

        self.time = 0.0
        self.timer = self.create_timer(0.1, self.publish_target)  # 10 Hz

        self.get_logger().info('Mock object detector started - circular motion')

    def publish_target(self):
        """Publish smooth circular trajectory"""
        dt = 1.0 / 15.0  # Match actual rate
        self.time += dt

        msg = Point()
        msg.x = self.radius * math.cos(self.angular_velocity * self.time)
        msg.y = self.radius * math.sin(self.angular_velocity * self.time)
        msg.z = self.bbox_area

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()