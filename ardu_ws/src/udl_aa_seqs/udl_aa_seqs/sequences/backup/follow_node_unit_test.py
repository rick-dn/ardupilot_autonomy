#!/usr/bin/env python3
"""
Follow Node - Simple bang-bang x-axis correction.
If detected object is outside x-axis tolerance, fires vx=0.1 for 5 seconds then stops.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

import numpy as np


class FollowNode(Node):

    def __init__(self):
        super().__init__('follow_node')

        from ardupilot_autonomy.mavros_interface import MavrosInterface
        self.mavros = MavrosInterface(self)

        # --- Parameters ---
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('tolerance_px', 50.0)   # pixels (~0.5m tolerance)
        self.declare_parameter('drive_velocity', 0.1)     # m/s
        self.declare_parameter('drive_duration', 5.0)     # seconds

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.tolerance_px = self.get_parameter('tolerance_px').value
        self.drive_vel = self.get_parameter('drive_velocity').value
        self.drive_duration = self.get_parameter('drive_duration').value

        # --- State ---
        self.driving = False
        self.drive_timer = None

        # --- ROS ---
        self.create_subscription(
            Detection2DArray,
            '/detections/persons',
            self.detection_callback,
            10
        )

        self.get_logger().info('Follow node ready')

    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return

        # Already driving — ignore new detections until done
        if self.driving:
            return

        best = msg.detections[0]
        cy = best.bbox.center.position.y
        cx = best.bbox.center.position.x

        # critical
        image_center_y = self.image_height / 2.0
        image_center_x = self.image_width / 2.0

        error_x = image_center_y - cy
        error_y = image_center_x - cx

        if abs(error_x) <= self.tolerance_px:
            self.get_logger().info(f'Target centred x (error={error_x:.1f}px) — no action')
            vx = 0
        else:
            # Determine direction
            vx = self.drive_vel if error_x > 0 else -self.drive_vel

        if abs(error_y) <= self.tolerance_px:
            self.get_logger().info(f'Target centred y (error={error_y:.1f}px) — no action')
            vy = 0
        else:
            # Determine direction
            vy = self.drive_vel if error_y > 0 else -self.drive_vel

        if vx == 0 and vy == 0:
            return

        self.get_logger().info(
            f'Error x={error_x:.1f}px vx={vx} | Error y={error_y:.1f}px vy={vy} — driving for {self.drive_duration}s'
        )

        self.driving = True
        # self.mavros.set_velocity(vx, vy, 0.0, 0.0)

        # yaw adjusted
        psi = self.mavros.local_yaw
        vx_world = vx * np.cos(psi) - vy * np.sin(psi)
        vy_world = vx * np.sin(psi) + vy * np.cos(psi)
        self.mavros.set_velocity(vx_world, vy_world, 0.0, 0.0)

        # Stop after drive_duration seconds
        self.drive_timer = self.create_timer(self.drive_duration, self.stop_drive)

    def stop_drive(self):
        self.mavros.stop_velocity()
        self.driving = False
        self.get_logger().info('Drive complete — hovering')

        # Cancel one-shot timer
        if self.drive_timer is not None:
            self.drive_timer.cancel()
            self.drive_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
