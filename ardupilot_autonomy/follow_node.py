#!/usr/bin/env python3
"""
Follow Me Node - IBVS based follow me with Kalman filter and yaw-corrected velocity.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import numpy as np


class FollowMeNode(Node):

    def __init__(self):
        super().__init__('follow_me_node')

        from ardupilot_autonomy.mavros_interface import MavrosInterface
        self.mavros = MavrosInterface(self)

        # --- Parameters ---
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('tolerance_px', 50.0)
        self.declare_parameter('lost_target_timeout', 5.0)
        self.declare_parameter('outlier_threshold', 150.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('camera_yaw_deg', -90.0)

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.kp = self.get_parameter('kp').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.tolerance_px = self.get_parameter('tolerance_px').value
        self.lost_timeout = self.get_parameter('lost_target_timeout').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        control_rate = self.get_parameter('control_rate').value
        self.camera_yaw = np.radians(self.get_parameter('camera_yaw_deg').value)

        # --- State ---
        self.latest_detection = None       # raw bbox from detector
        self.last_detection_time = None
        self.target_acquired = False


        # --- kalman ----
        dt = 1.0 / control_rate
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        self.Q = np.eye(4) * 5.0  # 0.1
        self.R = np.eye(2) * 1.0  # 10.0
        self.P0 = np.eye(4) * 500.0

        # Kalman state placeholder
        self.kf_state = None               # [cx, cy, vx, vy]
        self.kf_P = None

        # --- ROS ---
        self.create_subscription(
            Detection2DArray,
            '/detections/persons',
            self.detection_callback,
            10
        )

        self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info(
            f'Params: kp={self.kp} max_vel={self.max_vel} '
            f'tolerance={self.tolerance_px}px timeout={self.lost_timeout}s '
            f'outlier={self.outlier_threshold}px rate={control_rate}Hz'
        )

        self.get_logger().info('Follow me node ready')

    # ------------------------------------------------------------------
    # Detection callback — just stores latest bbox, never acts directly
    # ------------------------------------------------------------------
    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return
        best = msg.detections[0]
        self.latest_detection = best

    # ------------------------------------------------------------------
    # Kalman predict — called every control tick
    # ------------------------------------------------------------------
    def kalman_predict(self):
        if self.kf_state is None:
            return
        self.kf_state = self.F @ self.kf_state
        self.kf_P = self.F @ self.kf_P @ self.F.T + self.Q

    # ------------------------------------------------------------------
    # Kalman update — called when a valid detection arrives
    # ------------------------------------------------------------------
    def kalman_update(self, cx, cy):
        z = np.array([cx, cy])
        if self.kf_state is None:
            self.kf_state = np.array([cx, cy, 0.0, 0.0])
            self.kf_P = self.P0.copy()
            return
        S = self.H @ self.kf_P @ self.H.T + self.R
        K = self.kf_P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.kf_state
        self.kf_state = self.kf_state + K @ y
        self.kf_P = (np.eye(4) - K @ self.H) @ self.kf_P
        self.get_logger().info(f'raw=({cx:.1f},{cy:.1f}) kf=({self.kf_state[0]:.1f},{self.kf_state[1]:.1f})')

    # ------------------------------------------------------------------
    # Outlier check — reject if detection jumps too far from prediction
    # ------------------------------------------------------------------
    def is_outlier(self, cx, cy) -> bool:
        # PLACEHOLDER: return True if detection is an outlier
        return False

    # ------------------------------------------------------------------
    # Compute velocity command
    # Combines: pixel error → P gain → clamp → yaw rotation
    # Returns (vx_world, vy_world)
    # ------------------------------------------------------------------
    def compute_velocity_command(self, cx, cy) -> tuple:
        # Pixel errors
        error_x = self.image_height / 2.0 - cy   # vertical → body vx
        error_y = self.image_width / 2.0 - cx    # horizontal → body vy

        # Within tolerance — no action
        if abs(error_x) <= self.tolerance_px and abs(error_y) <= self.tolerance_px:
            return 0.0, 0.0

        # Proportional control
        vx = float(np.clip(self.kp * error_x, -self.max_vel, self.max_vel))
        vy = float(np.clip(self.kp * error_y, -self.max_vel, self.max_vel))

        # Yaw correction — rotate body frame velocity to world frame
        # psi = self.mavros.local_yaw
        psi = self.mavros.local_yaw - self.camera_yaw # for camera yaw
        vx_world = vx * np.cos(psi) - vy * np.sin(psi)
        vy_world = vx * np.sin(psi) + vy * np.cos(psi)

        self.get_logger().debug(
            f'error_x={error_x:.1f} error_y={error_y:.1f} '
            f'vx_w={vx_world:.3f} vy_w={vy_world:.3f} psi={np.degrees(psi):.1f}°'
        )

        return vx_world, vy_world

    # ------------------------------------------------------------------
    # Main control loop — runs at control_rate Hz
    # ------------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now()

        # Step 1: Kalman predict every tick
        self.kalman_predict()

        # Step 2: Process latest detection if available
        if self.latest_detection is not None:
            cx = self.latest_detection.bbox.center.position.x
            cy = self.latest_detection.bbox.center.position.y

            if not self.is_outlier(cx, cy):
                self.kalman_update(cx, cy)
                self.last_detection_time = now
                self.target_acquired = True

            self.latest_detection = None

        # Step 3: Lost target check
        if self.last_detection_time is not None:
            elapsed = (now - self.last_detection_time).nanoseconds / 1e9
            if elapsed > self.lost_timeout:
                if self.target_acquired:
                    self.get_logger().warn('Target lost — hovering')
                self.target_acquired = False
                self.kf_state = None
                self.kf_P = None
                self.mavros.stop_velocity()
                return

        # Step 4: No target yet
        if not self.target_acquired or self.kf_state is None:
            return

        # Step 5: Compute and send velocity using Kalman-smoothed position
        vx_world, vy_world = self.compute_velocity_command(
            self.kf_state[0], self.kf_state[1]
        )

        self.mavros.set_velocity(vx_world, vy_world, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = FollowMeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()