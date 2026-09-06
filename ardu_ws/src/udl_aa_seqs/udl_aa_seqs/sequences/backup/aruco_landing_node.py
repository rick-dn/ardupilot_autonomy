#!/usr/bin/env python3
"""
ArUco Landing Node — Python port of flight_commander::ArucoLandingSequence.

Phases: SCAN -> HOVER_CAPTURE -> APPROACH -> (autopilot LAND)

Frame conventions (matching the C++ and mavros_interface.py):
  - Position telemetry is ENU: x = east, y = north, z = up (home-relative).
  - Velocity setpoints on /mavros/setpoint_velocity/cmd_vel_unstamped are
    interpreted by MAVROS as local ENU, NOT body frame. Hence kDescentVz is
    negative for descent, and body->world rotation is done here explicitly.
"""

import math
from collections import deque
from enum import Enum
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray


# ──────────────────────────────────────────────────────────────────────────
# Constants — direct transcription of ArucoLandingSequence.hpp
# ──────────────────────────────────────────────────────────────────────────

K_WP_TOLERANCE_M = 1.5

K_IMAGE_WIDTH = 640.0
K_IMAGE_HEIGHT = 480.0
K_KP = 0.005
K_MAX_VEL = 1.0
K_TOLERANCE_PX = 50.0
K_CAMERA_YAW_DEG = -90.0

K_WINDOW_SIZE = 15
K_MAX_AGE = 0.5          # s — recency floor
K_MAX_JUMP = 150.0       # px — coherence floor

K_SETTLE_DIST_M = 0.5
K_SETTLE_SPEED_MPS = 0.2

K_CONF_ARM = 0.20
K_CONF_ABANDON = 0.10

K_YAW_KP = 0.5
K_MAX_YAW_RATE = 0.3           # rad/s ≈ 17°/s
K_YAW_TOLERANCE_RAD = 0.087    # ~5°

K_DESCENT_VZ = -0.3      # m/s, negative = down (ENU)
K_LAND_ALT_M = 0.5


class ALPhase(Enum):
    SCAN = 0
    HOVER_CAPTURE = 1
    APPROACH = 2
    DONE = 3


class SequenceStatus(Enum):
    INITIATED = 0
    COMPLETE = 1


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DetectionEntry:
    __slots__ = ('hit', 'cx', 'cy', 'yaw', 'stamp')

    def __init__(self, hit=False, cx=0.0, cy=0.0, yaw=0.0, stamp=None):
        self.hit = hit
        self.cx = cx
        self.cy = cy
        self.yaw = yaw
        self.stamp = stamp


class DetectionSnapshot:
    __slots__ = ('valid', 'cx', 'cy', 'yaw', 'class_id', 'stamp',
                 'confidence', 'recency', 'hit_rate', 'coherence')

    def __init__(self):
        self.valid = False
        self.cx = 0.0
        self.cy = 0.0
        self.yaw = 0.0
        self.class_id = ''
        self.stamp = None
        self.confidence = 0.0
        self.recency = 0.0
        self.hit_rate = 0.0
        self.coherence = 0.0


class ArucoLandingNode(Node):

    def __init__(self):
        super().__init__('aruco_landing_node')

        from ardupilot_autonomy.mavros_interface import MavrosInterface
        self.mavros = MavrosInterface(self)

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('flight_altitude', 5.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('scan_yaw_deg', 0.0)
        self.declare_parameter('marker_east', 7.0)
        self.declare_parameter('marker_north', 0.0)

        self.flight_alt = self.get_parameter('flight_altitude').value
        control_rate = self.get_parameter('control_rate').value
        self.scan_yaw = math.radians(self.get_parameter('scan_yaw_deg').value)
        marker_e = self.get_parameter('marker_east').value
        marker_n = self.get_parameter('marker_north').value

        self.camera_yaw = math.radians(K_CAMERA_YAW_DEG)

        # ── Waypoints: (east, north, up) ──────────────────────────────
        # Scan rectangle around the marker. NOTE: this follows the *comments*
        # in the C++ constructor (north sweeps -10..+10 at east=7), not the
        # literal initializer order, which had east and north transposed.
        # See notes — this is the "should be right but isnt" bug.
        self.waypoints = [
            (marker_e,  marker_n - 10.0, self.flight_alt),   # south of marker
            (marker_e,  marker_n + 10.0, self.flight_alt),   # straight over it
            (marker_n,  marker_n + 10.0, self.flight_alt),
            (marker_n,  marker_n - 10.0, self.flight_alt),
        ]

        # ── State ─────────────────────────────────────────────────────
        self.phase = ALPhase.SCAN
        self.wp_index = 0
        self.servo_armed = False

        self.capture_east = 0.0
        self.capture_north = 0.0
        self.capture_up = 0.0

        self.detection_window = deque(maxlen=K_WINDOW_SIZE)
        self.detection_lock = Lock()
        self.last_class_id = ''

        # Body-frame velocity, for the hover-capture settle check.
        # mavros_interface only tracks position, so subscribe separately.
        self.vel_x = 0.0
        self.vel_y = 0.0

        # ── Publishers / subscribers ──────────────────────────────────
        # Own position-setpoint publisher: mavros_interface.goto_neu()
        # spin_once()s internally, which is unsafe from a timer callback.
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)

        det_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Detection2DArray, '/detections/aruco',
            self.detection_callback, det_qos)

        self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_callback, 10)

        self.create_timer(1.0 / control_rate, self.update)

        self.get_logger().info(
            f'[AL] started — SCAN phase, {len(self.waypoints)} waypoints, '
            f'alt={self.flight_alt}m rate={control_rate}Hz'
        )

    # ──────────────────────────────────────────────────────────────────
    # Telemetry
    # ──────────────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y

    @property
    def east(self):
        return self.mavros.local_x

    @property
    def north(self):
        return self.mavros.local_y

    @property
    def up(self):
        return self.mavros.local_z

    @property
    def yaw(self):
        return self.mavros.local_yaw

    # ──────────────────────────────────────────────────────────────────
    # Position setpoint — non-blocking equivalent of GotoENUCmd
    # ──────────────────────────────────────────────────────────────────
    def goto_enu(self, east, north, up, yaw_rad=None):
        if yaw_rad is None:
            yaw_rad = self.scan_yaw

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(east)
        msg.pose.position.y = float(north)
        msg.pose.position.z = float(up)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        self.setpoint_pub.publish(msg)

    # ──────────────────────────────────────────────────────────────────
    # Detection callback — pushes a miss entry when the frame is empty,
    # which is what makes hit_rate meaningful.
    # ──────────────────────────────────────────────────────────────────
    def detection_callback(self, msg: Detection2DArray):
        entry = DetectionEntry(stamp=self.get_clock().now())

        class_id = None
        if msg.detections and msg.detections[0].results:
            d = msg.detections[0]
            entry.hit = True
            entry.cx = d.bbox.center.position.x
            entry.cy = d.bbox.center.position.y

            q = d.results[0].pose.pose.orientation
            entry.yaw = 2.0 * math.atan2(q.z, q.w)
            class_id = d.results[0].hypothesis.class_id

        with self.detection_lock:
            if class_id is not None:
                self.last_class_id = class_id
            self.detection_window.append(entry)

    # ──────────────────────────────────────────────────────────────────
    # Snapshot — recency x hit_rate x coherence
    # ──────────────────────────────────────────────────────────────────
    def snapshot_detection(self) -> DetectionSnapshot:
        s = DetectionSnapshot()

        with self.detection_lock:
            if not self.detection_window:
                return s
            window = list(self.detection_window)
            last_class_id = self.last_class_id

        hits = [e for e in window if e.hit]

        s.hit_rate = len(hits) / float(K_WINDOW_SIZE)

        if not hits:
            return s

        newest = hits[-1]
        s.valid = True
        s.cx = newest.cx
        s.cy = newest.cy
        s.yaw = newest.yaw
        s.stamp = newest.stamp
        s.class_id = last_class_id

        age = (self.get_clock().now() - newest.stamp).nanoseconds / 1e9
        s.recency = clamp(1.0 - age / K_MAX_AGE, 0.0, 1.0)

        if len(hits) < 2:
            s.coherence = 1.0
        else:
            max_jump = 0.0
            for i in range(1, len(hits)):
                dx = hits[i].cx - hits[i - 1].cx
                dy = hits[i].cy - hits[i - 1].cy
                max_jump = max(max_jump, math.hypot(dx, dy))
            s.coherence = clamp(1.0 - max_jump / K_MAX_JUMP, 0.0, 1.0)

        s.confidence = s.recency * s.hit_rate * s.coherence
        return s

    # ──────────────────────────────────────────────────────────────────
    # Dispatch
    # ──────────────────────────────────────────────────────────────────
    def update(self):
        if self.phase == ALPhase.SCAN:
            self.tick_scan()
        elif self.phase == ALPhase.HOVER_CAPTURE:
            self.tick_hover_capture()
        elif self.phase == ALPhase.APPROACH:
            if self.tick_approach() == SequenceStatus.COMPLETE:
                self.on_exit()
                self.phase = ALPhase.DONE

    # ──────────────────────────────────────────────────────────────────
    # SCAN
    # ──────────────────────────────────────────────────────────────────
    def tick_scan(self):
        det = self.snapshot_detection()

        if det.valid:
            self.capture_east = self.east
            self.capture_north = self.north
            self.capture_up = self.up
            self.get_logger().info(
                f'[AL] marker detected (id={det.class_id} '
                f'cx={det.cx:.1f} cy={det.cy:.1f}) — HOVER_CAPTURE at '
                f'(E{self.capture_east:.2f} N{self.capture_north:.2f} '
                f'U{self.capture_up:.2f})'
            )
            self.phase = ALPhase.HOVER_CAPTURE
            return

        wp_e, wp_n, wp_u = self.waypoints[self.wp_index]
        self.goto_enu(wp_e, wp_n, wp_u)

        self.get_logger().info(
            f'[AL] SCAN wp={self.wp_index} '
            f'target=(E{wp_e:.1f} N{wp_n:.1f} U{wp_u:.1f}) '
            f'pos=(E{self.east:.2f} N{self.north:.2f})',
            throttle_duration_sec=1.0
        )

        if self.waypoint_reached(wp_e, wp_n):
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            self.get_logger().info(
                f'[AL] SCAN — advancing to waypoint {self.wp_index}')

    def waypoint_reached(self, wp_east, wp_north) -> bool:
        de = self.east - wp_east
        dn = self.north - wp_north
        return (de * de + dn * dn) < (K_WP_TOLERANCE_M * K_WP_TOLERANCE_M)

    # ──────────────────────────────────────────────────────────────────
    # HOVER_CAPTURE
    # ──────────────────────────────────────────────────────────────────
    def tick_hover_capture(self):
        # Hold the position captured at first detection, overriding the scan
        # waypoint so the drone returns to where it saw the marker rather
        # than wherever momentum carried it.
        self.goto_enu(self.capture_east, self.capture_north, self.capture_up)

        det = self.snapshot_detection()

        dist = math.hypot(self.east - self.capture_east,
                          self.north - self.capture_north)
        speed = math.hypot(self.vel_x, self.vel_y)

        self.get_logger().info(
            f'[AL] HOVER_CAPTURE — dist={dist:.2f} speed={speed:.2f} | '
            f'conf={det.confidence:.3f} (rec={det.recency:.2f} '
            f'hit={det.hit_rate:.2f} coh={det.coherence:.2f}) '
            f'cx={det.cx:.1f} cy={det.cy:.1f} '
            f'yaw={det.yaw:.2f} ({math.degrees(det.yaw):.1f}°)',
            throttle_duration_sec=0.5
        )

        if dist < K_SETTLE_DIST_M and speed < K_SETTLE_SPEED_MPS:
            self.get_logger().info(
                f'[AL] settled (dist={dist:.2f} speed={speed:.2f} '
                f'conf={det.confidence:.2f}) — APPROACH'
            )
            self.phase = ALPhase.APPROACH

    # ──────────────────────────────────────────────────────────────────
    # APPROACH
    # ──────────────────────────────────────────────────────────────────
    def tick_approach(self) -> SequenceStatus:
        det = self.snapshot_detection()

        # ── Low enough → hand off to autopilot LAND ───────────────────
        if self.up <= K_LAND_ALT_M:
            self.get_logger().info(
                f'[AL] alt={self.up:.2f} — handing off to LAND')
            self.mavros.land()
            return SequenceStatus.COMPLETE

        # ── Confidence gate with hysteresis ───────────────────────────
        if not self.servo_armed and det.confidence >= K_CONF_ARM:
            self.servo_armed = True
            self.get_logger().info(
                f'[AL] servo ARMED (conf={det.confidence:.2f})')
        elif self.servo_armed and det.confidence < K_CONF_ABANDON:
            self.servo_armed = False
            self.get_logger().warn(
                f'[AL] servo ABANDONED (conf={det.confidence:.2f})')

        if not self.servo_armed:
            self.mavros.set_velocity(0.0, 0.0, 0.0, 0.0)
            self.get_logger().warn(
                f'[AL] APPROACH — not armed (conf={det.confidence:.2f}), '
                f'holding',
                throttle_duration_sec=1.0
            )
            return SequenceStatus.INITIATED

        # ── Errors ────────────────────────────────────────────────────
        error_x = K_IMAGE_HEIGHT / 2.0 - det.cy    # → body vx
        error_y = K_IMAGE_WIDTH / 2.0 - det.cx     # → body vy

        scale = clamp(self.up / self.flight_alt, 0.2, 1.0)

        yaw_err = det.yaw

        centred = (abs(error_x) <= K_TOLERANCE_PX and
                   abs(error_y) <= K_TOLERANCE_PX)
        yaw_ok = abs(yaw_err) <= K_YAW_TOLERANCE_RAD

        # ── Yaw runs always — independent axis ────────────────────────
        yaw_rate = clamp(K_YAW_KP * yaw_err, -K_MAX_YAW_RATE, K_MAX_YAW_RATE)
        if yaw_ok:
            yaw_rate = 0.0

        # ── Translation: descend only when centred AND yaw-aligned ────
        vx = vy = vz = 0.0
        max_val = scale * K_MAX_VEL

        if not centred:
            vx = clamp(K_KP * scale * error_x, -max_val, max_val)
            vy = clamp(K_KP * scale * error_y, -max_val, max_val)
        elif yaw_ok:
            self.get_logger().info(
                f'[AL] CENTRED + YAW_OK at alt={self.up:.2f} — LAND')
            vz = K_DESCENT_VZ
        # else: centred but yaw still turning → hold still, let yaw finish

        # ── Rotate body → world (ENU) ─────────────────────────────────
        psi = self.yaw - self.camera_yaw
        vx_world = vx * math.cos(psi) - vy * math.sin(psi)
        vy_world = vx * math.sin(psi) + vy * math.cos(psi)

        self.mavros.set_velocity(vx_world, vy_world, vz, yaw_rate)

        self.get_logger().info(
            f'[AL] APPROACH {"CENTRED" if centred else "correcting"}'
            f'{" YAW_OK" if yaw_ok else " yawing"} | '
            f'px=({det.cx:.0f},{det.cy:.0f}) '
            f'err=({error_x:.0f},{error_y:.0f}) '
            f'yaw_err={math.degrees(yaw_err):.1f}° | '
            f'v_world=({vx_world:.2f},{vy_world:.2f}) vz={vz:.2f} '
            f'yaw_rate={yaw_rate:.2f} | conf={det.confidence:.2f} '
            f'alt={self.up:.2f} drone_yaw={math.degrees(self.yaw):.1f}°',
            throttle_duration_sec=0.5
        )

        return SequenceStatus.INITIATED

    # ──────────────────────────────────────────────────────────────────
    def on_exit(self):
        self.mavros.stop_velocity()
        self.get_logger().info('[AL] exited, velocity stopped')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
