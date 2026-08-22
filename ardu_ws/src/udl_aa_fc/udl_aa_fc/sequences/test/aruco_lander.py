"""ArUco landing: scan for the marker, settle over it, servo down onto it.

Translation of ArucoLandingSequence.cpp/.hpp. The three C++ phases keep every
constant, threshold and branch of the original and are meant to be readable
side by side with it; PREFLIGHT and TRANSIT are additions, marked as such
where their constants are defined.

        PREFLIGHT             SCAN              HOVER_CAPTURE
   arm, climb, reach     fly the rectangle    return to where the
   waypoint 0        --> until the marker --> marker was seen and  --.
                         shows up in frame    wait until stopped     |
                                                                     |
             APPROACH                    TRANSIT                     |
        pixel servo down,        fly the bearing the marker          |
        hand off to LAND    <--  lay on, until it is back    <-------'
        under kLandAltM          in frame or 10s elapse

What is not literal is the environment, because there is no VehicleController
object here to call. Commands go out as VehicleCommand messages over
CommandLink, the same way as every other script in this folder, and telemetry
comes back off /vehicle/telemetry rather than from getTelemetry():

    GotoENUCmd{east, north, up, 0.0}  ->  GOTO_LOCAL {east, north, up, yaw_deg}
    SetVelocityCmd{vx, vy, vz, rate}  ->  SET_VELOCITY {east, north, up, yaw_rate}
    StopVelocityCmd{} / LandCmd{}     ->  STOP_VELOCITY / LAND
    telem.x / .y / .z / .yaw          ->  local_position.east / .north / .up
                                          / radians(.yaw_deg)
    telem.vx / .vy                    ->  velocity_body_odom.right / .forward

The last one is the only mapping that is not exact: those two are body-frame
here and world-frame there. It does not matter, because the single use is
hypot(vx, vy) and a magnitude is frame independent.

Three consequences of the transport, none of which change the logic:

  Position setpoints go out on change rather than every tick. The C++ called
  GotoENUCmd from every SCAN and HOVER_CAPTURE tick; ArduPilot latches a GUIDED
  position target, so re-sending an unchanged one does nothing at the vehicle -
  but mavros_interface.goto_local publishes three times with a spin between
  each, so it would cost the vehicle's 20 Hz tick up to 0.3 s per call. Every
  other command, including the zero-velocity hold in the unarmed branch, is
  issued every tick exactly as the C++ issues it.

  The tick runs at 5 Hz, matching the telemetry publish rate. Faster would
  re-decide on a sample it has already seen, and velocity setpoints only need
  to beat ArduPilot's 3 s GUID_TIMEOUT.

  No mutex around the detection window. The C++ had one because its callback
  ran on an executor thread while update() ran on another; here the callback
  fires inside spin_once, between ticks, never during one.

The handoff stops the servo and then lands, in that order and back to back -
the vehicle is locked and hovering at K_LAND_ALT_M, and what should be left
standing at it is a stop rather than the descent the servo was holding.

on_exit() still sends its own STOP_VELOCITY, as the C++ does, and the finally
block guarantees it whichever way the run ends. After a successful landing the
autopilot is in LAND, so that one is refused at the fsm's first gate - outside
GUIDED the stack injects nothing - and the vehicle logs nothing about it. It is
kept for ctrl-C, which is the case it actually exists for.

Open loop about outcomes everywhere except LAND, which is confirmed end to end:
accepted on the status topic under its own token, then landed and disarmed on
telemetry, re-issued if unacknowledged and abandoned if refused. See the LAND
handshake constants for why that one is different. The script exits non-zero if
the landing cannot be confirmed.

The one precondition the script cannot handle itself is GUIDED. Arming, the
climb and the flight to the first waypoint are PREFLIGHT's job, but
SET_MODE_GUIDED is unreachable from a sequence by design - it has to come from
the GCS or the pilot - so the run needs the vehicle already in it.
"""

import dataclasses
import math
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from vision_msgs.msg import Detection2DArray
from udl_aa_msgs.msg import VehicleStatus

from udl_aa_fc import vocabulary
from udl_aa_fc.command_link import STATE_QOS, TOPIC_STATUS, CommandLink

SEQUENCE = 'aruco_lander'

# rclcpp::QoS(5).best_effort(), transcribed. The detector publishes BEST_EFFORT
# and a RELIABLE subscriber would not match it at all.
DETECTION_TOPIC = '/detections/aruco'
DETECTION_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# --- LAND handshake ---------------------------------------------------------
# Nothing in the C++ corresponds to this: there LandCmd was a call on an
# in-process object and arrival was not in question. Here it is a message, and
# fs_adapter holds exactly one pending command which the vehicle drains once
# per 20 Hz tick - so a LAND followed inside 50 ms by anything else is
# overwritten and never dispatched, silently, since nothing rejected it. That
# is precisely what STOP_VELOCITY from on_exit was doing to it.
#
# The general rule this implements: a command is issued, its acceptance is
# confirmed against the status topic, and its effect is confirmed against
# telemetry before the sequence exits. LAND only - the velocity setpoints fire
# continuously and are self-correcting, so confirming each one would cost a
# round trip per tick to learn something the next tick supersedes.
LAND_ACK_TIMEOUT_S = 1.0      # 20 vehicle ticks; the echo comes on one of them
LAND_ACK_ATTEMPTS = 3
LAND_CONFIRM_TIMEOUT_S = 10.0

# --- Preflight --------------------------------------------------------------
# Not in the C++ either, where the sequence was started by an operator who had
# already put the vehicle where it needed to be. This is a test script, so it
# gets itself airborne: arm if disarmed, take off to `altitude`, fly to the
# first scan waypoint, then hand over to SCAN.
#
# The three steps are the permission table read forwards - ARM is only legal
# disarmed on the ground, TAKEOFF only armed on the ground, GOTO_LOCAL only
# armed and in the air - so each step is also what makes the next one legal.
# GUIDED is still a precondition: SET_MODE_GUIDED is unreachable from here by
# design, and has to come from the GCS or the pilot.
K_ARM_SETTLE_S = 3.0          # ArduPilot refuses TAKEOFF the instant arming completes
K_TAKEOFF_TOLERANCE_M = 0.5   # close enough to the target altitude to move on
K_PREFLIGHT_TIMEOUT_S = 60.0  # arm plus a full climb, with room to spare

# --- Transit ----------------------------------------------------------------
# Not in the C++. There, first detection recorded only the vehicle's own
# position, and HOVER_CAPTURE flew back to it - which works while the marker is
# near frame centre and fails when it is at the edge, because the point it
# returns to is one the marker is already leaving. The bounding box says which
# way the marker lies; the vehicle's yaw at that instant turns that into an ENU
# direction. TRANSIT flies it until the marker comes back into frame.
#
# A direction only. Nothing here estimates how far away the marker is.
K_TRANSIT_VEL_MPS = 0.5     # slow - the marker has to come back into frame
K_TRANSIT_TIMEOUT_S = 10.0

# --- ALPhase ----------------------------------------------------------------
PREFLIGHT = 'PREFLIGHT'
SCAN = 'SCAN'
HOVER_CAPTURE = 'HOVER_CAPTURE'
TRANSIT = 'TRANSIT'
APPROACH = 'APPROACH'

# PREFLIGHT's own steps. One command each, then watch telemetry for its effect
# - the same shape as the phases above, one level down.
PRE_ARM = 'arm'
PRE_TAKEOFF = 'takeoff'
PRE_WAYPOINT = 'waypoint'

# --- SequenceStatus ---------------------------------------------------------
INITIATED = 'INITIATED'
COMPLETE = 'COMPLETE'

# --- Constants, transcribed from ArucoLandingSequence.hpp --------------------
K_WP_TOLERANCE_M = 1.5

# Servo params (from proven follow_node)
K_IMAGE_WIDTH = 640.0
K_IMAGE_HEIGHT = 480.0
K_KP = 0.005
K_MAX_VEL = 1.0
K_TOLERANCE_PX = 50.0
K_CAMERA_YAW_DEG = -90.0

K_WINDOW_SIZE = 15
K_MAX_AGE = 0.5      # s - recency floor
K_MAX_JUMP = 150.0   # px - coherence floor

# Hover-capture settle criteria
K_SETTLE_DIST_M = 0.5
K_SETTLE_SPEED_MPS = 0.2

# Confidence gating (placeholder - regress against logs)
K_CONF_ARM = 0.20
K_CONF_ABANDON = 0.10

K_YAW_KP = 0.5
K_MAX_YAW_RATE = 0.3        # rad/s ~ 17 deg/s
K_YAW_TOLERANCE_RAD = 0.087  # ~5 deg

K_DESCENT_VZ = -0.3  # m/s, negative = down
K_LAND_ALT_M = 0.6


@dataclasses.dataclass
class Waypoint:
    """struct Waypoint { double east; double north; double up; };"""

    east: float
    north: float
    up: float


@dataclasses.dataclass
class DetectionEntry:
    hit: bool = False
    cx: float = 0.0
    cy: float = 0.0
    yaw: float = 0.0
    stamp: object = None


@dataclasses.dataclass
class DetectionSnapshot:
    valid: bool = False  # is there any hit in the window
    cx: float = 0.0      # newest hit
    cy: float = 0.0
    yaw: float = 0.0
    class_id: str = ''
    stamp: object = None
    confidence: float = 0.0
    # components, logged separately
    recency: float = 0.0
    hit_rate: float = 0.0
    coherence: float = 0.0


@dataclasses.dataclass
class Telemetry:
    """flight_stack::TelemetryData, as far as this sequence uses it.

    Assembled in one place so the phase handlers below read the same way the
    C++ ones do. x/y/z are ENU metres from the local origin, yaw is radians
    CCW from East, vx/vy are a horizontal velocity pair used only as a
    magnitude.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0


class ArucoLandingSequence:

    def __init__(self, node, link, flight_alt_m=10.0):
        self._node = node
        self._link = link
        self._flight_alt = flight_alt_m

        self._phase = SCAN
        self._wp_index = 0
        self._servo_armed = False

        self._capture_east = 0.0
        self._capture_north = 0.0
        self._capture_up = 0.0

        # Which way the marker lay at first detection, as an ENU unit vector.
        # Frozen at that instant: the gotos command an absolute yaw_deg=0, so
        # the vehicle rotates on its way back to the capture point, and a
        # direction held in the camera frame would rotate with it.
        self._bearing_east = 0.0
        self._bearing_north = 0.0
        self._transit_deadline = 0.0

        # Preflight. _pre_sent is what keeps ARM and TAKEOFF one-shot: they
        # have no publish-on-change equivalent, and re-issuing either every
        # tick would overwrite submissions the vehicle has not arbitrated yet.
        self._pre_step = PRE_ARM
        self._pre_sent = False
        self._pre_deadline = 0.0
        self._arm_commanded = False
        self._arm_settle_at = None

        self._detection_window = deque()
        self._last_class_id = ''
        self._detection_sub = None

        self._token = 0
        self._goto_published = None
        self._last_log = {}

        # LAND handshake. Read by main to decide the exit code.
        self._status_sub = None
        self._land_token = 0
        self._land_status = None
        self.land_confirmed = False

        # Scan rectangle centered on marker area (marker at E=7, N=0).
        # Corners give ~10m margin so every leg has line-of-sight to the
        # marker. Transcribed field for field from the C++ brace-init, which
        # is {east, north, up} - note its inline comments read the first two
        # the other way round.
        self._waypoints = [
            Waypoint(-10.0, 7.0, flight_alt_m),
            Waypoint(10.0, 7.0, flight_alt_m),
            Waypoint(10.0, 0.0, flight_alt_m),
            Waypoint(-10.0, 0.0, flight_alt_m),
        ]

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_start(self):
        self._phase = PREFLIGHT
        self._pre_step = PRE_ARM
        self._pre_sent = False
        self._pre_deadline = (self._node.get_clock().now().nanoseconds
                              + K_PREFLIGHT_TIMEOUT_S * 1e9)
        self._wp_index = 0
        self._servo_armed = False

        # Subscribe to detector topic
        self._detection_sub = self._node.create_subscription(
            Detection2DArray, DETECTION_TOPIC, self._detection_callback,
            DETECTION_QOS)

        # Second subscriber on the topic CommandLink already watches, because
        # CommandLink keeps only refusals - an ACCEPTED never survives its
        # callback. Same QoS object, so the two cannot drift apart.
        self._status_sub = self._node.create_subscription(
            VehicleStatus, TOPIC_STATUS, self._status_callback, STATE_QOS)

        self._node.get_logger().info(
            f'[AL] started - PREFLIGHT phase, target altitude '
            f'{self._flight_alt:.1f}m, {len(self._waypoints)} scan waypoints')

    def on_exit(self):
        self._stop_velocity()
        if self._detection_sub is not None:
            self._node.destroy_subscription(self._detection_sub)
            self._detection_sub = None
        if self._status_sub is not None:
            self._node.destroy_subscription(self._status_sub)
            self._status_sub = None
        self._node.get_logger().info('[AL] exited')

    def update(self):
        if self._phase == PREFLIGHT:
            return self._tick_preflight()
        if self._phase == SCAN:
            return self._tick_scan()
        if self._phase == HOVER_CAPTURE:
            return self._tick_hover_capture()
        if self._phase == TRANSIT:
            return self._tick_transit()
        if self._phase == APPROACH:
            return self._tick_approach()
        return INITIATED

    # ------------------------------------------------------------------
    # Preflight
    # ------------------------------------------------------------------

    def _tick_preflight(self):
        """Arm, climb, and reach the first waypoint. Then SCAN takes over.

        Each step issues its command once and then watches telemetry for the
        effect, because the vehicle reports that a command was dispatched and
        never that it finished. Whatever is already true is skipped: an armed
        vehicle in the air goes straight to the waypoint.
        """
        state = self._link.telemetry().state
        telem = self._get_telemetry()

        if self._node.get_clock().now().nanoseconds > self._pre_deadline:
            self._node.get_logger().error(
                f'[AL] PREFLIGHT timed out in step {self._pre_step!r} after '
                f'{K_PREFLIGHT_TIMEOUT_S:.0f}s - armed={state.armed} '
                f'in_air={state.in_air} alt={telem.z:.2f} mode={state.mode!r}')
            return COMPLETE

        if self._pre_step == PRE_ARM:
            return self._pre_tick_arm(state)
        if self._pre_step == PRE_TAKEOFF:
            return self._pre_tick_takeoff(state, telem)
        return self._pre_tick_waypoint(telem)

    def _pre_tick_arm(self, state):
        if not state.armed:
            if not self._pre_sent:
                self._node.get_logger().info('[AL] PREFLIGHT - arming')
                self._publish(vocabulary.ARM)
                self._pre_sent = True
                self._arm_commanded = True
            return INITIATED

        # Armed. Settle only if we armed it just now: ArduPilot refuses a
        # TAKEOFF issued the instant arming completes. A vehicle that was
        # already armed when we started has settled long ago.
        if not self._arm_commanded:
            self._enter_pre_step(PRE_TAKEOFF)
            return INITIATED

        if self._arm_settle_at is None:
            self._arm_settle_at = (self._node.get_clock().now().nanoseconds
                                   + K_ARM_SETTLE_S * 1e9)
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - armed, settling {K_ARM_SETTLE_S:.0f}s')
        if self._node.get_clock().now().nanoseconds >= self._arm_settle_at:
            self._enter_pre_step(PRE_TAKEOFF)
        return INITIATED

    def _pre_tick_takeoff(self, state, telem):
        if telem.z >= self._flight_alt - K_TAKEOFF_TOLERANCE_M:
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - at altitude {telem.z:.2f}m')
            self._enter_pre_step(PRE_WAYPOINT)
            return INITIATED

        # Airborne but low: TAKEOFF is not permitted in the air, and the
        # waypoint carries its own altitude, so the climb happens there.
        if state.in_air and not self._pre_sent:
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - already airborne at {telem.z:.2f}m, '
                'climbing on the waypoint')
            self._enter_pre_step(PRE_WAYPOINT)
            return INITIATED

        if not self._pre_sent:
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - takeoff to {self._flight_alt:.1f}m')
            self._publish(vocabulary.TAKEOFF, {'altitude': self._flight_alt})
            self._pre_sent = True

        if self._throttled('preflight', 1.0):
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - climbing {telem.z:.2f}/'
                f'{self._flight_alt:.1f}m')
        return INITIATED

    def _pre_tick_waypoint(self, telem):
        wp = self._waypoints[0]
        self._goto_enu(wp.east, wp.north, wp.up, 0.0)

        if self._throttled('preflight', 1.0):
            self._node.get_logger().info(
                f'[AL] PREFLIGHT - to waypoint 0 (E{wp.east:.1f} '
                f'N{wp.north:.1f} U{wp.up:.1f}) pos=(E{telem.x:.2f} '
                f'N{telem.y:.2f} U{telem.z:.2f})')

        if self._waypoint_reached(wp, telem):
            self._node.get_logger().info(
                '[AL] PREFLIGHT - waypoint 0 reached - SCAN')
            self._phase = SCAN
        return INITIATED

    def _enter_pre_step(self, step):
        """Move to the next step, re-arming the one-shot publish."""
        self._pre_step = step
        self._pre_sent = False

    # ------------------------------------------------------------------
    # Scan
    # ------------------------------------------------------------------

    def _tick_scan(self):
        telem = self._get_telemetry()
        det = self._snapshot_detection()

        # Count consecutive detections for debounce
        if det.valid:
            self._capture_east = telem.x
            self._capture_north = telem.y
            self._capture_up = telem.z
            self._capture_bearing(det, telem)
            self._node.get_logger().info(
                f'[AL] marker detected (id={det.class_id} cx={det.cx:.1f} '
                f'cy={det.cy:.1f}) - HOVER_CAPTURE at '
                f'(E{self._capture_east:.2f} N{self._capture_north:.2f} '
                f'U{self._capture_up:.2f}) bearing='
                f'{self._bearing_deg():.1f} deg '
                f'(E{self._bearing_east:+.2f} N{self._bearing_north:+.2f}) '
                f'drone_yaw={math.degrees(telem.yaw):.1f} deg')
            self._phase = HOVER_CAPTURE
            return INITIATED

        # Drive toward current waypoint
        wp = self._waypoints[self._wp_index]
        self._goto_enu(wp.east, wp.north, wp.up, 0.0)

        if self._throttled('scan', 1.0):
            self._node.get_logger().info(
                f'[AL] SCAN wp={self._wp_index} target=(E{wp.east:.1f} '
                f'N{wp.north:.1f} U{wp.up:.1f}) pos=(E{telem.x:.2f} '
                f'N{telem.y:.2f})')

        # Advance waypoint when close enough
        if self._waypoint_reached(wp, telem):
            self._wp_index = (self._wp_index + 1) % len(self._waypoints)
            self._node.get_logger().info(
                f'[AL] SCAN - advancing to waypoint {self._wp_index}')

        return INITIATED

    # ------------------------------------------------------------------
    # Hover capture
    # ------------------------------------------------------------------

    def _tick_hover_capture(self):
        # Hold the position captured at first detection. This overrides the
        # scan waypoint, so the drone returns to where it saw the marker
        # rather than wherever momentum carried it.
        self._goto_enu(self._capture_east, self._capture_north,
                       self._capture_up, 0.0)

        telem = self._get_telemetry()
        det = self._snapshot_detection()

        dist = math.hypot(telem.x - self._capture_east,
                          telem.y - self._capture_north)
        speed = math.hypot(telem.vx, telem.vy)

        if self._throttled('hover_capture', 0.5):
            self._node.get_logger().info(
                f'[AL] HOVER_CAPTURE - dist={dist:.2f} speed={speed:.2f} | '
                f'conf={det.confidence:.3f} (rec={det.recency:.2f} '
                f'hit={det.hit_rate:.2f} coh={det.coherence:.2f}) '
                f'cx={det.cx:.1f} cy={det.cy:.1f} yaw={det.yaw:.2f} '
                f'({math.degrees(det.yaw):.1f} deg)')

        # Settled at the capture point - now fly the bearing
        if dist < K_SETTLE_DIST_M and speed < K_SETTLE_SPEED_MPS:
            self._node.get_logger().info(
                f'[AL] settled (dist={dist:.2f} speed={speed:.2f} '
                f'conf={det.confidence:.2f}) - TRANSIT')
            self._phase = TRANSIT
            self._transit_deadline = (self._node.get_clock().now().nanoseconds
                                      + K_TRANSIT_TIMEOUT_S * 1e9)

        return INITIATED

    # ------------------------------------------------------------------
    # Transit
    # ------------------------------------------------------------------

    def _tick_transit(self):
        """Fly the bearing captured at first detection until the marker is back.

        Open loop by construction: the bearing was frozen at first detection
        and nothing here refines it, because there is nothing to refine it
        with - the marker is out of frame, which is why we are flying at all.
        The moment it returns, the pixel servo has real feedback and this
        phase has nothing left to offer.
        """
        det = self._snapshot_detection()

        if det.valid:
            self._node.get_logger().info(
                f'[AL] marker reacquired (cx={det.cx:.1f} cy={det.cy:.1f} '
                f'conf={det.confidence:.2f}) - APPROACH')
            # No STOP_VELOCITY on the way out: APPROACH commands its own
            # velocity on its first tick, and two commands inside one 20 Hz
            # vehicle tick means the first is dropped in the pending slot.
            self._phase = APPROACH
            return INITIATED

        remaining_s = (self._transit_deadline
                       - self._node.get_clock().now().nanoseconds) * 1e-9
        if remaining_s <= 0.0:
            self._node.get_logger().warn(
                f'[AL] TRANSIT - {K_TRANSIT_TIMEOUT_S:.0f}s flown without '
                'reacquiring the marker - APPROACH')
            self._phase = APPROACH
            return INITIATED

        east = K_TRANSIT_VEL_MPS * self._bearing_east
        north = K_TRANSIT_VEL_MPS * self._bearing_north
        # up=0 holds altitude, yaw_rate=0 holds heading - rotating is one of
        # the things that lost the marker in the first place.
        self._set_velocity(east, north, 0.0, 0.0)

        if self._throttled('transit', 0.5):
            telem = self._get_telemetry()
            self._node.get_logger().info(
                f'[AL] TRANSIT bearing={self._bearing_deg():.1f} deg | '
                f'v_world=({east:.2f},{north:.2f}) | {remaining_s:.1f}s left | '
                f'pos=(E{telem.x:.2f} N{telem.y:.2f}) alt={telem.z:.2f} '
                f'conf={det.confidence:.2f}')

        return INITIATED

    # ------------------------------------------------------------------
    # Approach
    # ------------------------------------------------------------------

    def _tick_approach(self):
        det = self._snapshot_detection()
        telem = self._get_telemetry()

        # -- Low enough - hand off to autopilot LAND --------------------
        if telem.z <= K_LAND_ALT_M:
            self._node.get_logger().info(
                f'[AL] alt={telem.z:.2f} - handing off to LAND')
            # Locked and hovering at K_LAND_ALT_M: stop the servo, then land.
            # The last thing standing at the vehicle should be a stop, not the
            # descent setpoint the servo was holding.
            self._stop_velocity()
            # Blocks until the vehicle is down, refused, or out of time. The
            # sequence has nothing left to do, so there is nothing for the
            # blocking to starve - and returning COMPLETE while LAND is still
            # unconfirmed is the failure this exists to prevent.
            self.land_confirmed = self._land_and_confirm()
            return COMPLETE

        # -- Confidence gate with hysteresis ---------------------------
        if not self._servo_armed and det.confidence >= K_CONF_ARM:
            self._servo_armed = True
            self._node.get_logger().info(
                f'[AL] servo ARMED (conf={det.confidence:.2f})')
        elif self._servo_armed and det.confidence < K_CONF_ABANDON:
            self._servo_armed = False
            self._node.get_logger().warn(
                f'[AL] servo ABANDONED (conf={det.confidence:.2f})')

        if not self._servo_armed:
            self._set_velocity(0.0, 0.0, 0.0, 0.0)
            if self._throttled('unarmed', 1.0):
                self._node.get_logger().warn(
                    f'[AL] APPROACH - not armed (conf={det.confidence:.2f}), '
                    'holding')
            return INITIATED

        # -- Errors ----------------------------------------------------
        error_x = K_IMAGE_HEIGHT / 2.0 - det.cy   # -> body vx
        error_y = K_IMAGE_WIDTH / 2.0 - det.cx    # -> body vy

        scale = _clamp(telem.z / self._flight_alt, 0.2, 1.0)

        # Square marker: any 90 deg rotation is equally aligned. Wrap to +-45.
        # yaw_err = math.remainder(det.yaw, math.pi / 2.0)
        yaw_err = det.yaw

        centred = (abs(error_x) <= K_TOLERANCE_PX
                   and abs(error_y) <= K_TOLERANCE_PX)
        yaw_ok = abs(yaw_err) <= K_YAW_TOLERANCE_RAD

        # -- Yaw runs always - independent axis, doesn't fight translation --
        yaw_rate = _clamp(K_YAW_KP * yaw_err, -K_MAX_YAW_RATE, K_MAX_YAW_RATE)
        if yaw_ok:
            yaw_rate = 0.0

        # -- Translation: descend only when centred AND yaw-aligned ----
        vx = vy = vz = 0.0

        max_val = scale * K_MAX_VEL   # scale max velocity with altitude

        if not centred:
            vx = _clamp(K_KP * scale * error_x, -max_val, max_val)
            vy = _clamp(K_KP * scale * error_y, -max_val, max_val)
        elif yaw_ok:
            self._node.get_logger().info(
                f'[AL] CENTRED + YAW_OK at alt={telem.z:.2f} - LAND')
            vz = K_DESCENT_VZ
        # else: centred but yaw still turning - hold still, let yaw finish

        # -- Rotate body -> world (ENU) --------------------------------
        vx_world, vy_world = _camera_to_enu(vx, vy, telem.yaw)

        self._set_velocity(vx_world, vy_world, vz, yaw_rate)

        if self._throttled('approach', 0.5):
            self._node.get_logger().info(
                f'[AL] APPROACH {"CENTRED" if centred else "correcting"}'
                f'{" YAW_OK" if yaw_ok else " yawing"} | '
                f'px=({det.cx:.0f},{det.cy:.0f}) '
                f'err=({error_x:.0f},{error_y:.0f}) '
                f'yaw_err={math.degrees(yaw_err):.1f} deg | '
                f'v_world=({vx_world:.2f},{vy_world:.2f}) vz={vz:.2f} '
                f'yaw_rate={yaw_rate:.2f} | conf={det.confidence:.2f} '
                f'alt={telem.z:.2f} drone_yaw={math.degrees(telem.yaw):.1f} deg')

        return INITIATED

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _capture_bearing(self, det, telem):
        """Freeze which way the marker lies, as an ENU unit vector.

        Same pixel errors the servo works on, through the same camera->ENU
        rotation, so the camera calibration angle is applied to the transit
        exactly as it is to every other velocity command. Only the magnitude
        is discarded: this says which way, never how far.
        """
        forward = K_IMAGE_HEIGHT / 2.0 - det.cy
        right = K_IMAGE_WIDTH / 2.0 - det.cx

        east, north = _camera_to_enu(forward, right, telem.yaw)
        norm = math.hypot(east, north)
        if norm < 1e-6:
            # Dead centre at the moment of detection - no direction to fly.
            # TRANSIT reads a zero bearing as a zero velocity command.
            self._bearing_east = self._bearing_north = 0.0
            return

        self._bearing_east = east / norm
        self._bearing_north = north / norm

    def _bearing_deg(self):
        """The stored bearing as a compass-free angle, CCW positive from East."""
        return math.degrees(math.atan2(self._bearing_north, self._bearing_east))

    def _waypoint_reached(self, wp, telem):
        # telem.x = east, telem.y = north, telem.z = up (ENU local frame)
        de = telem.x - wp.east
        dn = telem.y - wp.north
        return (de * de + dn * dn) < (K_WP_TOLERANCE_M * K_WP_TOLERANCE_M)

    def _detection_callback(self, msg: Detection2DArray):
        e = DetectionEntry(stamp=self._node.get_clock().now())

        if msg.detections and msg.detections[0].results:
            d = msg.detections[0]
            e.hit = True
            e.cx = d.bbox.center.position.x
            e.cy = d.bbox.center.position.y

            q = d.results[0].pose.pose.orientation
            e.yaw = 2.0 * math.atan2(q.z, q.w)

            self._last_class_id = d.results[0].hypothesis.class_id

        self._detection_window.append(e)
        while len(self._detection_window) > K_WINDOW_SIZE:
            self._detection_window.popleft()

    def _snapshot_detection(self):
        s = DetectionSnapshot()

        if not self._detection_window:
            return s   # all zeros, valid=False

        # -- Collect hits, newest last ---------------------------------
        hits = [e for e in self._detection_window if e.hit]

        # -- hit_rate --------------------------------------------------
        s.hit_rate = len(hits) / K_WINDOW_SIZE

        if not hits:
            return s   # recency = coherence = 0, conf = 0

        newest = hits[-1]
        s.valid = True
        s.cx = newest.cx
        s.cy = newest.cy
        s.yaw = newest.yaw
        s.stamp = newest.stamp
        s.class_id = self._last_class_id

        # -- recency ---------------------------------------------------
        age = (self._node.get_clock().now() - newest.stamp).nanoseconds * 1e-9
        s.recency = _clamp(1.0 - age / K_MAX_AGE, 0.0, 1.0)

        # -- coherence: worst jump between consecutive hits ------------
        if len(hits) < 2:
            s.coherence = 1.0   # single hit - nothing to contradict it
        else:
            max_jump = 0.0
            for prev, cur in zip(hits, hits[1:]):
                max_jump = max(max_jump,
                               math.hypot(cur.cx - prev.cx, cur.cy - prev.cy))
            s.coherence = _clamp(1.0 - max_jump / K_MAX_JUMP, 0.0, 1.0)

        s.confidence = s.recency * s.hit_rate * s.coherence
        return s

    # ------------------------------------------------------------------
    # LAND handshake
    # ------------------------------------------------------------------

    def _status_callback(self, msg: VehicleStatus):
        """Keep the one status that echoes the LAND we are waiting on.

        The topic is a per-tick heartbeat, so nearly every message is about
        nothing: token 0, no command. A token matches ours for exactly one
        message - the tick that arbitrated it - which is why this latches
        rather than expecting the wait loop to be looking at the right moment.

        Matched on token alone, not on command: a refusal from axis 3 or 4
        carries our token with an empty command, and a refusal because safety
        outranked us carries our token with safety's command instead of ours.
        Both are answers about our submission. The command is checked at the
        accept site, where it is meaningful.
        """
        if msg.token == 0 or msg.token != self._land_token:
            return
        self._land_status = (msg.cmd_status, msg.command, list(msg.cond))

    def _land_and_confirm(self):
        """Issue LAND until it is accepted, then confirm the vehicle is down.

        Silence and refusal are opposite cases and are treated as such. No echo
        means the submission never reached arbitration - the pending slot was
        overwritten, or the message was lost before a subscriber matched - so
        re-issuing is the right answer and costs nothing. A refusal means the
        vehicle considered it and said no; sending it again would just be
        refused again, so we stop and say why.
        """
        for attempt in range(1, LAND_ACK_ATTEMPTS + 1):
            self._land_status = None
            self._land_token = self._land()

            if not self._spin_until(lambda: self._land_status is not None,
                                    LAND_ACK_TIMEOUT_S):
                self._node.get_logger().warn(
                    f'[AL] LAND token={self._land_token} not acknowledged in '
                    f'{LAND_ACK_TIMEOUT_S:.1f}s - re-issuing '
                    f'(attempt {attempt}/{LAND_ACK_ATTEMPTS})')
                continue

            cmd_status, command, cond = self._land_status

            if cmd_status == VehicleStatus.ACCEPTED and command == vocabulary.LAND:
                self._node.get_logger().info(
                    f'[AL] LAND accepted (token={self._land_token})')
                return self._await_landed()

            self._node.get_logger().error(
                f'[AL] LAND {cmd_status} (token={self._land_token} '
                f'command={command or "none"} cond={cond}) - giving up')
            return False

        self._node.get_logger().error(
            f'[AL] LAND unacknowledged after {LAND_ACK_ATTEMPTS} attempts')
        return False

    def _await_landed(self):
        """Wait for the vehicle to be on the ground and disarmed."""
        if self._spin_until(self._landed, LAND_CONFIRM_TIMEOUT_S):
            self._node.get_logger().info('[AL] landed and disarmed')
            return True

        t = self._link.telemetry()
        self._node.get_logger().error(
            f'[AL] not down {LAND_CONFIRM_TIMEOUT_S:.0f}s after LAND was '
            f'accepted - mode={t.state.mode or "no link"} '
            f'armed={t.state.armed} in_air={t.state.in_air} '
            f'alt={t.local_position.up:.2f}')
        return False

    def _landed(self):
        """On the ground and disarmed, according to the vehicle.

        The mode test is not redundant. A dropped link reports the blank
        snapshot, whose armed and in_air are both False - the two fields this
        reads - so without it losing telemetry would read as a successful
        landing. No flight controller reports an empty mode.
        """
        t = self._link.telemetry()
        return (t.state.mode != ''
                and not t.state.armed
                and not t.state.in_air)

    def _spin_until(self, predicate, timeout_s):
        """Spin until predicate() or timeout. True if it came true in time.

        Checked before the first spin: the status echo may already have been
        latched by a callback that ran while we were publishing.
        """
        deadline = (self._node.get_clock().now().nanoseconds
                    + int(timeout_s * 1e9))
        while rclpy.ok():
            if predicate():
                return True
            if self._node.get_clock().now().nanoseconds >= deadline:
                return False
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return False

    def _get_telemetry(self):
        t = self._link.telemetry()
        return Telemetry(
            x=t.local_position.east,
            y=t.local_position.north,
            z=t.local_position.up,
            vx=t.velocity_body_odom.right,
            vy=t.velocity_body_odom.forward,
            yaw=math.radians(t.local_position.yaw_deg),
        )

    def _throttled(self, key, period_s):
        """True at most once per period_s - RCLCPP_*_THROTTLE, transcribed."""
        now = self._node.get_clock().now().nanoseconds
        last = self._last_log.get(key)
        if last is not None and now - last < period_s * 1e9:
            return False
        self._last_log[key] = now
        return True

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def _publish(self, command, params=None):
        self._token += 1
        self._link.publish(self._token, command, params)
        return self._token

    def _goto_enu(self, east, north, up, yaw_deg):
        """GotoENUCmd. Issued on change - see the module docstring."""
        target = (east, north, up, yaw_deg)
        if target == self._goto_published:
            return
        self._publish(vocabulary.GOTO_LOCAL, {
            'east': east,
            'north': north,
            'up': up,
            'yaw_deg': yaw_deg,
        })
        self._goto_published = target

    def _set_velocity(self, east, north, up, yaw_rate):
        """SetVelocityCmd. Issued every tick, as the C++ issues it."""
        self._publish(vocabulary.SET_VELOCITY, {
            'east': east,
            'north': north,
            'up': up,
            'yaw_rate': yaw_rate,
        })

    def _stop_velocity(self):
        """StopVelocityCmd. Logged here, at the one place it is sent from."""
        self._publish(vocabulary.STOP_VELOCITY)
        self._node.get_logger().info('[AL] velocity stopped')

    def _land(self):
        """LandCmd. Returns the token to match an acknowledgement against."""
        return self._publish(vocabulary.LAND)


def _clamp(value, low, high):
    return max(low, min(high, value))


def _camera_to_enu(forward, right, yaw_rad):
    """Rotate a camera-frame vector into ENU. Returns (east, north).

    psi folds both rotations into one: the vehicle's heading, less the
    camera's mounting offset within it. The single place the calibration
    angle is applied, so the transit and the approach servo cannot disagree
    about which way the camera points.
    """
    psi = yaw_rad - math.radians(K_CAMERA_YAW_DEG)
    east = forward * math.cos(psi) - right * math.sin(psi)
    north = forward * math.sin(psi) + right * math.cos(psi)
    return east, north


def sleep(node, seconds):
    """Spin for `seconds`, so callbacks keep running while we wait."""
    deadline = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Node(SEQUENCE)

    # flight_alt_m, which the C++ constructor also defaults to 5.0.
    altitude = node.declare_parameter('altitude', 10.0).value
    # Matched to the telemetry publish rate - see the module docstring.
    rate_hz = node.declare_parameter('rate_hz', 5.0).value

    link = CommandLink(node)
    sequence = ArucoLandingSequence(node, link, altitude)
    period_ns = int(1e9 / rate_hz)

    started = False
    exit_code = 0
    try:
        # VOLATILE profile: anything published before the vehicle has matched
        # us is not backfilled, it is simply gone.
        sleep(node, 2.0)
        if not link.live:
            node.get_logger().error(
                'vehicle link down - is the flight stack running?')
            return

        sequence.on_start()
        started = True

        next_tick = node.get_clock().now().nanoseconds
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = node.get_clock().now().nanoseconds
            if now < next_tick:
                continue
            next_tick = now + period_ns
            if sequence.update() == COMPLETE:
                if not sequence.land_confirmed:
                    exit_code = 1
                break
    finally:
        # The C++ framework calls on_exit when the sequence ends; here the
        # finally block is what guarantees it, ctrl-C included.
        if started:
            sequence.on_exit()
            # Let the stop reach the vehicle before the publisher goes away.
            sleep(node, 1.0)
        node.destroy_node()
        rclpy.shutdown()

    # After the cleanup, so on_exit still runs on a failed landing.
    if exit_code:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
