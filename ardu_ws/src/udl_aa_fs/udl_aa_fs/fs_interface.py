#!/usr/bin/env python3
"""
FS Interface - fs_adapter. Sits between the flight commander and
vehicle_controller: translates incoming VehicleCommand messages into the
(token, command) shape the tick expects, and publishes back both the per-tick
outcome and the vehicle telemetry snapshot.

Three endpoints, all created on vehicle_controller's node:

  sub  /fs_adapter/command   VehicleCommand    RELIABLE,    KEEP_LAST(10)
  pub  /fs_adapter/status    VehicleStatus     BEST_EFFORT, KEEP_LAST(1)
  pub  /vehicle/telemetry    VehicleTelemetry  BEST_EFFORT, KEEP_LAST(1)

Commands are RELIABLE because dropping one loses an operator intent. Status and
telemetry are BEST_EFFORT KEEP_LAST(1) because both are latest-value-wins state
broadcasts rather than event streams - a late subscriber wants the current
state, never the backlog.

No thread of its own: the subscription callback and vehicle_controller's tick
both run on the same single executor thread, so the pending command is a plain
synchronous read, not an atomic cross-thread one.

Malformed messages are desk-rejected in the callback and never reach the fsm -
see _desk_check. They come back as cond=['MALFORMED'], deliberately not as the
fsm's SANITY, which would wrongly imply the values were merely out of bounds
rather than the message being unusable.
"""

from typing import Any, Optional, Tuple

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from udl_aa_fs import constants
from udl_aa_fs.constants import Command
from udl_aa_msgs.msg import VehicleCommand, VehicleStatus, VehicleTelemetry

COMMAND_TOPIC = '/vehicle/command'
STATUS_TOPIC = '/vehicle/status'
TELEMETRY_TOPIC = '/vehicle/telemetry'

# cond entry for a message rejected at the boundary, upstream of the fsm's four
# axes. Not one of them - it's a protocol error, not a flight-state decision.
MALFORMED = 'MALFORMED'

NOTHING_PENDING: Tuple[Any, Any] = (0, None)


class FsInterface:

    def __init__(self, mavros):
        self._mavros = mavros
        self._node = mavros.node

        self._pending: Tuple[Any, Any] = NOTHING_PENDING

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Latest-value-wins for both outbound topics.
        broadcast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._command_sub = self._node.create_subscription(
            VehicleCommand, COMMAND_TOPIC, self._command_callback, command_qos)
        self._status_pub = self._node.create_publisher(
            VehicleStatus, STATUS_TOPIC, broadcast_qos)
        self._telemetry_pub = self._node.create_publisher(
            VehicleTelemetry, TELEMETRY_TOPIC, broadcast_qos)

        self._node.get_logger().info('FS Interface initialized')

    # ------------------------------------------------------------------
    # Command intake
    # ------------------------------------------------------------------

    def _command_callback(self, msg: VehicleCommand):
        reason = _desk_check(msg)
        if reason is not None:
            self._node.get_logger().warn(
                f'desk-rejected {msg.command!r} from {msg.sequence!r} '
                f'token={msg.token}: {reason}')
            self._publish_status(
                VehicleStatus.REJECTED, [MALFORMED], None, msg.token)
            return

        self._pending = (msg.token, Command(
            name=msg.command,
            params=dict(zip(msg.param_names, msg.param_values)),
        ))

    def fs_adapter_atomic(self) -> Tuple[Any, Any]:
        """
        (token, command), consumed on read - a submission is arbitrated exactly
        once. Nothing is held between ticks: leaving it pending would re-enter
        the same command into arbitration every tick and re-dispatch it at the
        full tick rate. If the fsm refuses it, the commander sees why in
        VehicleStatus and decides whether to resubmit.
        """
        pending, self._pending = self._pending, NOTHING_PENDING
        return pending

    # ------------------------------------------------------------------
    # Outbound
    # ------------------------------------------------------------------

    def report(self, cmd_status, cond, command, token):
        """
        Fires every tick without exception - a heartbeat, not a notification -
        so the commander stays aware of current state even on ticks where
        nothing was submitted and nothing happened.
        """
        self._publish_status(cmd_status, cond, command, token)

    def _publish_status(self, cmd_status, cond, command, token):
        msg = VehicleStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.cmd_status = cmd_status
        msg.cond = list(cond)
        # ROS has no null: absent command and absent token become '' and 0.
        msg.command = command.name if command is not None else ''
        msg.token = int(token) if token is not None else 0
        self._status_pub.publish(msg)

    def publish_telemetry(self):
        """
        On its own timer (telemetry_rate_hz), not the tick - the fastest MAVROS
        group feeding get_telemetry() measures around 3Hz, so publishing at tick
        rate would emit several byte-identical messages per new datum.
        """
        t = self._mavros.get_telemetry()

        msg = VehicleTelemetry()
        msg.header.stamp = self._node.get_clock().now().to_msg()

        msg.state.armed = bool(t['state']['armed'])
        msg.state.mode = t['state']['mode']
        msg.state.in_air = bool(t['state']['in_air'])

        _fill_global(msg.home, t['home'])
        _fill_global(msg.global_position, t['global_position'])

        lp = t['local_position']
        msg.local_position.east = lp['east']
        msg.local_position.north = lp['north']
        msg.local_position.up = lp['up']
        msg.local_position.yaw_deg = lp['yaw_deg']

        _fill_body_velocity(msg.velocity_body, t['velocity_body'])
        _fill_body_velocity(msg.velocity_body_odom, t['velocity_body_odom'])

        vg = t['velocity_gps']
        msg.velocity_gps.east = vg['east']
        msg.velocity_gps.north = vg['north']
        msg.velocity_gps.up = vg['up']

        imu = t['imu']
        msg.imu.yaw_deg = imu['yaw_deg']
        msg.imu.gyro_right = imu['gyro_right']
        msg.imu.gyro_forward = imu['gyro_forward']
        msg.imu.gyro_up = imu['gyro_up']
        msg.imu.accel_right = imu['accel_right']
        msg.imu.accel_forward = imu['accel_forward']
        msg.imu.accel_up = imu['accel_up']

        bat = t['battery']
        msg.battery.voltage = bat['voltage']
        msg.battery.current = bat['current']
        msg.battery.percentage = bat['percentage']

        gps = t['gps_status']
        msg.gps_status.fix_type = int(gps['fix_type'])
        msg.gps_status.satellites_visible = int(gps['satellites_visible'])
        msg.gps_status.eph = int(gps['eph'])
        msg.gps_status.epv = int(gps['epv'])

        msg.heading_deg = t['heading_deg']

        hud = t['vfr_hud']
        msg.vfr_hud.airspeed = hud['airspeed']
        msg.vfr_hud.groundspeed = hud['groundspeed']
        msg.vfr_hud.heading_deg = hud['heading_deg']
        msg.vfr_hud.throttle = float(hud['throttle'])
        msg.vfr_hud.altitude = hud['altitude']
        msg.vfr_hud.climb = hud['climb']

        rc = t['rc']
        msg.rc.channels = [int(c) for c in rc['channels']]
        msg.rc.rssi = int(rc['rssi'])

        st = t['statustext']
        msg.statustext.severity = int(st['severity'])
        msg.statustext.text = st['text']

        self._telemetry_pub.publish(msg)


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _desk_check(msg: VehicleCommand) -> Optional[str]:
    """
    Returns a reason string if the message is unusable, else None.

    Everything checked here is a protocol error - something the type system
    cannot express, because param_names and param_values are independently
    valid arrays whatever they contain. A well-formed command carrying bad
    *values* is not our business; that's the fsm's sanity axis.
    """
    if len(msg.param_names) != len(msg.param_values):
        return f'{len(msg.param_names)} names vs {len(msg.param_values)} values'

    expected = constants.COMMAND_PARAMS.get(msg.command)
    if expected is None:
        return 'unknown command'

    names = list(msg.param_names)
    if len(set(names)) != len(names):
        return 'duplicate parameter names'

    if set(names) != expected:
        return (f'params missing={sorted(expected - set(names))} '
                f'unexpected={sorted(set(names) - expected)}')

    return None


def _fill_global(field, src):
    field.lat = src['lat']
    field.lon = src['lon']
    field.alt = src['alt']


def _fill_body_velocity(field, src):
    field.right = src['right']
    field.forward = src['forward']
    field.up = src['up']
    field.yaw_rate = src['yaw_rate']
