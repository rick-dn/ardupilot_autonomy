#!/usr/bin/env python3
"""
Vehicle Node - owns mavros_adapter and exposes it as two topics.

The whole package's vehicle access is here. A command that arrives is
dispatched; telemetry is republished on a timer. There is no tick, no
arbitration, no state machine - compare udl_aa_fs, which wraps the same
adapter in fsm_handler / safety_monitor / conditions and a 20 Hz tick.

  pub  /udl_dev/telemetry  VehicleTelemetry  BEST_EFFORT, KEEP_LAST(1)
  sub  /udl_dev/command    VehicleCommand    RELIABLE,    KEEP_LAST(10)

Namespaced /udl_dev/* rather than /vehicle/* so this can run alongside
udl_aa_fs without the two fighting over the same topics.

Deliberately absent: there is no VehicleStatus. A sequence gets no
acknowledgement and confirms what happened by watching telemetry. A command
this node cannot dispatch is logged here and is invisible to the sender -
acceptable for development, and the first thing to add back for flight.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from udl_aa_dev.mavros_adapter import MavrosAdapter
from udl_aa_msgs.msg import VehicleCommand, VehicleTelemetry

COMMAND_TOPIC = '/udl_dev/command'
TELEMETRY_TOPIC = '/udl_dev/telemetry'


# Command name -> MavrosAdapter method. The incoming param_names/param_values
# arrays are zipped into a dict and splatted in as keyword arguments, so the
# names here must match the adapter's signatures exactly.
COMMAND_TO_METHOD = {
    'ARM': 'arm',
    'DISARM': 'disarm',
    'SET_MODE_GUIDED': 'set_mode_guided',
    'TAKEOFF': 'takeoff',
    'GOTO_GLOBAL': 'goto_global',
    'GOTO_LOCAL': 'goto_local',
    'GOTO_BODY': 'goto_body',
    'SET_VELOCITY': 'set_velocity',
    'STOP_VELOCITY': 'stop_velocity',
    'SET_ACCEL': 'set_acceleration',
    'STOP_ACCEL': 'stop_acceleration',
    'LAND': 'land',
    'RTL': 'rtl',
}


class VehicleNode(Node):
    """Adapter in, topics out."""

    def __init__(self):
        super().__init__('vehicle_node')

        # Declares fc_state_timeout_s on this node as a side effect.
        self.adapter = MavrosAdapter(self)

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Latest-value-wins: a late subscriber wants the current state, never
        # the backlog.
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._command_sub = self.create_subscription(
            VehicleCommand, COMMAND_TOPIC, self._command_callback, command_qos)
        self._telemetry_pub = self.create_publisher(
            VehicleTelemetry, TELEMETRY_TOPIC, telemetry_qos)

        self.declare_parameter('telemetry_rate_hz', 5.0)
        telemetry_rate_hz = self.get_parameter('telemetry_rate_hz').value
        self._telemetry_timer = self.create_timer(
            1.0 / telemetry_rate_hz, self._publish_telemetry)

        self.declare_parameter('logger_rate_hz', 1.0)
        logger_rate_hz = self.get_parameter('logger_rate_hz').value
        self._logger_timer = self.create_timer(
            1.0 / logger_rate_hz, self._flight_logger)

        self.get_logger().info(
            f'Vehicle Node initialized, telemetry={telemetry_rate_hz}Hz')

    # ------------------------------------------------------------------
    # Inbound
    # ------------------------------------------------------------------

    def _command_callback(self, msg: VehicleCommand):
        """
        Dispatch straight to the adapter. No permission check and no bounds
        check - whatever arrives is sent to the vehicle.

        The except clause is the whole of the validation: a misspelled
        parameter, a missing one, or a duplicate all surface as a TypeError
        from the splat rather than as a bespoke desk-check.
        """
        method_name = COMMAND_TO_METHOD.get(msg.command)
        if method_name is None:
            # Throttled - a stuck producer would otherwise repeat this at the
            # full publish rate.
            self.get_logger().error(
                f'unknown command: {msg.command!r}', throttle_duration_sec=1.0)
            return

        if len(msg.param_names) != len(msg.param_values):
            self.get_logger().error(
                f'{msg.command}: {len(msg.param_names)} names vs '
                f'{len(msg.param_values)} values')
            return

        params = dict(zip(msg.param_names, msg.param_values))
        try:
            getattr(self.adapter, method_name)(**params)
        except TypeError as exc:
            self.get_logger().error(f'{msg.command} not dispatched: {exc}')

    # ------------------------------------------------------------------
    # Outbound
    # ------------------------------------------------------------------

    def _publish_telemetry(self):
        """
        One snapshot per timer tick. get_telemetry() reverts to its startup
        values when /mavros/state goes stale, so a dead link publishes
        state.mode == '' rather than the last values it saw - that empty mode
        is what a consumer checks to tell "no link" from real data.
        """
        t = self.adapter.get_telemetry()

        msg = VehicleTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()

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

    def _flight_logger(self):
        t = self.adapter.get_telemetry()
        lp = t['local_position']    # ENU, world frame
        vel = t['velocity_body']    # RFU, body frame
        bat = t['battery']
        state = t['state']
        self.get_logger().info(
            f"pos=(E={lp['east']:.2f},N={lp['north']:.2f},U={lp['up']:.2f}) "
            f"vel=(R={vel['right']:.2f},F={vel['forward']:.2f},U={vel['up']:.2f}) "
            f"bat={bat['percentage'] * 100.0:.0f}% "
            f"armed={state['armed']} mode={state['mode'] or 'NO_LINK'} "
            f"in_air={state['in_air']}"
        )


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _fill_global(field, src):
    field.lat = src['lat']
    field.lon = src['lon']
    field.alt = src['alt']


def _fill_body_velocity(field, src):
    field.right = src['right']
    field.forward = src['forward']
    field.up = src['up']
    field.yaw_rate = src['yaw_rate']


def main(args=None):
    rclpy.init(args=args)
    node = VehicleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # plain print, not get_logger() - rclpy's SIGINT handler has already
        # torn the context down by now, so a /rosout publish would fail.
        print('Shutting down vehicle node')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
