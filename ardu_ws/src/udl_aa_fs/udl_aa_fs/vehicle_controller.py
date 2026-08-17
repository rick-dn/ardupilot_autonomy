#!/usr/bin/env python3
"""
Vehicle Controller - owns the always-on core tick and is the sole caller into
mavros_interface.

Each tick is a fixed, straight-line sequence - no loops: take one consistent
snapshot from safety_monitor and one request from fs_interface, hand both to
fsm_handler in a single call to arbitrate, dispatch the one command it picks
(if any), and report the outcome back. fsm_handler owns both state-based
permission and argument sanity-checking now - vc's dispatch is a single generic
call per command, nothing bespoke here.
"""

import rclpy
from rclpy.node import Node

from udl_aa_fs.constants import Command
from udl_aa_fs.mavros_interface import MavrosInterface
from udl_aa_fs.safety_monitor import SafetyMonitor
from udl_aa_fs.fsm_handler import FsmHandler
from udl_aa_fs.fs_interface import FsInterface


# Command name -> mavros_interface method name. fsm_handler has already
# validated both state-permission and argument sanity by the time a command
# reaches here, so this is a single generic call, not a bespoke method per
# command.
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


class VehicleController(Node):
    """
    Core tick + mavros_interface ownership.
    safety_monitor / fsm_handler / fs_interface expected interface:
      safety_monitor.snapshot()        -> (fc_state, {condition_name: (token, command)})
      fs_interface.fs_adapter_atomic() -> (token, command)
      fsm_handler.arbitrate(fc_state, safety_requests, fs_request)
                                       -> (cmd_status, cond, command, token)
      fs_interface.report(cmd_status, cond, command, token) -> None

    cmd_status is ACCEPTED/REJECTED/IDLE and describes fs_adapter's submission,
    not the dispatch - IDLE still carries a command when safety raised one.

    safety_monitor evaluates all five of its conditions on one thread in a
    single pass, so they're read back as one object rather than one call each -
    five separate reads could otherwise straddle a write and mix values from
    two different evaluation cycles.

    fs_interface.report() fires every tick without exception - it's a
    heartbeat, not a conditional notification - so it always knows current
    status, including on ticks where nothing was dispatched. fs_adapter in
    particular relies on this to stop a rejected sequence on the external
    side (the original command was sent async, so this is the only signal it
    gets). The call into it must be non-blocking, same as everything else here.
    """

    def __init__(self):
        super().__init__('vehicle_controller')

        self.mavros_interface = MavrosInterface(self)

        # Sanity bounds - real ROS parameters (see config/vehicle_controller.yaml),
        # injected into fsm_handler since fsm_handler has no rclpy Node of its own.
        self.declare_parameter('takeoff_alt_min_m', 0.5)
        self.declare_parameter('takeoff_alt_max_m', 30.0)
        self.declare_parameter('goto_alt_min_m', 0.0)
        self.declare_parameter('goto_alt_max_m', 50.0)
        self.declare_parameter('max_body_step_m', 20.0)
        self.declare_parameter('max_velocity_mps', 5.0)
        self.declare_parameter('max_accel_mps2', 3.0)

        # safety_monitor runs its own thread (started below); fsm_handler and
        # fs_interface are called synchronously from the tick, no thread of
        # their own.
        self.safety_monitor = SafetyMonitor(self.mavros_interface)
        self.safety_monitor.start()
        self.fsm_handler = FsmHandler({
            name: self.get_parameter(name).value
            for name in ('takeoff_alt_min_m', 'takeoff_alt_max_m',
                         'goto_alt_min_m', 'goto_alt_max_m',
                         'max_body_step_m', 'max_velocity_mps', 'max_accel_mps2')
        })
        self.fs_interface = FsInterface(self.mavros_interface)

        self.declare_parameter('tick_rate_hz', 20.0)
        tick_rate_hz = self.get_parameter('tick_rate_hz').value
        self._tick_timer = self.create_timer(1.0 / tick_rate_hz, self.tick)

        self.declare_parameter('logger_rate_hz', 1.0)
        logger_rate_hz = self.get_parameter('logger_rate_hz').value
        self._logger_timer = self.create_timer(1.0 / logger_rate_hz, self.flight_logger)

        # Telemetry gets its own rate rather than riding the tick: the fastest
        # MAVROS group feeding it measures ~3Hz, so publishing at tick rate
        # would emit several byte-identical messages per new datum.
        self.declare_parameter('telemetry_rate_hz', 5.0)
        telemetry_rate_hz = self.get_parameter('telemetry_rate_hz').value
        self._telemetry_timer = self.create_timer(
            1.0 / telemetry_rate_hz, self.fs_interface.publish_telemetry)

        self.get_logger().info(f'Vehicle Controller initialized, tick={tick_rate_hz}Hz')

    # ------------------------------------------------------------------
    # Core tick
    # ------------------------------------------------------------------

    def tick(self):
        """
        Core tick - always on, fixed rate, never blocks, no loops.
        Two atomic reads, one arbitration call, one dispatch, one report.
        That's the whole tick.
        """
        fc_state, safety_requests = self.safety_monitor.snapshot()
        fs_request = self.fs_interface.fs_adapter_atomic()

        cmd_status, cond, command, token = self.fsm_handler.arbitrate(
            fc_state, safety_requests, fs_request)

        if command:
            self._dispatch(command)

        # Always fires, every tick, regardless of whether anything was dispatched -
        # acts as a heartbeat so producers stay aware of current status even on
        # ticks where nothing happened.
        self.fs_interface.report(cmd_status, cond, command, token)

    def _dispatch(self, command: Command) -> bool:
        """
        fsm_handler has already validated both state-permission and argument
        sanity by the time a command reaches here - this is a single generic
        call, not a bespoke check-then-call per command.
        """
        method_name = COMMAND_TO_METHOD.get(command.name)
        if method_name is None:
            # Throttled - a stuck producer would otherwise repeat this at the
            # full tick rate.
            self.get_logger().error(
                f'Unknown command: {command.name}', throttle_duration_sec=1.0)
            return False
        getattr(self.mavros_interface, method_name)(**command.params)
        return True

    # ------------------------------------------------------------------
    # Flight logger - periodic, independent of the core tick's rate.
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.safety_monitor.stop()
        super().destroy_node()

    def flight_logger(self):
        t = self.mavros_interface.get_telemetry()
        lp = t['local_position']    # ENU, world frame
        vel = t['velocity_body']    # RFU, body frame
        bat = t['battery']
        fc, _ = self.safety_monitor.snapshot()
        self.get_logger().info(
            f"pos=(E={lp['east']:.2f},N={lp['north']:.2f},U={lp['up']:.2f}) "
            f"vel=(R={vel['right']:.2f},F={vel['forward']:.2f},U={vel['up']:.2f}) "
            f"bat={bat['percentage'] * 100.0:.0f}% "
            f"armed={fc.armed.name} mode={fc.mode.name} in_air={fc.in_air}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # plain print, not get_logger() - rclpy's SIGINT handler has already
        # torn the context down by now, so a /rosout publish would fail.
        print('Shutting down vehicle controller')
    finally:
        # unconditional - stops the safety_monitor thread. Safe on a dead
        # context; only shutdown() needs the rclpy.ok() guard.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
