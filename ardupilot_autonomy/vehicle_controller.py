#!/usr/bin/env python3
"""
Vehicle Controller - owns the always-on core tick and is the sole caller into
mavros_interface.

Each tick is a fixed, straight-line sequence - no loops: read one atomic value
from each known condition source, hand all of them to fsm_handler in a single
call to arbitrate, dispatch the one command it picks (if any), and report the
outcome back to whichever source raised the winning condition. fsm_handler
owns both state-based permission and argument sanity-checking now - vc's
dispatch is a single generic call per command, nothing bespoke here.
"""

import dataclasses
from typing import Any, Dict

import rclpy
from rclpy.node import Node

from ardupilot_autonomy.mavros_interface import MavrosInterface
from ardupilot_autonomy.safety_monitor import SafetyMonitor
from ardupilot_autonomy.fsm_handler import FsmHandler
from ardupilot_autonomy.fs_interface import FsInterface


@dataclasses.dataclass
class Command:
    name: str
    params: Dict[str, Any] = dataclasses.field(default_factory=dict)


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
      safety_monitor.rc_throttle_status_atomic() -> (condition_name, token, command)
      safety_monitor.geofence_atomic()           -> (condition_name, token, command)
      safety_monitor.rc_loss_atomic()            -> (condition_name, token, command)
      safety_monitor.battery_atomic()            -> (condition_name, token, command)
      safety_monitor.fc_state_atomic()           -> (condition_name, token, command)
      fs_interface.fs_adapter_atomic()           -> (condition_name, token, command)
      fsm_handler.arbitrate(request_list) -> (condition_name, command, token, reason)
      fs_interface.report(cmd_status, condition_name, command, token, reason) -> None

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
        self.fsm_handler = FsmHandler(
            takeoff_alt_min_m=self.get_parameter('takeoff_alt_min_m').value,
            takeoff_alt_max_m=self.get_parameter('takeoff_alt_max_m').value,
            goto_alt_min_m=self.get_parameter('goto_alt_min_m').value,
            goto_alt_max_m=self.get_parameter('goto_alt_max_m').value,
            max_body_step_m=self.get_parameter('max_body_step_m').value,
            max_velocity_mps=self.get_parameter('max_velocity_mps').value,
            max_accel_mps2=self.get_parameter('max_accel_mps2').value,
        )
        self.fs_interface = FsInterface(self.mavros_interface)

        self.declare_parameter('tick_rate_hz', 20.0)
        tick_rate_hz = self.get_parameter('tick_rate_hz').value
        self._tick_timer = self.create_timer(1.0 / tick_rate_hz, self.tick)

        self.declare_parameter('logger_rate_hz', 1.0)
        logger_rate_hz = self.get_parameter('logger_rate_hz').value
        self._logger_timer = self.create_timer(1.0 / logger_rate_hz, self.flight_logger)

        self.get_logger().info(f'Vehicle Controller initialized, tick={tick_rate_hz}Hz')

    # ------------------------------------------------------------------
    # Core tick
    # ------------------------------------------------------------------

    def tick(self):
        """
        Core tick - always on, fixed rate, never blocks, no loops.
        Six explicit atomic reads, one arbitration call, one dispatch,
        one report. That's the whole tick.
        """
        self.fs_interface.publish_telemetry()

        request_list = []
        request_list.append(self.safety_monitor.rc_throttle_status_atomic())
        request_list.append(self.safety_monitor.geofence_atomic())
        request_list.append(self.safety_monitor.rc_loss_atomic())
        request_list.append(self.safety_monitor.battery_atomic())
        request_list.append(self.safety_monitor.fc_state_atomic())
        request_list.append(self.fs_interface.fs_adapter_atomic())

        cond, command, token, reason = self.fsm_handler.arbitrate(request_list)

        cmd_status = False
        if command:
            cmd_status = self._dispatch(command)

        # Always fires, every tick, regardless of whether anything was dispatched -
        # acts as a heartbeat so producers stay aware of current status even on
        # ticks where nothing happened.
        self.fs_interface.report(cmd_status, cond, command, token, reason)

    def _dispatch(self, command: Command) -> bool:
        """
        fsm_handler has already validated both state-permission and argument
        sanity by the time a command reaches here - this is a single generic
        call, not a bespoke check-then-call per command.
        """
        method_name = COMMAND_TO_METHOD.get(command.name)
        if method_name is None:
            self.get_logger().error(f'Unknown command: {command.name}')
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
        _, _, fc = self.safety_monitor.fc_state_atomic()
        self.get_logger().info(
            f"pos=(E={lp['east']:.2f},N={lp['north']:.2f},U={lp['up']:.2f}) "
            f"vel=(R={vel['right']:.2f},F={vel['forward']:.2f},U={vel['up']:.2f}) "
            f"bat=({bat['voltage']:.1f}V,{bat['current']:.1f}A,{bat['percentage'] * 100.0:.0f}%) "
            f"armed={fc.armed.name} mode={fc.mode.name}"
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
