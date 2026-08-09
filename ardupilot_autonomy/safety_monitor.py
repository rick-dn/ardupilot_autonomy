#!/usr/bin/env python3
"""
Safety Monitor - runs its own thread, independent of vehicle_controller's tick
rate. Evaluates geofence / battery / rc_throttle_status / rc_loss / fc_state
conditions and exposes the latest result to vehicle_controller via
non-blocking atomic reads.

geofence / battery / rc_throttle_status / rc_loss stay pure stubs for now -
always inactive (command=None) - real detection logic is future work. What's
real is fc_state: mode/armed come straight from telemetry each cycle, and
in_air is derived and latched here (not in fsm_handler - fsm holds no
flight-state of its own by design), since there's no telemetry field for it.
"""

import threading
import time
from typing import Any, Tuple

from ardupilot_autonomy import constants
from ardupilot_autonomy.fsm_handler import FcState

Atomic = Tuple[Any, Any, Any]  # (condition_name, token, command)

# in_air hysteresis thresholds (meters, local/relative altitude). Climbing
# above UP latches in_air True; dropping below DOWN latches it False; between
# the two, hold the previous value - avoids flicker right at the boundary.
IN_AIR_UP_THRESHOLD_M = 0.5
IN_AIR_DOWN_THRESHOLD_M = 0.2


class SafetyMonitor:

    def __init__(self, mavros, tick_rate_hz: float = 10.0):
        self._mavros = mavros
        self._period_s = 1.0 / tick_rate_hz

        # Each is a single reference, swapped whole by _run() (the writer
        # thread) and read whole by the *_atomic() methods (called from
        # vehicle_controller's tick, a different thread). A plain rebind of
        # the name is atomic under the GIL - no lock needed for this
        # single-writer/multi-reader "latest value" pattern.
        self._geofence_status: Atomic = ('GEOFENCE', 0, None)
        self._battery_status: Atomic = ('BATTERY', 0, None)
        self._rc_throttle_status: Atomic = ('RC_THROTTLE_STATUS', 0, None)
        self._rc_loss_status: Atomic = ('RC_LOSS', 0, None)
        self._fc_state_status: Atomic = ('FC_STATE', 0, FcState(
            mode=constants.FcMode.UNKNOWN, armed=constants.FcArm.UNKNOWN, in_air=False))

        self._in_air = False
        self._fc_state_token = 0

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def _run(self):
        while not self._stop_event.is_set():
            self._geofence_status = self._eval_geofence()
            self._battery_status = self._eval_battery()
            self._rc_throttle_status = self._eval_rc_throttle_status()
            self._rc_loss_status = self._eval_rc_loss()
            self._fc_state_status = self._eval_fc_state()
            time.sleep(self._period_s)

    # ------------------------------------------------------------------
    # Evaluations
    # ------------------------------------------------------------------

    def _eval_geofence(self) -> Atomic:
        return ('GEOFENCE', 0, None)  # stub - never triggers

    def _eval_battery(self) -> Atomic:
        return ('BATTERY', 0, None)  # stub - never triggers

    def _eval_rc_throttle_status(self) -> Atomic:
        return ('RC_THROTTLE_STATUS', 0, None)  # stub - never triggers

    def _eval_rc_loss(self) -> Atomic:
        return ('RC_LOSS', 0, None)  # stub - never triggers

    def _eval_fc_state(self) -> Atomic:
        telemetry = self._mavros.get_telemetry()
        mode_str = telemetry['state']['mode']
        armed_bool = telemetry['state']['armed']
        altitude = telemetry['local_position']['up']

        if mode_str == '':  # mavros_interface's default before any /mavros/state message
            mode = constants.FcMode.UNKNOWN
            armed = constants.FcArm.UNKNOWN
        else:
            mode = {
                'GUIDED': constants.FcMode.GUIDED,
                'LAND': constants.FcMode.LAND,
                'RTL': constants.FcMode.RTL,
            }.get(mode_str, constants.FcMode.OTHER)
            armed = constants.FcArm.ARMED if armed_bool else constants.FcArm.DISARMED

        if armed == constants.FcArm.ARMED and altitude > IN_AIR_UP_THRESHOLD_M:
            self._in_air = True
        elif altitude < IN_AIR_DOWN_THRESHOLD_M:
            self._in_air = False
        # else: hold previous value

        self._fc_state_token += 1
        return ('FC_STATE', self._fc_state_token, FcState(mode=mode, armed=armed, in_air=self._in_air))

    # ------------------------------------------------------------------
    # Non-blocking atomic reads - vehicle_controller's expected interface.
    # ------------------------------------------------------------------

    def geofence_atomic(self) -> Atomic:
        return self._geofence_status

    def battery_atomic(self) -> Atomic:
        return self._battery_status

    def rc_throttle_status_atomic(self) -> Atomic:
        return self._rc_throttle_status

    def rc_loss_atomic(self) -> Atomic:
        return self._rc_loss_status

    def fc_state_atomic(self) -> Atomic:
        return self._fc_state_status

    def on_result(self, condition_name, token, command, cmd_status, reason):
        pass  # stub
