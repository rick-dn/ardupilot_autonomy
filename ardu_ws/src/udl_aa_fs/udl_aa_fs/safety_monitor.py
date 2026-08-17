#!/usr/bin/env python3
"""
Safety Monitor - runs its own thread, independent of vehicle_controller's tick
rate. Evaluates geofence / battery / rc_throttle_status / rc_loss / fc_state
conditions and exposes the latest result to vehicle_controller via a single
non-blocking atomic read.

One read, not five: all five conditions are evaluated by the same thread in one
pass, so handing them out individually would let a reader pick up some values
from one evaluation cycle and the rest from the next. snapshot() returns them
as one object so the tick always arbitrates over an internally consistent set.

geofence / battery / rc_throttle_status / rc_loss stay pure stubs for now -
always inactive (command=None) - real detection logic is future work. What's
real is fc_state, and all three of its fields now come straight from the
telemetry snapshot: mode, armed, and in_air, the last of which mavros_interface
derives and latches. This class holds no flight-state of its own.
"""

import threading
import time
from typing import Any, Dict, Tuple

from udl_aa_fs import constants
from udl_aa_fs.constants import FcState

Request = Tuple[Any, Any]                       # (token, command)
Snapshot = Tuple[FcState, Dict[str, Request]]   # (fc_state, {condition_name: request})


class SafetyMonitor:

    def __init__(self, mavros, tick_rate_hz: float = 10.0):
        self._mavros = mavros
        self._period_s = 1.0 / tick_rate_hz

        # A single reference, swapped whole by _run() (the writer thread) and
        # read whole by snapshot() (called from vehicle_controller's tick, a
        # different thread). A plain rebind of the name is atomic under the GIL
        # - no lock needed for this single-writer/multi-reader "latest value"
        # pattern. Keeping it to one object is also what makes the read
        # consistent: there's no window in which half of it has been updated.
        self._snapshot: Snapshot = (
            FcState(mode=constants.FcMode.UNKNOWN,
                    armed=constants.FcArm.UNKNOWN,
                    in_air=False),
            {
                'RC_THROTTLE_STATUS': (0, None),
                'GEOFENCE': (0, None),
                'RC_LOSS': (0, None),
                'BATTERY': (0, None),
            },
        )

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join()

    def _run(self):
        while not self._stop_event.is_set():
            self._snapshot = (
                self._eval_fc_state(),
                {
                    'RC_THROTTLE_STATUS': self._eval_rc_throttle_status(),
                    'GEOFENCE': self._eval_geofence(),
                    'RC_LOSS': self._eval_rc_loss(),
                    'BATTERY': self._eval_battery(),
                },
            )
            time.sleep(self._period_s)

    # ------------------------------------------------------------------
    # Evaluations
    # ------------------------------------------------------------------

    def _eval_geofence(self) -> Request:
        return (0, None)  # stub - never triggers

    def _eval_battery(self) -> Request:
        return (0, None)  # stub - never triggers

    def _eval_rc_throttle_status(self) -> Request:
        return (0, None)  # stub - never triggers

    def _eval_rc_loss(self) -> Request:
        return (0, None)  # stub - never triggers

    def _eval_fc_state(self) -> FcState:
        telemetry = self._mavros.get_telemetry()
        mode_str = telemetry['state']['mode']
        armed_bool = telemetry['state']['armed']

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

        return FcState(mode=mode, armed=armed, in_air=telemetry['state']['in_air'])

    # ------------------------------------------------------------------
    # Non-blocking atomic read - vehicle_controller's expected interface.
    # ------------------------------------------------------------------

    def snapshot(self) -> Snapshot:
        return self._snapshot

    def on_result(self, condition_name, token, command, cmd_status, reason):
        pass  # stub
