#!/usr/bin/env python3
"""
FSM Handler - called synchronously from vehicle_controller's tick, no thread
of its own, no rclpy dependency. Everything here is per-tick: nothing is
remembered between calls, so the same inputs always produce the same output.

Four axes, each an explicit pass/fail gate, evaluated in order and
short-circuiting on the first failure. That ordering is the command chain -
an axis is only reached if every axis above it passed:

  1. fc_state     - the autonomy stack commands only in GUIDED. Any other mode
                    stops the chain here, safety included.
  2. safety       - rc_throttle_status / geofence / rc_loss / battery. If any
                    is active the highest-severity one issues its own command
                    and the chain stops, so a safety override bypasses the
                    permission table and the sanity checks below.
  3. command state - the (armed, in_air) permission table. Reached only once
                    axes 1 and 2 have both passed.
  4. sanity       - fs_adapter's arguments against the configured limits.

Returns (cmd_status, cond, command, token):

  cmd_status - ACCEPTED / REJECTED / IDLE, describing what happened to
               fs_adapter's submission. Independent of `command`: nothing
               submitted while safety fires is IDLE with a command to dispatch.
  cond       - accumulates across all four axes; the names of every condition
               that had something to say this tick. Empty when clean.
  command    - the one command to dispatch, from safety (axis 2) or from
               fs_adapter (axis 4), or None. Which of the two it came from is
               readable from cmd_status: REJECTED with a command means safety
               outranked the submission.
  token      - fs_adapter's token, passed straight back so the caller can match
               the outcome to its submission. None when nothing was submitted.
"""

import math
from typing import Any, Dict, List, Tuple

from ardupilot_autonomy import constants
from ardupilot_autonomy.constants import FcState

Request = Tuple[Any, Any]                       # (token, command)
Decision = Tuple[str, List[str], Any, Any]      # (cmd_status, cond, command, token)

# cmd_status values - what became of fs_adapter's submission.
ACCEPTED = 'ACCEPTED'
REJECTED = 'REJECTED'
IDLE = 'IDLE'

# cond names contributed by the non-safety axes. The safety axis takes its
# names from the keys of the dict it's handed.
FC_STATE = 'FC_STATE'
COMMAND_STATE = 'COMMAND_STATE'
SANITY = 'SANITY'

# Axis 2: action severity (what wins when multiple safety sources are active
# at once) and condition priority (tiebreak only, for equal-severity actions).
ACTION_SEVERITY = {
    'WARNING': 1,
    'WAYPOINT_CMD': 2,
    'RTL': 3,
    'LAND': 4,
}

CONDITION_PRIORITY = {
    'RC_THROTTLE_STATUS': 1,
    'GEOFENCE': 2,
    'RC_LOSS': 3,
    'BATTERY': 4,
}

# Axis 3: state-based permission, keyed on (armed, in_air). Only consulted once
# the mode is already GUIDED, so it says nothing about mode itself.
ALLOWED_COMMANDS = {
    (constants.FcArm.DISARMED, False): {'ARM'},
    (constants.FcArm.ARMED, False): {'TAKEOFF', 'DISARM'},
    (constants.FcArm.ARMED, True): {
        'GOTO_GLOBAL', 'GOTO_LOCAL', 'GOTO_BODY',
        'SET_VELOCITY', 'STOP_VELOCITY',
        'SET_ACCEL', 'STOP_ACCEL',
        'LAND', 'RTL',
    },
}


class FsmHandler:

    def __init__(self, limits: Dict[str, float]):
        self._sanity_checks = _build_sanity_checks(limits)

    def arbitrate(self, fc_state: FcState, safety_requests: Dict[str, Request],
                  fs_request: Request) -> Decision:
        token, command = fs_request
        submitted = command is not None
        if not submitted:
            token = None  # nothing to match an outcome against

        cond: List[str] = []

        # Axis 1 - fc_state. Stops the chain outright: outside GUIDED the
        # autonomy stack injects nothing at all, safety commands included.
        if not self._fc_state_ok(fc_state, cond):
            return (REJECTED if submitted else IDLE), cond, None, token

        # Axis 2 - safety. Runs whether or not anything was submitted, since a
        # safety condition dispatches on its own.
        safety_command = self._safety_override(safety_requests, cond)
        if safety_command is not None:
            return (REJECTED if submitted else IDLE), cond, safety_command, token

        if not submitted:
            return IDLE, cond, None, token

        # Axis 3 - command state machine.
        if not self._command_state_ok(fc_state, command, cond):
            return REJECTED, cond, None, token

        # Axis 4 - argument sanity.
        if not self._sanity_ok(command, cond):
            return REJECTED, cond, None, token

        return ACCEPTED, cond, command, token

    # ------------------------------------------------------------------
    # Axes. Each appends its own name to cond on failure and returns a bool,
    # so the chain above reads as four gates and nothing else.
    # ------------------------------------------------------------------

    def _fc_state_ok(self, fc: FcState, cond: List[str]) -> bool:
        if fc.mode == constants.FcMode.GUIDED:
            return True
        cond.append(FC_STATE)
        return False

    def _safety_override(self, safety_requests: Dict[str, Request], cond: List[str]):
        """
        Axis 2. Names every active condition in cond - not just the winner,
        since cond is what the operator sees - and returns the winning command,
        or None if the axis passes. A WARNING-tier entry names itself without
        winning: it's information, not an override.

        Sorted by CONDITION_PRIORITY so cond's order is stable regardless of
        the order the caller happened to build the dict in.
        """
        active = sorted(
            ((name, command) for name, (_, command) in safety_requests.items()
             if command is not None),
            key=lambda e: CONDITION_PRIORITY.get(e[0], 0),
        )
        if not active:
            return None

        cond.extend(name for name, _ in active)

        forced = [e for e in active if e[1].name != 'WARNING']
        if not forced:
            return None

        _, command = max(forced, key=lambda e: (
            ACTION_SEVERITY.get(e[1].name, 0),
            CONDITION_PRIORITY.get(e[0], 0),
        ))
        return command

    def _command_state_ok(self, fc: FcState, command, cond: List[str]) -> bool:
        if command.name in ALLOWED_COMMANDS.get((fc.armed, fc.in_air), set()):
            return True
        cond.append(COMMAND_STATE)
        return False

    def _sanity_ok(self, command, cond: List[str]) -> bool:
        if self._params_ok(command):
            return True
        cond.append(SANITY)
        return False

    def _params_ok(self, command) -> bool:
        check = self._sanity_checks.get(command.name)
        if check is None:
            return True  # no bounds registered for this command - nothing to check
        try:
            if not all(math.isfinite(v) for v in command.params.values()
                       if isinstance(v, (int, float))):
                return False
            return bool(check(command.params))
        except (KeyError, TypeError):
            return False


def _build_sanity_checks(limits):
    return {
        'TAKEOFF': lambda p: limits['takeoff_alt_min_m'] <= p['altitude'] <= limits['takeoff_alt_max_m'],
        'GOTO_GLOBAL': lambda p: (
            -90.0 <= p['lat'] <= 90.0 and -180.0 <= p['lon'] <= 180.0
            and limits['goto_alt_min_m'] <= p['alt'] <= limits['goto_alt_max_m']),
        'GOTO_LOCAL': lambda p: limits['goto_alt_min_m'] <= p['up'] <= limits['goto_alt_max_m'],
        'GOTO_BODY': lambda p: (
            math.sqrt(p['right'] ** 2 + p['forward'] ** 2 + p['up'] ** 2) <= limits['max_body_step_m']),
        'SET_VELOCITY': lambda p: (
            math.sqrt(p['east'] ** 2 + p['north'] ** 2 + p['up'] ** 2) <= limits['max_velocity_mps']),
        'SET_ACCEL': lambda p: (
            math.sqrt(p['east'] ** 2 + p['north'] ** 2 + p['up'] ** 2) <= limits['max_accel_mps2']),
    }
