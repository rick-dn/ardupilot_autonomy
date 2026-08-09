#!/usr/bin/env python3
"""
FSM Handler - called synchronously from vehicle_controller's tick, no thread
of its own, no rclpy dependency. Arbitrates the current request_list down to
at most one command, across three axes in priority order:

  1. fc_state    - context only (mode/armed/in_air), never itself a decision.
  2. safety axis - rc_throttle_status / geofence / rc_loss / battery. The
                   highest-priority currently-active one short-circuits
                   everything below it (a forced safety command doesn't need
                   fs_adapter's permission). A WARNING-tier entry doesn't
                   short-circuit - it just gets folded into the reason text.
  3. fs_adapter  - the actual requested command, only reached if axis 2
                   didn't short-circuit. Gated by fc_state's permission table
                   AND by argument sanity-checking (moved here from
                   vehicle_controller - fsm now owns both state-permission and
                   argument validity, vehicle_controller just dispatches).

Holds no flight-state of its own - the only persisted state is the
reject-latch table (condition_name -> last-rejected token), so a rejected
fs_adapter submission stays rejected until a genuinely new token appears.
"""

import dataclasses
import math
from typing import Any, Dict

from ardupilot_autonomy import constants


@dataclasses.dataclass
class FcState:
    mode: constants.FcMode
    armed: constants.FcArm
    in_air: bool  # already resolved by the fc_state monitor


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

# Axis 3: state-based permission, keyed on (armed, in_air).
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

MOTION_COMMANDS = {'GOTO_GLOBAL', 'GOTO_LOCAL', 'GOTO_BODY', 'SET_VELOCITY', 'SET_ACCEL'}


class FsmHandler:

    def __init__(self, takeoff_alt_min_m, takeoff_alt_max_m,
                 goto_alt_min_m, goto_alt_max_m,
                 max_body_step_m, max_velocity_mps, max_accel_mps2):
        limits = {
            'takeoff_alt_min_m': takeoff_alt_min_m,
            'takeoff_alt_max_m': takeoff_alt_max_m,
            'goto_alt_min_m': goto_alt_min_m,
            'goto_alt_max_m': goto_alt_max_m,
            'max_body_step_m': max_body_step_m,
            'max_velocity_mps': max_velocity_mps,
            'max_accel_mps2': max_accel_mps2,
        }
        self._sanity_checks = _build_sanity_checks(limits)
        self._rejected: Dict[Any, Any] = {}  # condition_name -> last-rejected token

    def arbitrate(self, request_list):
        rc_throttle, geofence, rc_loss, battery, fc_state, fs_adapter = request_list
        fc: FcState = fc_state[2]

        forced, stacked_reasons = self._arbitrate_safety(rc_throttle, geofence, rc_loss, battery)
        if forced is not None:
            return forced

        return self._arbitrate_fs_adapter(fs_adapter, fc, stacked_reasons)

    def _arbitrate_safety(self, *entries):
        active = [e for e in entries if e[2] is not None]
        stacked_reasons = []

        if not active:
            return None, stacked_reasons

        winner = max(active, key=lambda e: (
            ACTION_SEVERITY.get(e[2].name, 0),
            CONDITION_PRIORITY.get(e[0], 0),
        ))
        cond, token, command = winner

        if command.name == 'WARNING':
            stacked_reasons.append(f'{cond}: warning')
            return None, stacked_reasons

        return (cond, command, token, f'{cond}: safety override ({command.name})'), stacked_reasons

    def _arbitrate_fs_adapter(self, fs_adapter, fc: FcState, stacked_reasons):
        cond, token, command = fs_adapter

        if command is None:
            return cond, None, token, _join(stacked_reasons, 'idle')

        if self._rejected.get(cond) == token:
            return cond, None, token, _join(stacked_reasons, 'latched: previously rejected')

        allowed = ALLOWED_COMMANDS.get((fc.armed, fc.in_air), set())
        if command.name not in allowed:
            self._rejected[cond] = token
            return cond, None, token, _join(
                stacked_reasons,
                f'{command.name} not allowed: armed={fc.armed.name}, in_air={fc.in_air}')

        if command.name in MOTION_COMMANDS and fc.mode != constants.FcMode.GUIDED:
            self._rejected[cond] = token
            return cond, None, token, _join(
                stacked_reasons, f'{command.name} requires GUIDED mode')

        if not self._sanity_ok(command):
            self._rejected[cond] = token
            return cond, None, token, _join(
                stacked_reasons, f'{command.name} failed sanity check: {command.params}')

        return cond, command, token, _join(stacked_reasons, 'accept')

    def _sanity_ok(self, command) -> bool:
        check = self._sanity_checks.get(command.name)
        if check is None:
            return True  # no bounds registered for this command - nothing to check
        try:
            if not all(math.isfinite(v) for v in command.params.values() if isinstance(v, (int, float))):
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


def _join(stacked_reasons, final):
    return '; '.join(stacked_reasons + [final])
