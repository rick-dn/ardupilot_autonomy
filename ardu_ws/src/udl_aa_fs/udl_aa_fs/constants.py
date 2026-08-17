#!/usr/bin/env python3
"""
Structural constants and the shared types built from them - fixed
protocol-level definitions, not meant to be tuned per deployment. Operational
values that ARE meant to be tuned (sanity bounds, rates, etc.) live in ROS
parameters instead - see config/vehicle_controller.yaml.

IntEnum rather than bare ints so the name is derivable from the same single
definition (.name -> 'UNKNOWN') without a second lookup table. Members still
behave as ints everywhere: comparisons, dict keys, arithmetic.
"""

import dataclasses
from enum import IntEnum
from typing import Dict


class FcMode(IntEnum):
    """Flight-controller mode buckets, as reported by fc_state monitor."""
    UNKNOWN = -1
    OTHER = 0     # anything except GUIDED/LAND/RTL
    GUIDED = 1
    LAND = 2
    RTL = 3


class FcArm(IntEnum):
    """Armed status buckets, as reported by fc_state monitor."""
    UNKNOWN = -1
    DISARMED = 0
    ARMED = 1


@dataclasses.dataclass
class Command:
    """
    One command to dispatch. name selects the mavros_interface method via
    vehicle_controller.COMMAND_TO_METHOD; params is splatted into it as keyword
    arguments, and must be exactly the set COMMAND_PARAMS declares below.

    Lives here rather than in vehicle_controller because fs_interface builds
    these from incoming VehicleCommand messages, and vehicle_controller already
    imports fs_interface - importing it back would be a cycle.
    """
    name: str
    params: Dict[str, float] = dataclasses.field(default_factory=dict)


# The exact parameter set each command expects, mirroring the mavros_interface
# method signatures that vehicle_controller._dispatch splats these into.
#
# Authoritative for desk-rejection: fs_adapter compares an incoming message's
# param_names against this before the command reaches the fsm, so an unknown
# command name, a misspelled parameter, a duplicate or a missing one is caught
# at the boundary as MALFORMED. Without it, a typo would fall through to the
# fsm's sanity axis and be reported as SANITY - which would wrongly imply the
# values were merely out of bounds.
#
# Every parameter in the stack is a float, which is what lets VehicleCommand
# carry them as parallel name/value arrays instead of a message type per
# command. Adding a command means adding a row here, a row in
# vehicle_controller.COMMAND_TO_METHOD, and an entry in the relevant
# ALLOWED_COMMANDS state.
COMMAND_PARAMS = {
    'ARM': frozenset(),
    'DISARM': frozenset(),
    'SET_MODE_GUIDED': frozenset(),
    'TAKEOFF': frozenset({'altitude'}),
    'GOTO_GLOBAL': frozenset({'lon', 'lat', 'alt', 'yaw_deg'}),
    'GOTO_LOCAL': frozenset({'east', 'north', 'up', 'yaw_deg'}),
    'GOTO_BODY': frozenset({'right', 'forward', 'up', 'yaw_deg'}),
    'SET_VELOCITY': frozenset({'east', 'north', 'up', 'yaw_rate'}),
    'STOP_VELOCITY': frozenset(),
    'SET_ACCEL': frozenset({'east', 'north', 'up'}),
    'STOP_ACCEL': frozenset(),
    'LAND': frozenset(),
    'RTL': frozenset(),
}


@dataclasses.dataclass
class FcState:
    """
    What the fc_state monitor reports. Lives here rather than in fsm_handler so
    the producer (safety_monitor) doesn't have to import its own output type
    back from a consumer. in_air is already resolved by the monitor - there's
    no telemetry field for it.
    """
    mode: FcMode
    armed: FcArm
    in_air: bool
