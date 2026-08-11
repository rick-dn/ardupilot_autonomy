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
