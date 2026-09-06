#!/usr/bin/env python3
"""
Structural constants for the safety stack - the fixed vocabulary, not the tuned
values. Anything meant to be retuned per deployment is a ROS parameter instead;
see config/safety_stack.yaml.

Plain classes of string constants rather than enums: the value is the name, so
nothing needs a second lookup to render one, and it goes onto a ROS string field
as-is.

Every mapping here is a table. A condition grades itself against its own tiers
and picks a Message; safety_monitor only ranks the messages it is handed.
Nothing in this file is control flow.

Verdict and Message live here rather than in safety_monitor because the
conditions produce them and safety_monitor consumes them - putting them with the
consumer would make every condition import its own reader, and the reader import
them back.
"""

import dataclasses
from typing import Optional


class Condition:
    """Condition names. These are what the operator sees in the cond list."""
    BATTERY = 'BATTERY'
    GEOFENCE = 'GEOFENCE'
    RC_LOSS = 'RC_LOSS'
    RC_THROTTLE = 'RC_THROTTLE'


class Sequence:
    """
    Sequence names, as the commander registers them - the same role 'takeoff'
    and 'smart_rtl' play in udl_aa_fc's SEQUENCES table.

    A condition never names a sequence directly; it picks a Message, and the
    Message carries the sequence. That way the operator-facing wording and the
    action it implies are defined together and cannot drift apart.
    """
    SMART_RTL = 'smart_rtl'
    RTL = 'rtl'
    LAND = 'land'
    EMERGENCY_LAND = 'emergency_land'
    MOTOR_CUTOFF = 'motor_cutoff'


class Level:
    """
    Bands a condition grades itself into. The name maps to a config key through
    that condition's key function - Level.VERY_LOW -> 'batt_very_low_v' - so
    adding a band is a row in a tier table and a line in the yaml, never a code
    change.

    Names are reused across families where they mean the same shape of thing:
    LOW is a battery band and a throttle band, and the two never collide because
    each family has its own key function.
    """
    # battery
    INVALID = 'INVALID'
    CRITICAL = 'CRITICAL'
    VERY_LOW = 'VERY_LOW'
    LOW = 'LOW'
    # geofence
    BREACHED = 'BREACHED'
    NEAR = 'NEAR'
    # rc link
    LOST = 'LOST'
    # rc throttle - LOW is shared with the battery bands above
    MEDIUM = 'MEDIUM'
    HIGH = 'HIGH'


@dataclasses.dataclass(frozen=True)
class SafetyMessage:
    """
    One thing the safety stack can have to say, and what it wants done about it.

    The pairing is the point: `text` reaches the operator and `sequence` reaches
    the commander, and because they are defined on the same row they cannot
    disagree about what is happening. A sequence of None is a warning - it names
    itself in cond and runs nothing.
    """
    text: str
    sequence: Optional[str] = None


class Message:
    """
    The catalogue. Every distinct thing a condition can report is a row here, so
    the full vocabulary of the safety stack is readable in one place rather than
    scattered across the conditions that raise it.
    """
    # Nothing to report. The inactive verdict, so no condition needs a None
    # branch and `message.sequence` is always safe to read.
    NONE = SafetyMessage('')

    # battery
    BATT_LOW = SafetyMessage('battery low')
    BATT_VERY_LOW = SafetyMessage('battery very low', Sequence.SMART_RTL)
    BATT_CRITICAL = SafetyMessage('battery critical', Sequence.LAND)

    # geofence
    FENCE_NEAR = SafetyMessage('approaching fence')
    FENCE_BREACHED = SafetyMessage('fence breached', Sequence.RTL)

    # rc
    RC_LOST = SafetyMessage('rc link lost', Sequence.RTL)
    RC_THROTTLE_LOW = SafetyMessage('rc throttle low')


@dataclasses.dataclass(frozen=True)
class Verdict:
    """
    One condition's latest answer.

    There is no `active` flag and no separate `sequence` field - both would be a
    second copy of what `message` already says, and two copies can disagree.
    Inactive is Message.NONE, which is also the default, so a condition that has
    never run reads as clear rather than as anything else.

    `token` identifies one firing, so the commander can acknowledge a condition
    once rather than being asked again for the same crossing. Stubbed at 0 until
    that is implemented.

    `condition` is stamped by the thread that ran the evaluator, not by the
    evaluator itself - a condition should not have to know its own name to
    report a fact. It is what the priority tiebreak sorts on, and what tells a
    reader downstream which condition raised the message.
    """
    message: SafetyMessage = Message.NONE
    token: int = 0
    condition: str = ''


CLEAR = Verdict()


# Which sequence wins when several conditions fire at once. Ranked over the
# sequence rather than the condition, so a battery asking to land outranks a
# fence asking to return regardless of which raised it.
#
# None is the warning tier and ranks lowest, which is what makes WARNING need no
# special case anywhere: a real sequence always outranks it, and if only
# warnings are active the winner's sequence is None and nothing runs.
SEVERITY = {
    None: 1,
    Sequence.SMART_RTL: 2,
    Sequence.RTL: 3,
    Sequence.LAND: 4,
    Sequence.EMERGENCY_LAND: 5,
    Sequence.MOTOR_CUTOFF: 6,
}

# Tiebreak between equal-severity sequences, and the order of the cond list -
# so what the operator sees does not depend on dict insertion order.
PRIORITY = {
    Condition.RC_THROTTLE: 1,
    Condition.GEOFENCE: 2,
    Condition.RC_LOSS: 3,
    Condition.BATTERY: 4,
}


# ----------------------------------------------------------------------
# Per-condition tier tables. Owned by the condition, not by the monitor.
# Every table is walked in order and the first crossed threshold wins.
# ----------------------------------------------------------------------

# Crossed going *down*, so ascending: most severe first.
#
# INVALID is not a battery band. Before the first telemetry message the whole
# snapshot reads zero, and 0.0 V means nobody is talking to this vehicle rather
# than a flat pack. Giving it a row resolves it to Message.NONE through the same
# walk as everything else, so the condition needs no guard clause.
BATTERY_TIERS = (
    (Level.INVALID, Message.NONE),
    (Level.CRITICAL, Message.BATT_CRITICAL),
    (Level.VERY_LOW, Message.BATT_VERY_LOW),
    (Level.LOW, Message.BATT_LOW),
)

# Crossed going *up*, so descending - same walk, opposite direction. The same
# two bands grade both axes and only the config key differs, which is what lets
# one table serve both and the worse of the two win.
FENCE_TIERS = (
    (Level.BREACHED, Message.FENCE_BREACHED),
    (Level.NEAR, Message.FENCE_NEAR),
)

FENCE_AXES = ('radius', 'alt')

# One band. A single-row table rather than a bare constant so it reads like the
# graded conditions and gains a band without restructuring.
RC_LOSS_TIERS = ((Level.LOST, Message.RC_LOST),)

# Crossed going down, ascending. HIGH is the fallback band and so has no
# threshold and no row - anything above MEDIUM is normal flight.
#
# Only LOW reports, and it reports without a sequence on purpose: a throttle
# that far down is ArduPilot's own failsafe event and it will change mode
# itself. Acting here would put a second vehicle-mover in a race with the
# autopilot over the same fact.
RC_THROTTLE_TIERS = (
    (Level.LOW, Message.RC_THROTTLE_LOW),
    (Level.MEDIUM, Message.NONE),
)

# Throttle is RC channel 3, zero-indexed into the channels array.
THROTTLE_CHANNEL = 2


def batt_key(level: str) -> str:
    """Battery band to config: VERY_LOW -> batt_very_low_v."""
    return f'batt_{level.lower()}_v'


def fence_key(axis: str, level: str) -> str:
    """Fence axis and band to config: ('radius', BREACHED) -> fence_radius_breached_m."""
    return f'fence_{axis}_{level.lower()}_m'


def throttle_key(level: str) -> str:
    """Throttle band to config: LOW -> rc_throttle_low_us."""
    return f'rc_throttle_{level.lower()}_us'
