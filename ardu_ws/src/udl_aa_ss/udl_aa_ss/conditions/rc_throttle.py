#!/usr/bin/env python3
"""
RC throttle condition - stub. Never fires.

When implemented: the throttle channel (constants.THROTTLE_CHANNEL) graded
against constants.RC_THROTTLE_TIERS. Ascending, first crossed wins; HIGH is the
fallback band and has no threshold, because anything above MEDIUM is normal
flight.

Only LOW warns, and that is deliberate - a throttle that far down is ArduPilot's
own failsafe event and it will change mode itself. Acting here would put a
second vehicle-mover in a race with the autopilot over the same fact, so this
reports and steps aside.

A zero or absent channel must read as no RC rather than as zero throttle, same
as rc_loss.
"""

from typing import Dict

from udl_aa_ss.constants import CLEAR, Verdict


def evaluate(telemetry, limits: Dict[str, float], state: Dict) -> Verdict:
    return CLEAR  # stub - never fires
