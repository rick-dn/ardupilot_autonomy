#!/usr/bin/env python3
"""
RC link condition - stub. Never fires.

When implemented: RC is considered lost when the channel values stop changing.
This is the one condition that cannot be answered from a single snapshot, so it
is the one that will use `state` - the last channel tuple and when it last
moved. A real receiver and real sticks jitter by a microsecond or two even held
still, so frozen values mean frames stopped arriving rather than a steady hand.

It will be a heuristic, deliberately: /vehicle/telemetry carries no receive
timestamp for the RC group, only for the snapshot as a whole. An empty channel
list must read as never having had RC rather than as losing it, so a vehicle
flown with no receiver at all never trips this.
"""

from typing import Dict

from udl_aa_ss.constants import CLEAR, Verdict


def evaluate(telemetry, limits: Dict[str, float], state: Dict) -> Verdict:
    return CLEAR  # stub - never fires
