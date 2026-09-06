#!/usr/bin/env python3
"""
Geofence condition - stub. Never fires.

When implemented: horizontal distance from the EKF origin and height above it,
each graded against constants.FENCE_TIERS, the worse of the two winning. Local
ENU throughout, so distance is from where the EKF initialised rather than from
the home the operator set, and altitude is above that origin rather than AGL.

Both axes read exactly zero before the first telemetry message, which resolves
as inside the fence - the safe answer, and why this will need no INVALID band
the way the battery does.

Thresholds are not written yet: FENCE_TIERS names them through
constants.fence_key, and the yaml gains fence_radius_* / fence_alt_* when this
becomes real.
"""

from typing import Dict

from udl_aa_ss.constants import CLEAR, Verdict


def evaluate(telemetry, limits: Dict[str, float], state: Dict) -> Verdict:
    return CLEAR  # stub - never fires
