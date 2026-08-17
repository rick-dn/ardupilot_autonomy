# Flight-controller state.
#
# armed and mode come straight off /mavros/state. mode is the raw ArduPilot
# mode string, e.g. GUIDED, STABILIZE, RTL - not the bucket the fsm arbitrates
# on, so a consumer sees what the FC actually reports rather than the stack's
# interpretation of it.
#
# in_air is NOT reported by the FC - there is no such telemetry field. It is
# derived on the vehicle from the local ENU altitude (height above the EKF
# origin, not AGL and not AMSL) and latched with hysteresis: true above 0.5 m,
# false below 0.2 m, hold in between. It is published because the command
# permission table is keyed on it, and re-deriving it commander-side would
# duplicate vehicle logic that could then disagree at the latch boundary.
# Note the false branch is altitude-only, so disarming at height leaves in_air
# true until the altitude reading comes down.

bool   armed
string mode
bool   in_air
