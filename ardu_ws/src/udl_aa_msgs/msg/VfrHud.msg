# VFR HUD figures. heading_deg is converted to the stack's convention -
# CCW positive from East - unlike MAVLink's CW-from-North bearing.

float64 airspeed       # m/s
float64 groundspeed    # m/s
float64 heading_deg    # CCW positive from East
float64 throttle       # 0.0 - 1.0
float64 altitude       # metres AMSL
float64 climb          # m/s, positive up
