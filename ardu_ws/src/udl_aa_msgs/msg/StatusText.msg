# Last STATUSTEXT received from the flight controller. Latched - it holds the
# most recent message indefinitely, so a consumer cannot infer from this field
# alone that something was said recently.
# severity follows MAVLink MAV_SEVERITY: 0 emergency ... 7 debug.

uint8  severity
string text
