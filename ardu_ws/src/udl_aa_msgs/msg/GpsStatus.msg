# Raw GPS status from /mavros/gpsstatus/gps1/raw.
# fix_type follows MAVLink GPS_FIX_TYPE: 0 no-GPS, 1 no-fix, 2 2D, 3 3D,
# 4 DGPS, 5 RTK-float, 6 RTK-fixed.
# eph/epv are horizontal/vertical dilution of precision, unitless * 100.

uint8  fix_type
uint8  satellites_visible
uint16 eph
uint16 epv
