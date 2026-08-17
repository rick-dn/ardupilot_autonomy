# One atomic snapshot of vehicle state, mirroring
# mavros_interface.get_telemetry() group for group.
#
# Conventions throughout: ENU (east/north/up) for world-frame quantities,
# RFU (right/forward/up) for body-frame quantities, yaw CCW positive from East.
# MAVROS' FLU axes and CW-from-North bearings are converted on the way in, so
# nothing downstream has to know either convention exists.
#
# Published as a single message on purpose: a consumer subscribing to MAVROS
# directly would get thirteen independently-timed topics and have to reconcile
# fields sampled at different instants. Here every field comes from the same
# snapshot.

std_msgs/Header header

FcStatus        state
GlobalPosition  home
GlobalPosition  global_position
LocalPosition   local_position

# velocity_body comes from /mavros/local_position/velocity_body,
# velocity_body_odom from the twist half of /mavros/local_position/odom.
# They are separate sources for the same quantity and can disagree.
BodyVelocity    velocity_body
BodyVelocity    velocity_body_odom
EnuVelocity     velocity_gps

ImuData         imu
Battery         battery
GpsStatus       gps_status

float64         heading_deg   # compass, CCW positive from East

VfrHud          vfr_hud
RcInput         rc
StatusText      statustext
