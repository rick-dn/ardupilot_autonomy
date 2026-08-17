# IMU, already converted out of MAVROS' FLU convention.
# yaw_deg is CCW positive from East, derived from the ENU orientation
# quaternion. Gyro and accel are body RFU: rad/s and m/s^2.

float64 yaw_deg
float64 gyro_right
float64 gyro_forward
float64 gyro_up
float64 accel_right
float64 accel_forward
float64 accel_up
