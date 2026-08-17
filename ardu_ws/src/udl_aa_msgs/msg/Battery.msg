# percentage is 0.0-1.0 as MAVROS publishes it, NOT 0-100. The flight logger
# multiplies by 100 for display; consumers of this message must do the same.

float64 voltage      # volts
float64 current      # amps
float64 percentage   # 0.0 - 1.0
