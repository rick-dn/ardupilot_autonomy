# RC input as received by the flight controller.
# channels are raw PWM microseconds; the array is empty until the first
# /mavros/rc/in message arrives.

uint16[] channels
uint8    rssi
