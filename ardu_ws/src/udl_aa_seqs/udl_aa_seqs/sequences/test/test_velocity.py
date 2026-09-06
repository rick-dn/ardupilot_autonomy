"""SET_VELOCITY check: the ENU diamond again, flown on timed velocity legs.

Same five waypoints as test_goto_local.py - north, east, south, west, center,
each 10 m from the origin - but reached by holding a velocity for a computed
time instead of commanding a position.

              north (0, 10)
                   /\
    west (-10, 0) <  > east (10, 0)    then back to center (0, 0)
                   \/
              south (0, -10)

Speed is fixed at 2 m/s and each leg's duration comes from its length, so the
shape is preserved: the two axis legs are 10 m and take 5 s, the four diagonals
are 14.14 m and take ~7.07 s. Holding every leg to 5 s instead would fly a
lopsided figure, since the diagonals are longer than the axis legs.

Yaw is exercised at the ends, not during the legs. SET_VELOCITY carries
yaw_rate, not yaw_deg - mavros_interface publishes the stream as a Twist whose
angular.z is an angular velocity, and there is no yaw-angle field on it. So the
nose cannot be pointed along a leg the way the goto tests did it; the legs run
at yaw_rate=0.0 and the heading stays wherever it was.

What a rate can do is spin. The run opens with a full 360 CW and closes with a
full 360 CCW, each flown as zero translation plus a yaw_rate held for a fixed
time - the angle is the integral of rate over duration, so the revolution is
timed rather than commanded. Note this is the one place a full turn actually
works: GOTO_BODY's 360 wrapped to zero and did nothing, because an angle wraps
and an integrated rate does not.

Distance here is dead reckoned, not measured. The vehicle ramps in and out of
each setpoint, so every leg falls short and the diamond will not close exactly.
The goto tests were absolute and self-correcting; this one accumulates error by
construction.

STOP_VELOCITY at the end is mandatory, not tidiness. SET_VELOCITY starts a
10 Hz setpoint stream that persists until stopped, and ArduPilot's GUID_TIMEOUT
is 3.0 s - exiting without stopping leaves the vehicle flying the last setpoint
for up to 3 more seconds, about 6 m at this speed. The finally block covers
ctrl-C for the same reason.

Preconditions the script cannot see: GUIDED, armed, in air.
"""

import math

import rclpy
from rclpy.node import Node

from udl_aa_seqs import vocabulary
from udl_aa_seqs.command_link import CommandLink

SEQUENCE = 'test_velocity'
LEG_M = 10.0

# (label, east, north) - absolute, metres from the local origin. Identical to
# test_goto_local.py; only the way we get there differs.
PATTERN = [
    ('north', 0.0, LEG_M),
    ('east', LEG_M, 0.0),
    ('south', 0.0, -LEG_M),
    ('west', -LEG_M, 0.0),
    ('center', 0.0, 0.0),
]


def sleep(node, seconds):
    """Spin for `seconds`, so callbacks keep running while we wait."""
    deadline = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Node(SEQUENCE)

    speed = node.declare_parameter('speed_mps', 2.0).value
    spin_s = node.declare_parameter('spin_s', 10.0).value

    link = CommandLink(node)
    token = 0
    sent_velocity = False

    def spin(label, ccw):
        """One full revolution, held as a rate for spin_s seconds.

        yaw_rate is rad/s and CCW positive, so a CW turn is negative. Rate is
        derived from the duration rather than fixed, which keeps the turn a
        full 2*pi however spin_s is set.
        """
        nonlocal sent_velocity, token
        rate = (1.0 if ccw else -1.0) * 2.0 * math.pi / spin_s
        node.get_logger().info(
            f'-> {label} at {math.degrees(rate):+.0f} deg/s for {spin_s:.0f}s')
        token += 1
        link.publish(token, vocabulary.SET_VELOCITY, {
            'east': 0.0,
            'north': 0.0,
            'up': 0.0,
            'yaw_rate': rate,
        })
        sent_velocity = True
        sleep(node, spin_s)

    try:
        # VOLATILE profile: anything published before the vehicle has matched
        # us is not backfilled, it is simply gone.
        sleep(node, 2.0)
        if not link.live:
            node.get_logger().error(
                'vehicle link down - is the flight stack running?')
            return

        spin('spin 360 cw', ccw=False)

        # Legs are chained from the origin, where takeoff leaves the vehicle.
        east, north = 0.0, 0.0
        for label, target_east, target_north in PATTERN:
            delta_east = target_east - east
            delta_north = target_north - north
            distance = math.hypot(delta_east, delta_north)
            duration = distance / speed

            node.get_logger().info(
                f'-> {label} ({distance:.1f} m at {speed:.1f} m/s, '
                f'{duration:.1f}s)')
            # Each SET_VELOCITY replaces the running setpoint, so legs chain
            # directly and only the final stop is needed.
            token += 1
            link.publish(token, vocabulary.SET_VELOCITY, {
                'east': delta_east / duration,
                'north': delta_north / duration,
                'up': 0.0,
                'yaw_rate': 0.0,
            })
            sent_velocity = True
            sleep(node, duration)
            east, north = target_east, target_north

        spin('spin 360 ccw', ccw=True)

        node.get_logger().info('diamond issued - nothing was confirmed')
    finally:
        if sent_velocity:
            token += 1
            link.publish(token, vocabulary.STOP_VELOCITY)
            # Let the stop reach the vehicle before the publisher goes away.
            sleep(node, 1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
