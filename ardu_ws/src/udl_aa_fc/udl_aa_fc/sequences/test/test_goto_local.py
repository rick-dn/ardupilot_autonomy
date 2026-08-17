"""GOTO_LOCAL check: a diamond around the origin, nose into the turn.

Five waypoints, each 10 m from the local origin except the last:

              north (0, 10)
                   /\
    west (-10, 0) <  > east (10, 0)    then back to center (0, 0)
                   \/
              south (0, -10)

Waypoints are absolute positions in the local ENU frame, not offsets from
wherever the vehicle happens to be - GOTO_LOCAL is measured from the origin,
so the pattern only looks like a diamond if the vehicle starts near it. Take
off first and do not fly it anywhere else in between.

Open loop, like takeoff.py: no status, no telemetry, no arrival check. Each
command gets a fixed dwell and then the next one goes out regardless of
whether the vehicle got there. Watch it in the GCS - that is the test.

Preconditions the script cannot see: GUIDED, armed, and already in air.
GOTO_LOCAL is only permitted armed and in air, so running this on the ground
gets every leg rejected with cond=['COMMAND_STATE'] while this reports nothing.
"""

import math

import rclpy
from rclpy.node import Node

from udl_aa_fc import vocabulary
from udl_aa_fc.command_link import CommandLink

SEQUENCE = 'test_goto_local'
LEG_M = 10.0

# (label, east, north) - absolute, metres from the local origin.
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


def heading_deg(from_east, from_north, to_east, to_north):
    """Yaw that faces along the leg, CCW positive from East.

    atan2 gives exactly that convention with no correction: atan2(dnorth,
    deast) is 0 due East and grows counter-clockwise, which is what the stack
    wants. Nothing here converts to a compass bearing - MAVROS' CW-from-North
    is handled on the vehicle side and never reaches us.
    """
    return math.degrees(math.atan2(to_north - from_north, to_east - from_east))


def main(args=None):
    rclpy.init(args=args)
    node = Node(SEQUENCE)

    altitude = node.declare_parameter('altitude', 5.0).value
    dwell = node.declare_parameter('dwell_s', 10.0).value

    link = CommandLink(node)
    token = 0
    try:
        # VOLATILE profile: anything published before the vehicle has matched
        # us is not backfilled, it is simply gone.
        sleep(node, 2.0)
        if not link.live:
            node.get_logger().error(
                'vehicle link down - is the flight stack running?')
            return

        # The pattern starts from the origin, which is where takeoff leaves the
        # vehicle, so that is what the first heading is measured from.
        east, north = 0.0, 0.0
        for label, target_east, target_north in PATTERN:
            yaw = heading_deg(east, north, target_east, target_north)
            node.get_logger().info(
                f'-> {label} ({target_east:.1f}E, {target_north:.1f}N) '
                f'yaw {yaw:.1f}')
            token += 1
            link.publish(token, vocabulary.GOTO_LOCAL, {
                'east': target_east,
                'north': target_north,
                'up': altitude,
                'yaw_deg': yaw,
            })
            sleep(node, dwell)
            east, north = target_east, target_north

        node.get_logger().info('pattern issued - nothing was confirmed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
