"""GOTO_GLOBAL check: fly a list of WGS-84 waypoints, nose along each leg.

Coordinates came from GCS clicks, flown in click order. Add or remove points by
editing WAYPOINTS; headings and leg lengths derive from them.

alt is 10 m RELATIVE TO HOME, not AMSL. This is the trap in GOTO_GLOBAL: the
command takes home-relative altitude, while telemetry's global_position.alt is
AMSL, so the number sent here and the number read back are not comparable
without going through home.alt. The GCS showing 101 m for these points is AMSL
and is not what goes on the wire.

Heading is computed, not tabulated. yaw_deg is CCW positive from East, and
atan2(dnorth, deast) is already in exactly that convention - so no conversion
from the compass bearing the GCS prints. As a check, wp1 -> wp2 comes out
51.77 m at yaw 154.2, which is compass bearing 295.8 - matching the distance
and bearing the GCS reported for that click.

Every leg including the first has a real bearing, because HOME below stands in
for the start position. It is a constant, not a reading - nothing here
subscribes to telemetry, so arming somewhere other than HOME leaves the first
heading wrong and the rest unaffected.

Open loop, 30 s per leg, chosen for the size of the area rather than measured.
No status, no telemetry, no arrival check.

Preconditions the script cannot see: GUIDED, armed, in air, and a home position
set - GOTO_GLOBAL is meaningless without one.
"""

import math

import rclpy
from rclpy.node import Node

from udl_aa_seqs import vocabulary
from udl_aa_seqs.command_link import CommandLink

SEQUENCE = 'test_goto_global'

# Where the vehicle starts, so the first leg has a real bearing like the rest.
# Assumed, not measured - nothing here reads home from telemetry, so if the
# vehicle is armed somewhere else only the first heading is wrong.
HOME = ('home', 51.280503, -1.103467)

# (label, lat, lon) - WGS-84 degrees, in click order.
WAYPOINTS = [
    ('wp1', 51.28051551, -1.10402570),
    ('wp2', 51.28071784, -1.10469509),
    ('wp3', 51.28098761, -1.10400773),
    ('wp4', 51.28060262, -1.10340124),
    ('wp5', 51.28041715, -1.10302836),
    ('wp6', 51.28031318, -1.10316762),
]

# Metres per degree. Flat-earth approximation, good to well under a metre over
# the tens of metres these legs span; it is used only to derive headings, never
# to position the vehicle - the waypoints themselves go out as raw lat/lon.
M_PER_DEG_LAT = 111320.0


def sleep(node, seconds):
    """Spin for `seconds`, so callbacks keep running while we wait."""
    deadline = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def heading_deg(from_lat, from_lon, to_lat, to_lon):
    """Yaw facing along the leg, CCW positive from East.

    Longitude degrees shrink with latitude, hence the cosine; without it the
    east component is overstated by a third at this latitude and every heading
    comes out wrong.
    """
    mean_lat = math.radians((from_lat + to_lat) / 2.0)
    delta_east = (to_lon - from_lon) * M_PER_DEG_LAT * math.cos(mean_lat)
    delta_north = (to_lat - from_lat) * M_PER_DEG_LAT
    return math.degrees(math.atan2(delta_north, delta_east))


def main(args=None):
    rclpy.init(args=args)
    node = Node(SEQUENCE)

    altitude = node.declare_parameter('altitude', 10.0).value
    dwell = node.declare_parameter('dwell_s', 30.0).value

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

        # HOME is prepended as the starting point only; it is never flown to.
        legs = [HOME] + WAYPOINTS
        for index, (label, lat, lon) in enumerate(WAYPOINTS):
            _, prev_lat, prev_lon = legs[index]
            yaw = heading_deg(prev_lat, prev_lon, lat, lon)

            node.get_logger().info(
                f'-> {label} ({lat:.8f}, {lon:.8f}) alt {altitude:.1f} m rel '
                f'home, yaw {yaw:.1f}')
            token += 1
            link.publish(token, vocabulary.GOTO_GLOBAL, {
                'lat': lat,
                'lon': lon,
                'alt': altitude,
                'yaw_deg': yaw,
            })
            sleep(node, dwell)

        node.get_logger().info('waypoints issued - nothing was confirmed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
