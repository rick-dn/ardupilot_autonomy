"""GOTO_BODY check: spin, trace a cross, spin back, end where you started.

  yaw CCW 360   forward 10, back 20, forward 10    (out and back on forward)
                right 10, left 20, right 10        (out and back on right)
  yaw CW 360

Every offset sums to zero on both axes, and the translation legs all command
yaw_deg=0, so the heading is held and the body frame does not rotate underneath
them. That is what makes the net displacement zero rather than approximately
zero - if the legs turned the vehicle, each subsequent offset would be measured
in a different frame and the cross would not close.

Two things this file does not do naively:

  A single 360 deg yaw is a no-op. mavros_interface.goto_body passes yaw
  through as radians and ArduPilot wraps it, so 360 becomes 0 and the vehicle
  never turns. Each revolution is issued as YAW_STEPS increments instead. Set
  YAW_STEPS = 1 and YAW_STEP_DEG = 360.0 to watch it do nothing.

  Yaw is relative here, unlike GOTO_LOCAL. The frame is BODY_OFFSET_NED, for
  which ArduPilot interprets yaw as an offset from the current heading. So +90
  means turn 90 deg CCW, not face 90 deg.

Open loop, like the rest: no status, no telemetry, no arrival check. That
matters more here than it did for GOTO_LOCAL - local targets are absolute, so a
leg that undershot was silently corrected by the next one. Body offsets are
relative to wherever the vehicle actually is, so undershoot accumulates and the
cross drifts. If it does not close, lengthen the dwell before suspecting the
conventions.

Preconditions the script cannot see: GUIDED, armed, in air.
"""

import rclpy
from rclpy.node import Node

from udl_aa_seqs import vocabulary
from udl_aa_seqs.command_link import CommandLink

SEQUENCE = 'test_goto_body'
LEG_M = 10.0

# One revolution, in increments small enough to survive yaw wrapping.
YAW_STEP_DEG = 90.0
YAW_STEPS = 4

# (label, right, forward, yaw_deg). up is 0 throughout - hold altitude.
# Note the 20 m legs sit exactly on the GOTO_BODY sanity bound of 20.0.
PATTERN = (
    [(f'yaw ccw {YAW_STEP_DEG:.0f}', 0.0, 0.0, YAW_STEP_DEG)] * YAW_STEPS
    + [
        ('forward 10', 0.0, LEG_M, 0.0),
        ('back 20', 0.0, -2 * LEG_M, 0.0),
        ('forward 10', 0.0, LEG_M, 0.0),
        ('right 10', LEG_M, 0.0, 0.0),
        ('left 20', -2 * LEG_M, 0.0, 0.0),
        ('right 10', LEG_M, 0.0, 0.0),
    ]
    + [(f'yaw cw {YAW_STEP_DEG:.0f}', 0.0, 0.0, -YAW_STEP_DEG)] * YAW_STEPS
)


def sleep(node, seconds):
    """Spin for `seconds`, so callbacks keep running while we wait."""
    deadline = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Node(SEQUENCE)

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

        for label, right, forward, yaw in PATTERN:
            node.get_logger().info(
                f'-> {label} (R={right:.1f}, F={forward:.1f}, yaw={yaw:+.0f})')
            token += 1
            link.publish(token, vocabulary.GOTO_BODY, {
                'right': right,
                'forward': forward,
                'up': 0.0,
                'yaw_deg': yaw,
            })
            sleep(node, dwell)

        node.get_logger().info(
            'pattern issued - should be back at the start, unconfirmed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
