"""Transport to the commander: two topics, their QoS, and nothing that waits.

The other half of the hop `udl_aa_fc/command_link.py` publishes on. This is the
only module here that touches it, so the QoS profiles have exactly one place to
be wrong.

Not to be confused with command_link.py in this package, which is the link
*downward* to the flight stack. This one faces up.

QoS is not a preference. Requests are RELIABLE because the commander publishes
on change only, so a dropped request is never retried - losing one would leave
a safety sequence un-run with nothing to notice. Status is BEST_EFFORT
KEEP_LAST(1) because it is a latest-value-wins heartbeat, and a RELIABLE
subscriber would not match a BEST_EFFORT publisher at all.

Requests are latched in the callback and consumed on read, newest wins. Latching
is what makes one impossible to miss between ticks; consuming it is what makes
it impossible to act on twice. Newest-wins is right rather than lossy: a request
is an intent about what should be running *now*, so an older one that has been
superseded was never worth acting on - a d-pad held through three ticks is three
requests for the same thing, and only the last says anything true.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from udl_aa_msgs.msg import SequenceRequest, SequenceStatus

TOPIC_REQUEST = '/sequence/request'
TOPIC_STATUS = '/sequence/status'

REQUEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CommanderLink:

    def __init__(self, node):
        self._node = node
        self._pending = None

        self._sub = node.create_subscription(
            SequenceRequest, TOPIC_REQUEST, self._request_callback, REQUEST_QOS)
        self._pub = node.create_publisher(SequenceStatus, TOPIC_STATUS,
                                          STATUS_QOS)

    def _request_callback(self, msg):
        self._pending = msg

    def request(self):
        """The newest request since the last call, or None. Consumed on read."""
        pending, self._pending = self._pending, None
        return pending

    def publish_status(self, sequence, state, token):
        """Publish the current state.

        Called every tick without exception - it is a heartbeat, not a
        notification. The commander's rule is to deny a new sequence while one
        is running, so it needs "nothing is running" as a positive fact, and an
        event-only topic could not give it that after a dropped message.
        """
        msg = SequenceStatus()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.sequence = sequence
        msg.state = state
        msg.token = token
        self._pub.publish(msg)
