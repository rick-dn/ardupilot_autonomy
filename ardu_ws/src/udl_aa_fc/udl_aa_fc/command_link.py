"""Transport to the sequence node: three topics, their QoS, and nothing that waits.

The commander decides which sequence should run; udl_aa_seqs runs it. This is
the only module that touches either side of that hop, so the QoS profiles have
exactly one place to be wrong.

Three endpoints, not two. Telemetry is subscribed here as well, because the
safety conditions are evaluated in the commander's own tick and need the
vehicle's state to evaluate against. It comes straight from the flight stack
rather than being relayed through the sequence node - telemetry is a broadcast
with no addressee, so a second consumer costs nothing and a relay would only add
a hop that can lag.

QoS is not a preference. Requests are RELIABLE because dropping one loses an
intent - and worse here than on the command topic, since a request is published
on change only, so a dropped one is never retried. Status and telemetry are
latest-value-wins state broadcasts; a BEST_EFFORT publisher does not match a
RELIABLE subscription at all, and DDS connects nothing and says nothing, so the
symptom would be silence rather than an error.

Nothing below blocks. Every method is a single read that returns immediately,
because the caller is a 20 Hz tick.

Two facts are published rather than checked by callers:

  live      the sequence node is up and the flight controller is reporting.
            Both limbs are single reads with no bookkeeping - a subscriber count
            DDS maintains, and a mode string the flight stack blanks when
            telemetry drops.

  status    what is running, straight off the wire. Kept whole rather than
            unpacked: the commander matches its own token against it, and
            whether a sequence is still going is the question it needs answered,
            not what phase it is in.
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from udl_aa_msgs.msg import (
    SequenceRequest,
    SequenceStatus,
    VehicleTelemetry,
)

TOPIC_REQUEST = '/sequence/request'
TOPIC_STATUS = '/sequence/status'
TOPIC_TELEMETRY = '/vehicle/telemetry'

REQUEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Status and telemetry are latest-value-wins state broadcasts. BEST_EFFORT is
# not a preference: both publishers are BEST_EFFORT, and a RELIABLE subscriber
# does not match one - DDS connects nothing and says nothing.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CommandLink:

    def __init__(self, node):
        self._node = node

        # Seeded rather than None: ROS has no null, so a default-constructed
        # message already reads sequence == '' and mode == '', which is exactly
        # what "nothing running" and "no telemetry" read as. Startup and
        # dropped-link are the same state, so nothing downstream needs a None
        # branch - which is also what lets every safety condition be written as
        # a plain function of the snapshot.
        self._status = SequenceStatus()
        self._telemetry = VehicleTelemetry()

        # Minted here rather than by the commander: this is the thing that
        # publishes, so the counter belongs with the wire. Starts at 1 because
        # the sequence node reports token 0 when nothing is running, which would
        # otherwise match a real request.
        self._token = 0

        self._pub = node.create_publisher(SequenceRequest, TOPIC_REQUEST,
                                          REQUEST_QOS)
        self._status_sub = node.create_subscription(
            SequenceStatus, TOPIC_STATUS, self._status_callback, STATE_QOS)
        self._telemetry_sub = node.create_subscription(
            VehicleTelemetry, TOPIC_TELEMETRY, self._telemetry_callback,
            STATE_QOS)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _status_callback(self, msg):
        self._status = msg

    def _telemetry_callback(self, msg):
        self._telemetry = msg

    # ------------------------------------------------------------------
    # Reads - all immediate
    # ------------------------------------------------------------------

    def status(self):
        """The latest sequence status, never None.

        Handed out whole so the commander can match `token` and read `sequence`
        and `state` from the same message - three fields that only mean anything
        together.
        """
        return self._status

    def telemetry(self):
        """The latest snapshot, never None.

        Handed out whole: every field in it was sampled at the same instant, so
        a condition can compare fields against each other without reconciling
        timestamps.
        """
        return self._telemetry

    @property
    def live(self):
        """The sequence node is up and the flight controller is reporting.

        The subscriber count covers the sequence node - it is the only
        subscriber to the request topic, so losing it drops this count. The mode
        string covers the vehicle behind it: the flight stack blanks it when
        telemetry stops, and a blank mode fails every sequence's precondition
        anyway.

        The count falls on DDS discovery timeout rather than instantly, so a
        hard kill takes a second or two to register here.
        """
        return (self._pub.get_subscription_count() > 0
                and self._telemetry.state.mode != '')

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def request(self, sequence, params=None):
        """Publish one sequence request and return its token.

        The token is returned rather than stored, because matching it is the
        commander's business: it is what distinguishes "the sequence I asked for
        is now running" from a stale status.

        `params` is passed through without validation. The sequence node holds
        the registry and is the only thing that can say whether a name exists or
        a parameter set fits it, so checking here would be guessing - and an
        unknown name is deliberately allowed onto the wire so a typo surfaces as
        a rejection rather than as silence.
        """
        params = params or {}

        self._token += 1
        msg = SequenceRequest()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.sequence = sequence
        msg.token = self._token
        # Zipped back into a dict by the sequence node; order is irrelevant,
        # pairing by index is not.
        msg.param_names = list(params.keys())
        msg.param_values = [float(params[name]) for name in msg.param_names]

        self._pub.publish(msg)
        self._node.get_logger().info(
            f'request token={self._token} {sequence} {params or ""}'.rstrip())
        return self._token
