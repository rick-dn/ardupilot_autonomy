"""Transport to the vehicle: the three topics, their QoS, and nothing that waits.

This is the only module that publishes a VehicleCommand or touches the status
and telemetry topics, so the QoS profiles have exactly one place to be wrong.

QoS is not a preference here. Commands are RELIABLE because dropping one loses
an operator intent, and a BEST_EFFORT publisher would not match the vehicle's
RELIABLE subscription at all - DDS connects nothing and says nothing, so the
symptom is a commander that publishes into the void rather than an error.

Nothing below blocks. Every method is a single read that returns immediately,
because the caller is a 20 Hz tick: a method that waited would stall the tick
that polls the operator's abort, and calling spin_once from inside a callback
the executor is already spinning is invalid regardless of how briefly it waits.

Two facts are published rather than checked by callers:

  live      the vehicle stack is up and the FC is reporting. Both limbs are
            single reads with no bookkeeping - a subscriber count that DDS
            maintains, and a mode string the adapter blanks when telemetry
            drops. No sequence ever checks either one; the commander gates on
            this once per tick and ends whatever is running.

  rejection the one status worth keeping. Status publishes every tick as a
            heartbeat and echoes a token on exactly the tick that arbitrated
            it, so a rejection exists for one message and is gone. Latching it
            in the callback is what makes it impossible to miss; consuming it
            on read is what makes it impossible to act on twice.
"""

import dataclasses
from typing import List

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from udl_aa_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    VehicleTelemetry,
)

from udl_aa_seqs import vocabulary

TOPIC_COMMAND = '/vehicle/command'
TOPIC_STATUS = '/vehicle/status'
TOPIC_TELEMETRY = '/vehicle/telemetry'

COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# Status and telemetry are latest-value-wins state broadcasts. BEST_EFFORT is
# not a preference: the vehicle publishes both BEST_EFFORT, and a RELIABLE
# subscriber does not match a BEST_EFFORT publisher - DDS connects nothing and
# says nothing, so the symptom is silence rather than an error.
STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclasses.dataclass
class Rejection:
    """A refused submission. `command` is empty unless safety outranked us."""

    token: int
    cond: List[str]
    command: str


class CommandLink:

    def __init__(self, node):
        self._node = node

        # Seeded rather than None: ROS has no null, so a default-constructed
        # message already reads mode == '', which is exactly what a dropped
        # link reads. Startup-before-first-message and telemetry-lost are the
        # same state, so nothing downstream needs a None branch.
        self._telemetry = VehicleTelemetry()
        self._rejection = None

        self._pub = node.create_publisher(VehicleCommand, TOPIC_COMMAND,
                                          COMMAND_QOS)
        self._status_sub = node.create_subscription(
            VehicleStatus, TOPIC_STATUS, self._status_callback, STATE_QOS)
        self._telemetry_sub = node.create_subscription(
            VehicleTelemetry, TOPIC_TELEMETRY, self._telemetry_callback,
            STATE_QOS)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _status_callback(self, msg):
        """Keep refusals, discard the heartbeat.

        No token matching here. Whose rejection it is depends on which
        sequence is running, which this module has no business knowing - the
        commander compares it against the active sequence's token.
        """
        if msg.cmd_status != VehicleStatus.REJECTED:
            return
        self._rejection = Rejection(msg.token, list(msg.cond), msg.command)

    def _telemetry_callback(self, msg):
        self._telemetry = msg

    # ------------------------------------------------------------------
    # Reads - all immediate
    # ------------------------------------------------------------------

    def telemetry(self):
        """The latest snapshot, never None.

        Handed out whole: every field in it was sampled at the same instant, so
        a caller can compare fields against each other without reconciling
        timestamps.
        """
        return self._telemetry

    @property
    def live(self):
        """The vehicle stack is up and the FC is reporting.

        The subscriber count covers the stack: all three endpoints are created
        on one node over there, so losing it drops this count whichever topic
        died. The mode string covers the FC behind it: the adapter blanks it
        when telemetry stops, and a blank mode fails every sequence's
        precondition anyway.

        The count falls on DDS discovery timeout rather than instantly, so a
        hard kill takes a second or two to register here.
        """
        return (self._pub.get_subscription_count() > 0
                and self._telemetry.state.mode != '')

    def rejection(self):
        """The refusal since the last call, or None. Consumed on read."""
        rejection, self._rejection = self._rejection, None
        return rejection

    # ------------------------------------------------------------------
    # Write
    # ------------------------------------------------------------------

    def publish(self, token, command, params=None):
        """Publish one command under the caller's token.

        The token is minted by the sequence, not here - it carries the
        sequence id in its high bits, which is what lets a rejection on this
        shared topic be attributed to one sequence rather than another.

        `params` must be exactly the set the command expects; vocabulary
        raises otherwise, so a wrong parameter set fails at the call site with
        the offending names rather than returning as an anonymous MALFORMED.
        """
        params = params or {}
        vocabulary.validate(command, params)

        msg = VehicleCommand()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # Identity only, never addressing. It reaches the vehicle's log line
        # when a message is desk-rejected, and nowhere else.
        msg.sequence = self._node.get_name()
        msg.token = token
        msg.command = command
        # Zipped back into a dict on the vehicle; order is irrelevant, pairing
        # by index is not.
        msg.param_names = list(params.keys())
        msg.param_values = [float(params[name]) for name in msg.param_names]

        self._pub.publish(msg)
        self._node.get_logger().info(
            f'sent token={token} {command} {params or ""}'.rstrip())
