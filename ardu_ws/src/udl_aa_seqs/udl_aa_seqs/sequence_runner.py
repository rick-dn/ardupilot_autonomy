"""Runs one sequence at a time and reports what it is doing.

The executive layer. It holds the registry and the lifecycle, and it decides
nothing: which sequence should run arrives from the commander, and this runs
whatever it is told. Sequences own their phases; it never inspects one, and a
sequence reports only RUNNING, COMPLETE or ABORT.

One at a time, structurally. This is the only thing that runs sequences and the
only thing that publishes commands, so two sequences cannot be flying the
vehicle at once no matter how many things ask - which is what moving them out of
the commanders bought.

A request displaces whatever is running, but not in the same tick that it stops
it. on_exit() may send a command - a sequence that opened a velocity stream has
to close it - and the flight stack holds exactly one submission per tick, so
starting the replacement immediately would overwrite that command before it was
ever arbitrated. Stopping takes this tick; starting takes the next, 50 ms later.

Rejection is the one signal that arrives from outside the tick. command_link
latches it in the status callback, because the vehicle echoes a token on exactly
the one tick that arbitrated it - sampling the status topic from this tick would
miss it and the sequence would run on believing a command was flying.
"""

import rclpy
from udl_aa_msgs.msg import SequenceStatus

from udl_aa_seqs.command_link import CommandLink
from udl_aa_seqs.commander_link import CommanderLink
from udl_aa_seqs.sequences.abort import AbortSequence
from udl_aa_seqs.sequences.land import LandSequence
from udl_aa_seqs.sequences.manual import ManualSequence
from udl_aa_seqs.sequences.rtl import RtlSequence
from udl_aa_seqs.sequences.smart_rtl import SmartRtlSequence
from udl_aa_seqs.sequences.takeoff import TakeoffSequence

NODE_NAME = 'udl_aa_seqs'

# Matches the vehicle's own rate: the flight stack arbitrates one submission per
# 20 Hz tick, so there is nothing to gain by acting faster than it does.
TICK_RATE_HZ = 20.0

# name -> (seq_id, class). The name is the wire contract: it is what the
# commander puts in a SequenceRequest, and this table is authoritative for which
# names exist. Plain strings rather than a shared constant, because importing
# the commander's vocabulary here would make the executive layer depend on the
# layer above it.
#
# seq_id lands in the high bits of every token a sequence sends to the flight
# stack, which is what lets a rejection on the shared status topic be attributed
# to one sequence rather than another. Ids are fixed and never reused.
SEQUENCES = {
    # seq_id 0: a manual token is just its command counter, which starts at 1,
    # so token 0 still never appears on the wire.
    'manual': (0, ManualSequence),
    'takeoff': (1, TakeoffSequence),
    'smart_rtl': (2, SmartRtlSequence),
    'rtl': (3, RtlSequence),
    'land': (4, LandSequence),
    'abort': (9, AbortSequence),
    # 'emergency_land': (5, EmergencyLandSequence),
    # 'motor_cutoff':   (6, MotorCutoffSequence),
}


class SequenceRunner:

    def __init__(self, node, fs, commander):
        self._node = node
        self._fs = fs
        self._commander = commander

        # Built once and reused. on_start() is what resets a sequence for its
        # next run, so a second takeoff is the same object, not a new one.
        self._registry = {
            name: cls(seq_id, fs, self._log)
            for name, (seq_id, cls) in SEQUENCES.items()
        }

        self._active = None
        self._pending = None

        # What the commander is told. The terminal state is held rather than
        # cleared, so a completed sequence stays visible until something else is
        # asked for - the commander distinguishes "ended" from "never ran" by
        # the token, and clearing it immediately would erase that.
        self._sequence = ''
        self._state = ''
        self._token = 0

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def tick(self):
        """One pass. Never blocks and never waits on a command."""
        incoming = self._commander.request()
        if incoming is not None:
            self._pending = incoming

        # Checked here rather than in any sequence. Both limbs are reads on the
        # link, so this costs nothing and every sequence gets it for free.
        if not self._fs.live:
            self._stop('vehicle link down')

        # Ordered before update() on purpose: a rejected command was never
        # dispatched, so letting the sequence tick once more would have it
        # waiting on telemetry that is never going to move.
        rejection = self._fs.rejection()
        if rejection is not None and self._active is not None:
            if rejection.token == self._active.token:
                self._stop('rejected - ' + ', '.join(rejection.cond))

        self._service_pending()

        if self._active is not None:
            status = self._active.update()
            if status == self._active.COMPLETE:
                self._stop('complete', SequenceStatus.COMPLETE)
            elif status == self._active.ABORT:
                self._stop('failed', SequenceStatus.ABORT)

        self._commander.publish_status(self._sequence, self._state, self._token)

    def _service_pending(self):
        """Start what was asked for, displacing whatever is running first.

        The return after _stop is the whole point: on_exit's command needs this
        tick to itself, because the flight stack holds one submission and clears
        it on its next tick. Starting now would overwrite a STOP_VELOCITY that
        then never goes out, and ArduPilot would hold the last setpoint for
        GUID_TIMEOUT instead.
        """
        if self._pending is None:
            return

        if self._active is not None:
            self._stop(f'preempted by {self._pending.sequence}')
            return

        request, self._pending = self._pending, None
        self._start(request)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _start(self, request):
        """Run what the request names, or report that it cannot be run.

        An unknown name is answered rather than ignored: the commander
        deliberately lets one onto the wire so a typo surfaces, and ABORT with
        the request's own token is how it surfaces.
        """
        sequence = self._registry.get(request.sequence)
        self._sequence = request.sequence
        self._token = request.token

        if sequence is None:
            self._state = SequenceStatus.ABORT
            self._node.get_logger().error(
                f'unknown sequence: {request.sequence!r}')
            return

        params = dict(zip(request.param_names, request.param_values))
        self._active = sequence
        self._state = SequenceStatus.RUNNING
        sequence.on_start(params)
        self._log(f'{request.sequence} started'
                  + (f' {params}' if params else ''))

    def _stop(self, reason, state=SequenceStatus.ABORT):
        """The only way a sequence ends. Logs, unwinds, forgets.

        The forgetting is in a finally because an on_exit() that raises would
        otherwise leave this believing a dead sequence is still running -
        nothing new could start and the commander would be told the vehicle was
        held, until the node was restarted.
        """
        if self._active is None:
            return
        self._log(f'{self._sequence} {reason}')
        self._state = state
        try:
            self._active.on_exit()
        finally:
            self._active = None

    def _log(self, text):
        self._node.get_logger().info(text)


def main(args=None):
    """The runner is the node. One timer and a spin.

    Single-threaded executor, deliberately: the tick and the subscription
    callbacks never overlap, so the rejection latch, the telemetry cache and the
    pending request need no locking.
    """
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME)

    runner = SequenceRunner(node, CommandLink(node), CommanderLink(node))
    node.create_timer(1.0 / TICK_RATE_HZ, runner.tick)

    node.get_logger().info(f'{NODE_NAME} up - {TICK_RATE_HZ:.0f} Hz tick')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # plain print, not get_logger() - rclpy's SIGINT handler has already
        # torn the context down by now, so a /rosout publish would fail.
        print('Shutting down sequence runner')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
