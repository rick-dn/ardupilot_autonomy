"""Decides which sequence should be running, and asks for it.

Both the nominal and the safety commander; there is no separate safety node. It
runs no sequences - udl_aa_seqs does that. The whole decision each tick is
whether to put a request on the wire.

Safety conditions are evaluated here, inline, in the tick. They are pure table
walks over a cached telemetry snapshot, so four of them cost microseconds and
the verdicts are fresh rather than read from somewhere. Each one keeps a scratch
dict across ticks, which is what lets a condition hold history - rc_loss needs
the last channel values - without this module knowing how any of them work. The
signature is deliberately the same one a thread would call, so a condition that
later gets expensive (polygonal geofencing) can be moved onto its own thread
without anything else changing.

One ranking over every candidate, from conditions and from the operator alike.
Ranking safety first and then ranking the winner against the operator is two
stages that can disagree about what should happen; one table cannot. See
arbitrate.py.

Publishing on change only is what keeps a standing condition from becoming a
20 Hz request storm, and it also means a sequence that ended is not immediately
asked for again: the candidate has not changed, so nothing is published, and a
battery that is still critical after landing does not land twice.
"""

import rclpy

from udl_aa_fc import arbitrate, gui_link
from udl_aa_fc.command_link import CommandLink
from udl_aa_fc.conditions import battery, geofence, rc_loss, rc_throttle
from udl_aa_fc.constants import Condition, Message, Sequence
from udl_aa_fc.gui_link import GuiLink

NODE_NAME = 'udl_aa_fc'

# The tick matches the vehicle's own rate: the flight stack arbitrates one
# submission per 20 Hz tick, so there is nothing to gain by deciding faster than
# it can act.
TICK_RATE_HZ = 20.0

# Telemetry to the operator runs slower and on its own timer. The source is
# 5 Hz; publishing at tick rate would emit four byte-identical messages per new
# datum.
TELEMETRY_RATE_HZ = 5.0

# One row per condition. Adding a condition is a module in conditions/, a row
# here, and a row in arbitrate.PRIORITY - nothing else in the stack changes.
CONDITIONS = {
    Condition.BATTERY: battery.evaluate,
    Condition.GEOFENCE: geofence.evaluate,
    Condition.RC_LOSS: rc_loss.evaluate,
    Condition.RC_THROTTLE: rc_throttle.evaluate,
}

# A released d-pad is a manual request with every axis at zero - the same
# mechanism as a press, not a special case. The sequence sends it once and
# completes, so releasing both stops the vehicle and frees the sequence slot.
STOP_AXES = {'east': 0.0, 'north': 0.0, 'up': 0.0, 'yaw_rate': 0.0}


class VehicleCommander:

    def __init__(self, node, fs, gui, limits):
        self._node = node
        self._fs = fs
        self._gui = gui
        self._limits = limits

        # One scratch dict per condition, persisting across ticks. Only rc_loss
        # uses it today; it is in the contract so a condition can keep history
        # without the caller knowing which do.
        self._scratch = {name: {} for name in CONDITIONS}

        # What was last put on the wire, for change detection. None means
        # nothing has been requested, which is not the same as nothing running.
        self._requested = None
        self._reported = ()

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def tick(self):
        """One pass. Never blocks and never waits on anything."""
        telemetry = self._fs.telemetry()

        candidates = self._conditions(telemetry) + self._operator()
        self._report(candidates)

        # Checked after the conditions so a warning is still logged when the
        # link is down, and before anything is published because a request into
        # a dead link is a request that is never answered.
        if not self._fs.live:
            return

        winner = arbitrate.arbitrate(
            (sequence, message) for sequence, _, message in candidates)
        if winner is None:
            self._requested = None
            return

        params = next(p for s, p, _ in candidates if s == winner)
        if self._allowed(winner) and self._changed(winner, params):
            self._fs.request(winner, params)
            self._requested = (winner, tuple(sorted(params.items())))

    def _conditions(self, telemetry):
        """Every condition, evaluated inline. Returns candidate tuples."""
        return [
            (verdict.message.sequence, {}, verdict.message)
            for verdict in (
                evaluate(telemetry, self._limits, self._scratch[name])
                for name, evaluate in CONDITIONS.items())
            if verdict.message is not Message.NONE
        ]

    def _operator(self):
        """The operator's request, if any, as a candidate tuple.

        STOP is a manual request with zero axes rather than an action of its
        own - see STOP_AXES. ABORT names the abort sequence, which arbitrate
        ranks above everything including safety.
        """
        request = self._gui.poll()
        if request is None:
            return []
        if request.action == gui_link.START:
            return [(request.name, request.params or {}, None)]
        if request.action == gui_link.STOP:
            return [(Sequence.MANUAL, dict(STOP_AXES), None)]
        return [(Sequence.ABORT, {}, None)]

    # ------------------------------------------------------------------
    # The two gates
    # ------------------------------------------------------------------

    def _allowed(self, winner):
        """Is this allowed to displace what is already running?

        A running sequence is denied to everything except an abort, the same
        sequence again, or a strictly more severe one. Equal priority never
        displaces, which is what stops a condition flickering across its
        threshold from restarting anything, and stops a clearing condition from
        abandoning the response to it.

        The same sequence is always allowed, because that is not displacement -
        it is an update to what is already running, and it is how a d-pad press
        changes axis and how a release sends its zeros. Denying it on equal
        priority would leave manual holding its last setpoint until
        GUID_TIMEOUT. Repeats cost nothing anyway: _changed drops the ones that
        carry no new information.
        """
        status = self._fs.status()
        if status.state != status.RUNNING:
            return True
        if winner == Sequence.ABORT or winner == status.sequence:
            return True
        return (arbitrate.PRIORITY.get(winner, arbitrate.UNKNOWN)
                > arbitrate.PRIORITY.get(status.sequence, arbitrate.UNKNOWN))

    def _changed(self, winner, params):
        """Has the request changed since the last one published?

        A standing condition asks every tick; republishing at tick rate would
        make the sequence node responsible for deduplicating. Manual is the one
        sequence whose repeats are real rather than redundant, and it falls out
        of this for free: its axes are part of the request, so a different
        press is a different request and a held one is not.
        """
        return self._requested != (winner, tuple(sorted(params.items())))

    # ------------------------------------------------------------------
    # Outbound
    # ------------------------------------------------------------------

    def publish_telemetry(self):
        """Operator-facing snapshot. Own timer, matched to the source rate."""
        telemetry = self._fs.telemetry()
        status = self._fs.status()

        self._gui.publish_telemetry({
            'east': telemetry.local_position.east,
            'north': telemetry.local_position.north,
            'up': telemetry.local_position.up,
            'yaw_deg': telemetry.local_position.yaw_deg,
            'vel_east': telemetry.velocity_gps.east,
            'vel_north': telemetry.velocity_gps.north,
            'vel_up': telemetry.velocity_gps.up,
            # The stack reports 0.0 - 1.0; the page shows a percentage.
            'batt': telemetry.battery.percentage * 100.0,
            'armed': telemetry.state.armed,
            'in_air': telemetry.state.in_air,
            'mode': telemetry.state.mode,
            'mission': status.sequence or None,
        })

    def _report(self, candidates):
        """Log every active condition, on change rather than per tick.

        All of them, not just the winner: the rest are how the operator sees
        what else is wrong while one thing is being acted on.
        """
        current = tuple(message.text for _, _, message in candidates
                        if message is not None)
        if current == self._reported:
            return
        self._reported = current
        self._log('; '.join(current) if current else 'all clear')

    def _log(self, text):
        """Console and operator at once."""
        self._node.get_logger().info(text)
        self._gui.publish_log(text)


def main(args=None):
    """The commander is the node. Two timers and a spin.

    Single-threaded executor, deliberately: the tick and the subscription
    callbacks never overlap, so the status and telemetry caches need no locking.
    The GUI link is the one thing on another thread, and it is a mailbox behind
    a lock for exactly that reason.

    Parameters are declared from the config file rather than named here. The
    conditions own their thresholds; a commander that knows what a volt is has
    the wrong job.
    """
    rclpy.init(args=args)
    node = rclpy.create_node(
        NODE_NAME, automatically_declare_parameters_from_overrides=True)
    limits = {name: parameter.value
              for name, parameter in node.get_parameters_by_prefix('').items()}

    gui = GuiLink()
    commander = VehicleCommander(node, CommandLink(node), gui, limits)

    node.create_timer(1.0 / TICK_RATE_HZ, commander.tick)
    node.create_timer(1.0 / TELEMETRY_RATE_HZ, commander.publish_telemetry)

    node.get_logger().info(f'{NODE_NAME} up - {TICK_RATE_HZ:.0f} Hz tick')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        gui.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
