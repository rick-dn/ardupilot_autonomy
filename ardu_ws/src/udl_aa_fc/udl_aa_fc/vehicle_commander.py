"""Runs one sequence at a time and decides when it stops.

The commander owns the lifecycle; sequences own their phases. It starts what
the operator asks for, ends what the vehicle refuses, and logs one line at
every ending. It never inspects a sequence's internals - a sequence reports
only RUNNING, COMPLETE or ABORT, and everything else the operator sees comes
from the log lines the sequence emits as it runs.

Rejection is the one signal that arrives from outside the tick. fs_link latches
it in the status callback and hands it over here, because the vehicle echoes a
token on exactly the one tick that arbitrated it - sampling the status topic
from this tick would miss it and the sequence would run on believing a command
was flying.

Telemetry to the operator runs on its own timer, not this tick. The source is
5 Hz; publishing at tick rate would emit four byte-identical messages per new
datum.
"""

import rclpy

from udl_aa_fc import gui_link
from udl_aa_fc.command_link import CommandLink
from udl_aa_fc.gui_link import GuiLink
from udl_aa_fc.sequences.abort import AbortSequence
from udl_aa_fc.sequences.manual import ManualSequence
from udl_aa_fc.sequences.smart_rtl import SmartRtlSequence
from udl_aa_fc.sequences.takeoff import TakeoffSequence

NODE_NAME = 'udl_aa_fc'

# The tick matches the vehicle's own rate: it arbitrates one submission per
# 20 Hz tick, so there is nothing to gain by deciding faster than it acts.
TICK_RATE_HZ = 20.0

# Telemetry to the operator runs slower and on its own timer. The source is
# 5 Hz; publishing at tick rate would emit four byte-identical messages per
# new datum.
TELEMETRY_RATE_HZ = 5.0

ABORT_SEQUENCE = 'abort'

# name -> (seq_id, class). seq_id lands in the high bits of every token the
# sequence sends, which is what lets a rejection on the shared status topic be
# attributed to us rather than to another commander. Ids are fixed and never
# reused; the wire names must match the page's data-sequence attributes.
SEQUENCES = {
    # seq_id 0: a manual token is just its command counter, which starts at 1,
    # so token 0 still never appears on the wire.
    'manual': (0, ManualSequence),
    'takeoff': (1, TakeoffSequence),
    'smart_rtl': (2, SmartRtlSequence),
    # 'smart_land':    (3, SmartLandSequence),
    # 'follow':        (4, FollowSequence),
    # 'aruco_landing': (5, ArucoLandingSequence),
    ABORT_SEQUENCE: (9, AbortSequence),
}


class VehicleCommander:

    def __init__(self, node, fs, gui):
        self._node = node
        self._fs = fs
        self._gui = gui

        # Built once and reused. on_start() is what resets a sequence for its
        # next run, so a second takeoff is the same object, not a new one.
        self._registry = {
            name: cls(seq_id, fs, self._log)
            for name, (seq_id, cls) in SEQUENCES.items()
        }

        self._active = None
        self._active_name = None

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def tick(self):
        """One pass. Never blocks and never waits on a command."""
        request = self._gui.poll()
        if request is not None:
            if request.action == gui_link.START:
                self._start(request.name, request.params)
            elif request.action == gui_link.STOP:
                self._stop('released by operator')
            else:
                self._stop('aborted by operator')
                self._start(ABORT_SEQUENCE)

        # Checked here rather than in any sequence. Both limbs are reads on
        # the link, so this costs nothing and every sequence gets it for free.
        if not self._fs.live:
            self._stop('vehicle link down')
            return

        # Ordered before update() on purpose: a rejected command was never
        # dispatched, so letting the sequence tick once more would have it
        # waiting on telemetry that is never going to move.
        rejection = self._fs.rejection()
        if rejection is not None and self._active is not None:
            if rejection.token == self._active.token:
                self._stop('rejected - ' + ', '.join(rejection.cond))

        if self._active is not None:
            status = self._active.update()
            if status == self._active.COMPLETE:
                self._stop('complete')
            elif status == self._active.ABORT:
                self._stop('failed')

    def publish_telemetry(self):
        """Operator-facing snapshot. Own timer, matched to the source rate."""
        telemetry = self._fs.telemetry()
        if telemetry is None:
            return

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
            'mission': self._active_name,
        })

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _start(self, name, params=None):
        # One sequence at a time. A start arriving while something runs is
        # refused, not queued and not honoured - two things steering the same
        # vehicle is never what the operator meant. Abort is the only override,
        # and it goes through _stop first so this never sees a live sequence.
        #
        # This is a GCS, not the emergency control path. That is RC, which
        # outranks everything here by taking the vehicle out of GUIDED.
        if self._active is not None:
            self._log(f'{name} refused - {self._active_name} is running')
            return

        sequence = self._registry.get(name)
        if sequence is None:
            self._log(f'unknown sequence: {name}')
            return

        self._active = sequence
        self._active_name = name
        sequence.on_start(params)
        self._log(f'{name} started')

    def _stop(self, reason):
        """The only way a sequence ends. Logs, unwinds, forgets.

        The forgetting is in a finally because an on_exit() that raises would
        otherwise leave the commander believing a dead sequence is still
        running - nothing new could start and the operator's abort would do
        nothing, until the node was restarted.
        """
        if self._active is None:
            return
        self._log(f'{self._active_name} {reason}')
        try:
            self._active.on_exit()
        finally:
            self._active = None
            self._active_name = None

    def _log(self, text):
        """Sequence and commander lines, to the console and the operator."""
        self._node.get_logger().info(text)
        self._gui.publish_log(text)


def main(args=None):
    """The flight commander is the node. Two timers and a spin.

    Single-threaded executor, deliberately: the tick and the subscription
    callbacks never overlap, so the rejection latch and the telemetry cache
    need no locking. The GUI link is the one thing on another thread, and it
    is a mailbox behind a lock for exactly that reason.
    """
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME)

    gui = GuiLink()
    commander = VehicleCommander(node, CommandLink(node), gui)

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
