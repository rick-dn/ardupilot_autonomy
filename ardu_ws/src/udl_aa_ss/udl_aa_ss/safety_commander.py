#!/usr/bin/env python3
"""Runs one safety sequence at a time and decides when it stops.

The same shape as udl_aa_fc's vehicle_commander, with one substitution: where
that polls gui_link for an operator's button press, this asks safety_monitor
what the vehicle's own state is calling for. It owns the lifecycle; sequences
own their phases. It never inspects a sequence's internals - a sequence reports
only RUNNING, COMPLETE or ABORT.

Two deliberate differences from the flight commander, both consequences of the
trigger being a standing condition rather than a one-shot press:

  Preempt, not refuse. VehicleCommander turns down a start while something runs.
  Here a more severe condition takes the vehicle from a less severe one - that
  is the whole point of a severity ladder.

  Severity, not equality, decides. The monitor re-derives its answer every tick,
  so a condition that flickers on the edge of its threshold would start and stop
  a sequence repeatedly if the commander simply followed index 0. Instead a
  running sequence is displaced only by something strictly more severe, and
  otherwise ends on its own COMPLETE or ABORT. A condition clearing does not
  abandon the response to it - a battery that sagged under load and recovered
  when the vehicle slowed has not stopped being a reason to come home.

Rejection is the one signal that arrives from outside the tick. fs_link latches
it in the status callback and hands it over here, because the vehicle echoes a
token on exactly the one tick that arbitrated it - sampling the status topic
from this tick would miss it and the sequence would run on believing a command
was flying.
"""

import rclpy

from udl_aa_ss import constants
from udl_aa_ss.constants import Sequence
from udl_aa_ss.fs_link import FsLink
from udl_aa_ss.safety_monitor import SafetyMonitor
from udl_aa_ss.sequences.smart_rtl import SmartRtlSequence

NODE_NAME = 'udl_aa_ss'

# Matches the vehicle's own rate: it arbitrates one submission per 20 Hz tick,
# so there is nothing to gain by deciding faster than it acts.
TICK_RATE_HZ = 20.0

# name -> (seq_id, class). The name is what constants.Message points at, so a
# message and its sequence are wired together through this table and nowhere
# else. seq_id lands in the high bits of every token the sequence sends, which
# is what lets a rejection on the status topic be attributed to one sequence
# rather than another. Ids are fixed and never reused.
SEQUENCES = {
    Sequence.SMART_RTL: (1, SmartRtlSequence),
    # Sequence.RTL:            (2, RtlSequence),
    # Sequence.LAND:           (3, LandSequence),
    # Sequence.EMERGENCY_LAND: (4, EmergencyLandSequence),
    # Sequence.MOTOR_CUTOFF:   (5, MotorCutoffSequence),
}

# Sanity bounds and rates - real ROS parameters, see config/safety_stack.yaml.
# Injected into safety_monitor as a plain dict, since it has no rclpy Node of
# its own and no condition should have to reach for one.
LIMITS = {
    'condition_rate_hz': 10.0,
    'batt_invalid_v': 1.0,
    'batt_critical_v': 10.0,
    'batt_very_low_v': 10.5,
    'batt_low_v': 11.1,
}


class SafetyCommander:

    def __init__(self, node, fs, monitor):
        self._node = node
        self._fs = fs
        self._monitor = monitor

        # Built once and reused. on_start() is what resets a sequence for its
        # next run, so a second smart_rtl is the same object, not a new one.
        self._registry = {
            name: cls(seq_id, fs, self._log)
            for name, (seq_id, cls) in SEQUENCES.items()
        }

        self._active = None
        self._active_name = None
        self._reported = ()

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def tick(self):
        """One pass. Never blocks and never waits on a command."""
        verdicts = self._monitor.decide()
        self._report(verdicts)

        # Checked before anything is started. Both limbs are reads on the link,
        # so this costs nothing and every sequence gets it for free.
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

        # Index 0 is the winner; everything after it is information. A winner
        # carrying no sequence is a warning - it was reported above and runs
        # nothing.
        wanted = verdicts[0].message.sequence if verdicts else None
        self._arbitrate(wanted)

        if self._active is not None:
            status = self._active.update()
            if status == self._active.COMPLETE:
                self._stop('complete')
            elif status == self._active.ABORT:
                self._stop('failed')

    def _arbitrate(self, wanted):
        """Start, preempt, or leave alone. Never stops on a condition clearing.

        Severity rather than equality, so a condition flickering across its
        threshold cannot restart a sequence, and a less severe condition cannot
        take the vehicle from a more severe one already handling it.
        """
        if wanted is None:
            return

        if self._active is None:
            self._start(wanted)
            return

        severity = constants.SEVERITY
        if severity[wanted] > severity[self._active_name]:
            self._stop(f'preempted by {wanted}')
            self._start(wanted)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _start(self, name):
        sequence = self._registry.get(name)
        if sequence is None:
            # Throttled - a standing condition would otherwise repeat this at
            # the full tick rate.
            self._node.get_logger().error(
                f'no sequence registered for {name!r}', throttle_duration_sec=1.0)
            return

        self._active = sequence
        self._active_name = name
        sequence.on_start()
        self._log(f'{name} started')

    def _stop(self, reason):
        """The only way a sequence ends. Logs, unwinds, forgets.

        The forgetting is in a finally because an on_exit() that raises would
        otherwise leave the commander believing a dead sequence is still
        running - nothing new could start and no condition could be answered,
        until the node was restarted.
        """
        if self._active is None:
            return
        self._log(f'{self._active_name} {reason}')
        try:
            self._active.on_exit()
        finally:
            self._active = None
            self._active_name = None

    # ------------------------------------------------------------------
    # Reporting
    # ------------------------------------------------------------------

    def _report(self, verdicts):
        """Log every active condition, once per change rather than per tick.

        All of them, not just the winner: the rest are why the operator can see
        what else is wrong while one thing is being acted on.
        """
        current = tuple((v.condition, v.message.text) for v in verdicts)
        if current == self._reported:
            return
        self._reported = current

        if not current:
            self._log('all clear')
            return
        self._log('; '.join(f'{name}: {text}' for name, text in current))

    def _log(self, text):
        self._node.get_logger().info(text)


def main(args=None):
    """The safety stack is the node. One timer and a spin.

    Single-threaded executor, deliberately: the tick and the subscription
    callbacks never overlap, so the rejection latch and the telemetry cache need
    no locking. The condition threads are the one thing on other threads, and
    each hands over a single frozen object for exactly that reason.
    """
    rclpy.init(args=args)
    node = rclpy.create_node(NODE_NAME)

    for name, default in LIMITS.items():
        node.declare_parameter(name, default)
    limits = {name: node.get_parameter(name).value for name in LIMITS}

    fs = FsLink(node)
    monitor = SafetyMonitor(fs.telemetry, limits)
    commander = SafetyCommander(node, fs, monitor)

    monitor.start()
    node.create_timer(1.0 / TICK_RATE_HZ, commander.tick)

    node.get_logger().info(f'{NODE_NAME} up - {TICK_RATE_HZ:.0f} Hz tick')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # plain print, not get_logger() - rclpy's SIGINT handler has already
        # torn the context down by now, so a /rosout publish would fail.
        print('Shutting down safety stack')
    finally:
        # unconditional - stops the condition threads. Safe on a dead context;
        # only shutdown() needs the rclpy.ok() guard.
        monitor.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
