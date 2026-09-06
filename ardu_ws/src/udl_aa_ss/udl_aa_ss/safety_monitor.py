#!/usr/bin/env python3
"""
Safety Monitor - the safety stack's answer to the flight commander's gui_link.

gui_link supplies "what should run" from an operator pressing a button; this
supplies it from the vehicle's own state. Everything below it - the commander,
the sequences, the link to the flight stack - is the same shape as udl_aa_fc.

It does not decide what a condition means. Each condition in conditions/ grades
itself against its own tiers and reports the response it wants; this module runs
them on threads, arbitrates between their reports when more than one fires at
once, and looks up the sequence name. It holds no thresholds and knows nothing
about volts, metres or microseconds - all of that lives with the condition that
measures it.

Each condition runs on its own thread at its own rate, decoupled from the
commander's tick, and publishes a frozen Verdict the tick reads without
blocking. The rebind is atomic under the GIL, so a reader never sees a
half-updated verdict - the same single-writer/multi-reader pattern the flight
stack's old safety_monitor used, now one instance per condition rather than one
bundled snapshot.

No rclpy here. Limits arrive as a plain dict from the node, the way
vehicle_controller hands its bounds to FsmHandler, which is what lets every
condition be tested as a pure function with no ROS and no threads.
"""

import dataclasses
import threading
import time
from typing import Callable, Dict, List

from udl_aa_ss import constants
from udl_aa_ss.conditions import battery, geofence, rc_loss, rc_throttle
from udl_aa_ss.constants import CLEAR, Condition, Message, Verdict

# The registry. One row, one thread. Adding a condition is a module in
# conditions/, a row here, and a row in constants.PRIORITY - nothing else in the
# stack changes.
CONDITIONS: Dict[str, Callable] = {
    Condition.BATTERY: battery.evaluate,
    Condition.GEOFENCE: geofence.evaluate,
    Condition.RC_LOSS: rc_loss.evaluate,
    Condition.RC_THROTTLE: rc_throttle.evaluate,
}


class _ConditionThread:
    """
    One condition, one thread. Holds the latest Verdict and nothing else the
    monitor can see.

    The evaluator is called with whatever telemetry is current; it is not fed a
    queue and it never blocks. A slow or wedged evaluator delays only its own
    condition, which is the point of a thread each. `_state` is scratch owned by
    this thread alone, so a stateful condition stays confined to it.
    """

    def __init__(self, name: str, evaluate: Callable, telemetry: Callable,
                 limits: Dict[str, float], period_s: float):
        self.name = name
        self._evaluate = evaluate
        self._telemetry = telemetry
        self._limits = limits
        self._period_s = period_s
        self._state: Dict = {}

        # Single writer (_run), many readers (the commander's tick). A plain
        # rebind of a frozen object is atomic under the GIL, so no lock.
        self.verdict: Verdict = CLEAR

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name=f'condition-{name.lower()}')

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join()

    def _run(self):
        while not self._stop.is_set():
            # Stamped here rather than in the evaluator: a condition reports a
            # fact and should not have to know its own name to do it. replace()
            # keeps the message reference intact, so the Message.NONE identity
            # test downstream still holds.
            self.verdict = dataclasses.replace(
                self._evaluate(self._telemetry(), self._limits, self._state),
                condition=self.name)
            time.sleep(self._period_s)


class SafetyMonitor:
    """
    Runs the condition threads and arbitrates between their verdicts.

    Expected interface, called from safety_commander's tick:
      decide() -> [Verdict, ...]   ordered, index 0 wins
    """

    def __init__(self, telemetry: Callable, limits: Dict[str, float]):
        """
        `telemetry` is a callable returning the latest snapshot - never None,
        seeded with a default-constructed message the way fc's CommandLink does,
        so no condition needs a None branch.

        One rate for every condition: the telemetry feeding them publishes at
        5 Hz, so per-condition rates would buy nothing but a tuning knob that
        cannot matter.
        """
        period_s = 1.0 / limits['condition_rate_hz']
        self._threads = {
            name: _ConditionThread(name, evaluate, telemetry, limits, period_s)
            for name, evaluate in CONDITIONS.items()
        }

    def start(self):
        for thread in self._threads.values():
            thread.start()

    def stop(self):
        for thread in self._threads.values():
            thread.stop()

    def decide(self) -> List[Verdict]:
        """
        One pass over the latched verdicts, returned in order. Pure with respect
        to the threads: it reads, it never writes, so calling it twice in a tick
        is harmless.

        **Index 0 is the winner** - the sequence the commander should run is
        `decide()[0].message.sequence`. Everything after it is information, not
        a queue: it exists so the operator and both stacks can see every active
        condition rather than only the one being acted on. An empty list means
        nothing is active.

        No special case for warnings. They rank lowest in SEVERITY, so a real
        sequence always outranks one - and if only warnings are active, index 0
        carries sequence None and nothing runs.

        Stateless on purpose: this reports what currently wins and has no idea
        what is already running. Whether that means start, ignore or preempt is
        the commander's call.
        """
        return sorted(
            (thread.verdict for thread in self._threads.values()
             if thread.verdict.message is not Message.NONE),
            key=lambda verdict: (-constants.SEVERITY[verdict.message.sequence],
                                 constants.PRIORITY[verdict.condition]),
        )
