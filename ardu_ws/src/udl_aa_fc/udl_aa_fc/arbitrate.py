#!/usr/bin/env python3
"""
The arbitration table - which sequence wins when more than one is asked for.

One ranking over every candidate, from conditions and from the operator alike.
Deliberately one and not two: ranking safety first and then ranking the winner
against the operator is two stages that can disagree about what should happen,
and a single table cannot.

Pure, and with no rclpy - it is a dict and a max(), testable by calling it.

What it does not do, on purpose:

  It does not know whether anything is already running. "Deny a new sequence
  while one is live unless it is abort or a more severe safety sequence" is the
  commander's rule, decided against the status topic. This answers only "of the
  things being asked for right now, which one wins".

  It does not rank on who asked. The operator and a condition can both ask for
  smart_rtl, and v1 ranks the two identically - see the TODO below.
"""

from typing import Iterable, Optional, Tuple

from udl_aa_fc.constants import Sequence

# Higher wins. Gaps are intentional: a band inserted later needs no renumbering,
# and the gaps are where "who asked" will eventually break a tie between two
# requests for the same sequence.
#
# abort sits above everything, including every safety sequence. The operator can
# always stop what the vehicle is doing, which keeps the rule to one line and
# costs nothing in safety terms - RC outranks all of this anyway by taking the
# vehicle out of GUIDED.
#
# The operator's own sequences sit at the bottom, which is what makes "safety
# outranks the operator" fall out of the same table rather than needing a
# separate check.
PRIORITY = {
    Sequence.ABORT: 100,

    Sequence.MOTOR_CUTOFF: 60,
    Sequence.EMERGENCY_LAND: 50,
    Sequence.LAND: 40,
    Sequence.RTL: 30,
    Sequence.SMART_RTL: 20,

    Sequence.TAKEOFF: 10,
    Sequence.MANUAL: 10,
}

# An unregistered name ranks here: below everything real, so it can only win
# when it is the sole candidate. It is not dropped, because publishing it makes
# the typo visible as a rejection from the sequence node rather than as silence.
UNKNOWN = 0


def arbitrate(candidates: Iterable[Tuple[Optional[str], object]]) -> Optional[str]:
    """
    Return the winning sequence name, or None if nothing should run.

    `candidates` is (sequence, message) pairs - every sequence asked for this
    tick. The message rides along so a caller can log why something won without
    this module having to know what a message is; v1 does not rank on it.

    A candidate whose sequence is None is a warning: it was raised to be seen,
    not acted on, and is skipped here. If every candidate is a warning the
    answer is None, which is the same answer as no candidates at all - and
    correctly so, since neither should start anything.

    TODO: rank on who asked, not just on what was asked for. An operator
    smart_rtl and a battery smart_rtl are indistinguishable today.
    """
    return max(
        (sequence for sequence, _ in candidates if sequence is not None),
        key=lambda sequence: PRIORITY.get(sequence, UNKNOWN),
        default=None,
    )
