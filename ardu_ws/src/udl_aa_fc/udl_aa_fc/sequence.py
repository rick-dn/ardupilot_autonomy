"""Base for every flight sequence: three hooks and two helpers.

A sequence is a phase machine the commander ticks. It owns its phases and
decides from telemetry when to move between them - the vehicle reports only
that a command was dispatched, never that it finished, so arrival, altitude
and settling are all determined here.

The base holds no outcome, no condition list and no kill flag, on purpose.
A rejection is matched by the commander against `token` and ends the sequence
there; completion and failure are the two values `update()` returns. Nothing
has to be stored to report either - the commander logs one line, calls
on_exit(), and drops the sequence.

One command per tick. send() mints a token and publishes; the vehicle holds
exactly one submission and clears it on its next tick, so a subclass issues on
a phase transition and then watches telemetry. Re-issuing every tick overwrites
submissions that are then never arbitrated and never answered.
"""


class Sequence:
    """One flight sequence. Only update() must be implemented.

    `log` is a callable supplied by the commander. Calling self.log(...) puts a
    line on the console and in front of the operator at the same time, which is
    the only reason a sequence needs to hold no status of its own.
    """

    RUNNING = 'RUNNING'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'

    def __init__(self, seq_id, fs, log):
        self.seq_id = seq_id
        self.fs = fs
        self.log = log
        self.cmd_id = 0
        self.token = 0

    # ------------------------------------------------------------------
    # The commander calls these
    # ------------------------------------------------------------------

    def on_start(self):
        """Reset phase state and open whatever the run needs."""

    def update(self):
        """One tick. Returns RUNNING, COMPLETE or ABORT."""
        raise NotImplementedError

    def on_exit(self):
        """Close what on_start opened.

        Runs on every ending, rejection included. A sequence that opened a
        velocity stream must send STOP_VELOCITY here: ArduPilot's GUID_TIMEOUT
        is 3 s, so a stream left open holds the last setpoint for that long
        after the sequence is gone.
        """

    # ------------------------------------------------------------------
    # Subclasses use these
    # ------------------------------------------------------------------

    def send(self, command, params=None):
        """Publish one command under a fresh token.

        The token carries seq_id in its high bits so the commander can tell our
        rejection from another commander's on the shared status topic. cmd_id
        starts at 1 because the vehicle reports token 0 when nothing was
        submitted, which would otherwise match us.
        """
        self.cmd_id += 1
        self.token = (self.seq_id << 16) | self.cmd_id
        self.fs.publish(self.token, command, params)

    def telemetry(self):
        """The latest snapshot, or None before the first one arrives.

        Handed out whole: every field was sampled at the same instant, so a
        subclass can compare fields against each other without reconciling
        timestamps.
        """
        return self.fs.telemetry()
