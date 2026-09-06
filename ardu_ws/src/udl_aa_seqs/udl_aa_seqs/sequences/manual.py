"""The operator's d-pad, as a sequence.

A held button is a repeated SET_VELOCITY, not one command. Nothing streams
setpoints on our behalf any more: ArduPilot stops the vehicle if setpoints stop
arriving within GUID_TIMEOUT (3 s), so refreshing is how this says "still
held", and going quiet is how everything else says stop.

That makes the deadman free and unconditional. Release, a crash, a killed node,
a severed link - all of them are silence, and silence stops the vehicle in at
most 3 s without anything on our side having to notice or act. STOP_VELOCITY on
release is only the fast path, worth sending because it stops in ~50 ms instead
of 3 s, and harmless if it never arrives.

It is a sequence rather than a special case so that it inherits what every
sequence gets: the commander's live gate, the rejection kill, and one-at-a-time
exclusion. seq_id is 0, so a manual token is just its command counter, starting
at 1 - token 0 never appears.
"""

import time

from udl_aa_seqs import vocabulary
from udl_aa_seqs.sequence import Sequence

GUIDED = 'GUIDED'

# Comfortably inside ArduPilot's 3 s GUID_TIMEOUT: two consecutive refreshes
# would have to be lost before the vehicle brakes, and at a 20 Hz tick on both
# sides that needs something genuinely stuck rather than ordinary jitter.
REFRESH_S = 1.0

# The exact parameter set SET_VELOCITY expects. Missing axes are zero rather
# than absent: the vehicle desk-rejects a partial set as MALFORMED.
AXES = ('east', 'north', 'up', 'yaw_rate')


class ManualSequence(Sequence):

    def on_start(self, params=None):
        params = params or {}
        self._params = {axis: float(params.get(axis, 0.0)) for axis in AXES}
        self._issued = False
        self._next_send = 0.0

    def update(self):
        """Refresh the setpoint for as long as the button is held."""
        if time.monotonic() < self._next_send:
            return self.RUNNING

        # Re-checked on every refresh, not just the first. A hold that outlives
        # the conditions that allowed it - the pilot takes RC, the vehicle
        # lands - ends here, and the silence that follows stops the vehicle.
        telemetry = self.fs.telemetry()
        checks = (
            (f'mode is {GUIDED}', telemetry.state.mode == GUIDED),
            ('armed', telemetry.state.armed),
            ('in air', telemetry.state.in_air),
        )
        failed = [label for label, passed in checks if not passed]
        if failed:
            self.log(f'manual: refused - {", ".join(failed)}')
            return self.ABORT

        self.send(vocabulary.SET_VELOCITY, self._params)
        self._next_send = time.monotonic() + REFRESH_S

        # Only the first one is worth a line; the rest carry the same numbers.
        if not self._issued:
            self._issued = True
            self.log('manual: ' + ' '.join(
                f'{axis}={self._params[axis]:+.1f}' for axis in AXES))
        return self.RUNNING

    def on_exit(self):
        # Nothing was commanded if the press was refused, and stopping is only
        # ever an optimisation - silence would do it 3 s later regardless.
        if self._issued:
            self.send(vocabulary.STOP_VELOCITY)
