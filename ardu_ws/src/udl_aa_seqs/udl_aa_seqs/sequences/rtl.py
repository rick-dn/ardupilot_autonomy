"""Hand over to the autopilot's RTL, then watch until the vehicle is down.

  HANDOVER   GUIDED, armed, in air   -> RTL
  RETURNING  wait for on the ground and disarmed

RETURNING is why this does not end at the RTL command. RTL is not an instant -
it is a climb, a transit, a descent and an auto-disarm - and a sequence that
reported COMPLETE the moment the command went out would free the runner to start
something else into a vehicle that is still flying itself home. It owns the
vehicle until ArduPilot has actually put it down.

Nothing is commanded during RETURNING, and nothing could be: RTL takes the
vehicle out of GUIDED, so every command in the vocabulary is refused from that
point on. This only watches.

The difference from smart_rtl is the yaw. That one squares the heading to 0
before handing over, so the return flies with a known attitude; this goes
immediately. Use it where the extra half-minute of aligning is not worth having
- a breached fence or a lost link, where the point is to leave now.
"""

import time

from udl_aa_seqs import vocabulary
from udl_aa_seqs.sequence import Sequence

GUIDED = 'GUIDED'


class RtlSequence(Sequence):

    HANDOVER = 'HANDOVER'
    RETURNING = 'RETURNING'

    def __init__(self, seq_id, fs, log, return_timeout_s=180.0):
        super().__init__(seq_id, fs, log)
        self.return_timeout = return_timeout_s

        self._phase = self.HANDOVER
        self._deadline = 0.0

    def on_start(self, params=None):
        self._phase = self.HANDOVER

    def update(self):
        telemetry = self.fs.telemetry()
        if self._phase == self.HANDOVER:
            return self._handover(telemetry)
        return self._returning(telemetry)

    def _handover(self, telemetry):
        checks = (
            (f'mode is {GUIDED}', telemetry.state.mode == GUIDED),
            ('armed', telemetry.state.armed),
            ('in air', telemetry.state.in_air),
        )
        failed = [label for label, passed in checks if not passed]
        if failed:
            # Already on the ground, or someone else has the vehicle. Either
            # way there is nothing to do and nothing this could command.
            self.log(f'rtl: refused - {", ".join(failed)}')
            return self.ABORT

        self.log('rtl: returning')
        self.send(vocabulary.RTL)
        self._phase = self.RETURNING
        self._deadline = time.monotonic() + self.return_timeout
        return self.RUNNING

    def _returning(self, telemetry):
        """Watch only. ArduPilot has the vehicle until it is down."""
        state = telemetry.state

        # ArduPilot disarms itself once RTL has landed, so both together are
        # the end of the return - armed alone would clear on a mid-air disarm,
        # and in_air alone clears at 0.2 m with the rotors still turning.
        if not state.in_air and not state.armed:
            self.log('rtl: down and disarmed')
            return self.COMPLETE

        if time.monotonic() >= self._deadline:
            self.log(f'rtl: still returning after '
                     f'{self.return_timeout:.0f}s, giving up the watch')
            return self.ABORT
        return self.RUNNING
