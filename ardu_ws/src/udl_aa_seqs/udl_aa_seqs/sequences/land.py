"""Hand over to the autopilot's LAND, then watch until the vehicle is down.

  HANDOVER    GUIDED, armed, in air   -> LAND
  DESCENDING  wait for on the ground and disarmed

DESCENDING is why this does not end at the LAND command. Landing is not an
instant - it is a descent, a touchdown and an auto-disarm - and a sequence that
reported COMPLETE the moment the command went out would free the runner to start
something else into a vehicle that is still coming down. It owns the vehicle
until ArduPilot has actually put it on the ground.

Nothing is commanded during DESCENDING, and nothing could be: LAND takes the
vehicle out of GUIDED, so every command in the vocabulary is refused from that
point on. This only watches.

Requested for the most severe conditions - a critically low battery - so there
is no alignment, no positioning and no preamble. It descends where it is.
Getting down is the whole response; anything done first is time the battery does
not have.
"""

import time

from udl_aa_seqs import vocabulary
from udl_aa_seqs.sequence import Sequence

GUIDED = 'GUIDED'


class LandSequence(Sequence):

    HANDOVER = 'HANDOVER'
    DESCENDING = 'DESCENDING'

    def __init__(self, seq_id, fs, log, land_timeout_s=120.0):
        super().__init__(seq_id, fs, log)
        self.land_timeout = land_timeout_s

        self._phase = self.HANDOVER
        self._deadline = 0.0

    def on_start(self, params=None):
        self._phase = self.HANDOVER

    def update(self):
        telemetry = self.fs.telemetry()
        if self._phase == self.HANDOVER:
            return self._handover(telemetry)
        return self._descending(telemetry)

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
            self.log(f'land: refused - {", ".join(failed)}')
            return self.ABORT

        self.log(f'land: descending from {telemetry.local_position.up:.1f} m')
        self.send(vocabulary.LAND)
        self._phase = self.DESCENDING
        self._deadline = time.monotonic() + self.land_timeout
        return self.RUNNING

    def _descending(self, telemetry):
        """Watch only. ArduPilot has the vehicle until it is down."""
        state = telemetry.state

        # ArduPilot disarms itself once it has landed, so both together are the
        # end of the descent - armed alone would clear on a mid-air disarm, and
        # in_air alone clears at 0.2 m with the rotors still turning.
        if not state.in_air and not state.armed:
            self.log('land: down and disarmed')
            return self.COMPLETE

        if time.monotonic() >= self._deadline:
            self.log(f'land: still descending after '
                     f'{self.land_timeout:.0f}s, giving up the watch')
            return self.ABORT
        return self.RUNNING
