"""Square up, then return, and stay running until the vehicle is down.

  PREFLIGHT  GUIDED, armed, in air        -> hold position, command yaw 0
  ALIGNING   wait for yaw to come round   -> RTL
  RETURNING  wait for on the ground and disarmed

RETURNING is why this does not end at the RTL command. RTL is not an instant -
it is a climb, a transit, a descent and an auto-disarm, and a sequence that
reported COMPLETE the moment the command went out would free the commander to
start something else into a vehicle that is still flying itself home. The
sequence owns the vehicle until ArduPilot has actually put it down.

Nothing is commanded during RETURNING, and nothing could be: RTL takes the
vehicle out of GUIDED, so every command in the vocabulary is refused from that
point on. This only watches.

The yaw is squared to 0 before handing over rather than after, for the same
reason - nothing here can steer once RTL lands. Aligning first means the return
flies with a known heading instead of whatever the last leg left behind.

Yaw 0 is due East: the stack is ENU throughout and yaw is CCW positive from
East, so this is a local-frame heading and not a compass bearing.

Holding position while turning is one GOTO_LOCAL at the current east/north/up
with yaw_deg 0. There is no turn-in-place command in the vocabulary, and a
position target the vehicle is already at is exactly that.
"""

import time

from udl_aa_seqs import vocabulary
from udl_aa_seqs.sequence import Sequence

GUIDED = 'GUIDED'


def wrap_deg(degrees):
    """Fold an angle onto -180..180, so 359 reads as -1 rather than far away."""
    return (degrees + 180.0) % 360.0 - 180.0


class SmartRtlSequence(Sequence):

    PREFLIGHT = 'PREFLIGHT'
    ALIGNING = 'ALIGNING'
    RETURNING = 'RETURNING'

    def __init__(self, seq_id, fs, log, yaw_tolerance_deg=5.0,
                 align_timeout_s=30.0, return_timeout_s=180.0):
        super().__init__(seq_id, fs, log)
        self.yaw_tolerance = yaw_tolerance_deg
        self.align_timeout = align_timeout_s
        self.return_timeout = return_timeout_s

        self._phase = self.PREFLIGHT
        self._deadline = 0.0

    def on_start(self, params=None):
        self._phase = self.PREFLIGHT

    def update(self):
        telemetry = self.fs.telemetry()
        if self._phase == self.PREFLIGHT:
            return self._preflight(telemetry)
        if self._phase == self.ALIGNING:
            return self._aligning(telemetry)
        return self._returning(telemetry)

    def _preflight(self, telemetry):
        checks = (
            (f'mode is {GUIDED}', telemetry.state.mode == GUIDED),
            ('armed', telemetry.state.armed),
            ('in air', telemetry.state.in_air),
        )
        failed = [label for label, passed in checks if not passed]
        if failed:
            self.log(f'smart_rtl: refused - {", ".join(failed)}')
            return self.ABORT

        position = telemetry.local_position
        self.log(f'smart_rtl: yaw {wrap_deg(position.yaw_deg):+.1f} deg, '
                 f'squaring to 0 before return')
        self.send(vocabulary.GOTO_LOCAL, {
            'east': position.east,
            'north': position.north,
            'up': position.up,
            'yaw_deg': 0.0,
        })
        self._phase = self.ALIGNING
        self._deadline = time.monotonic() + self.align_timeout
        return self.RUNNING

    def _aligning(self, telemetry):
        error = wrap_deg(telemetry.local_position.yaw_deg)

        if abs(error) <= self.yaw_tolerance:
            self.log(f'smart_rtl: aligned at {error:+.1f} deg, returning')
            self.send(vocabulary.RTL)
            self._phase = self.RETURNING
            self._deadline = time.monotonic() + self.return_timeout
            return self.RUNNING

        if time.monotonic() >= self._deadline:
            self.log(f'smart_rtl: yaw still {error:+.1f} deg after '
                     f'{self.align_timeout:.0f}s, not returning')
            return self.ABORT
        return self.RUNNING

    def _returning(self, telemetry):
        """Watch only. ArduPilot has the vehicle until it is down."""
        state = telemetry.state

        # ArduPilot disarms itself once RTL has landed, so both together are
        # the end of the return - armed alone would clear on a mid-air disarm,
        # and in_air alone clears at 0.2 m with the rotors still turning.
        if not state.in_air and not state.armed:
            self.log('smart_rtl: down and disarmed')
            return self.COMPLETE

        if time.monotonic() >= self._deadline:
            self.log(f'smart_rtl: still returning after '
                     f'{self.return_timeout:.0f}s, giving up the watch')
            return self.ABORT
        return self.RUNNING
