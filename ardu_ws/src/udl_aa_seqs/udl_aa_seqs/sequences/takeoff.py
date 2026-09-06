"""Pre-arm, arm, take off, confirm every step from telemetry.

The same flow as takeoff.py, turned inside out. That version blocks: it sends,
waits for a status, then spins until a predicate holds. This one is ticked, so
every wait becomes a phase that is re-entered until the world moves.

  PREFLIGHT  not in_air, IMU reporting, mode is GUIDED   -> send ARM
  ARMING     wait for state.armed                        -> send TAKEOFF
  CLIMBING   wait for in_air and local_position.up       -> COMPLETE

Nothing here reads command status. ACCEPTED would only say the command reached
MAVROS, never that the vehicle acted, so every confirmation comes from
telemetry - and a rejection ends the sequence from the commander, above this
class, before the next update().

Timeouts are set by how fast the fields update, not by how fast the vehicle
flies. state (armed, mode, in_air) comes from /mavros/state at ~0.79 Hz, so a
transition can take over a second to become visible; local_position runs at
~3.16 Hz. Nothing below is tight enough to trip on sample rate alone.

GUIDED is a precondition, not something to command. SET_MODE_GUIDED sits in the
dispatch table but in no permitted state, so the stack cannot put itself into
GUIDED - a GCS or the pilot must. Hence the check rather than a command.
"""

import time

from udl_aa_seqs import vocabulary
from udl_aa_seqs.sequence import Sequence

GUIDED = 'GUIDED'


def imu_is_live(snapshot):
    """Heuristic: the IMU is reporting.

    There is no validity flag on the IMU group and every field defaults to
    zero, so an absent IMU and a perfectly still one look identical. Gravity is
    what separates them - a reporting accelerometer always shows about 1 g
    total, a silent one shows exactly nothing.
    """
    imu = snapshot.imu
    magnitude = (imu.accel_right ** 2
                 + imu.accel_forward ** 2
                 + imu.accel_up ** 2) ** 0.5
    return magnitude > 1.0


class TakeoffSequence(Sequence):

    PREFLIGHT = 'PREFLIGHT'
    ARMING = 'ARMING'
    CLIMBING = 'CLIMBING'

    def __init__(self, seq_id, fs, log, altitude=5.0, tolerance_m=0.5,
                 telemetry_timeout_s=10.0, arm_timeout_s=10.0,
                 climb_timeout_s=60.0):
        super().__init__(seq_id, fs, log)
        self.altitude = altitude
        self.tolerance = tolerance_m
        self.telemetry_timeout = telemetry_timeout_s
        self.arm_timeout = arm_timeout_s
        self.climb_timeout = climb_timeout_s

        self._phase = self.PREFLIGHT
        self._deadline = 0.0

    def on_start(self, params=None):
        self._enter(self.PREFLIGHT, self.telemetry_timeout)
        self.log(f'takeoff: target {self.altitude:.1f} m')

    def update(self):
        telemetry = self.fs.telemetry()
        if telemetry is None:
            # No snapshot is not a failed check - there is nothing to check
            # yet. Bounded anyway, since it also means the vehicle stack is
            # down and no phase below would ever advance.
            if self._expired():
                self.log('takeoff: no telemetry - is the vehicle stack up?')
                return self.ABORT
            return self.RUNNING

        if self._phase == self.PREFLIGHT:
            return self._preflight(telemetry)
        if self._phase == self.ARMING:
            return self._arming(telemetry)
        return self._climbing(telemetry)

    # ------------------------------------------------------------------
    # Phases
    # ------------------------------------------------------------------

    def _preflight(self, telemetry):
        """Read straight off one snapshot, so the checks cannot disagree."""
        checks = (
            ('not in air', not telemetry.state.in_air),
            ('imu reporting', imu_is_live(telemetry)),
            (f'mode is {GUIDED}', telemetry.state.mode == GUIDED),
        )
        failed = [label for label, passed in checks if not passed]
        if failed:
            self.log(f'takeoff: pre-arm failed - {", ".join(failed)} '
                     f'(mode={telemetry.state.mode!r} '
                     f'armed={telemetry.state.armed} '
                     f'in_air={telemetry.state.in_air})')
            return self.ABORT

        self.log('takeoff: pre-arm clear, arming')
        self.send(vocabulary.ARM)
        self._enter(self.ARMING, self.arm_timeout)
        return self.RUNNING

    def _arming(self, telemetry):
        if telemetry.state.armed:
            self.log(f'takeoff: armed, climbing to {self.altitude:.1f} m')
            self.send(vocabulary.TAKEOFF, {'altitude': self.altitude})
            self._enter(self.CLIMBING, self.climb_timeout)
            return self.RUNNING

        if self._expired():
            # Accepted but never armed means ArduPilot refused on its own
            # pre-arm checks, and it says why only in statustext - cond is
            # clean, because the stack's own gates all passed.
            self.log(f'takeoff: not armed within {self.arm_timeout:.0f}s - '
                     f'last statustext: {telemetry.statustext.text!r}')
            return self.ABORT
        return self.RUNNING

    def _climbing(self, telemetry):
        target = self.altitude - self.tolerance
        up = telemetry.local_position.up

        # in_air latches on the vehicle above 0.5 m and is the same value the
        # permission gate uses, so anything flown after this is already
        # permitted by the time the altitude check passes.
        if telemetry.state.in_air and up >= target:
            self.log(f'takeoff: complete at {up:.2f} m')
            return self.COMPLETE

        if self._expired():
            self.log(f'takeoff: {up:.2f} m after {self.climb_timeout:.0f}s, '
                     f'target was {target:.1f} m')
            return self.ABORT
        return self.RUNNING

    # ------------------------------------------------------------------
    # Deadlines
    # ------------------------------------------------------------------

    def _enter(self, phase, timeout_s):
        self._phase = phase
        self._deadline = time.monotonic() + timeout_s

    def _expired(self):
        return time.monotonic() >= self._deadline
