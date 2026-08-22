"""Stop whatever is happening. One command, then done.

In the air that is a zero velocity - the vehicle brakes and, once the setpoint
stops being refreshed, ArduPilot's GUID_TIMEOUT leaves it holding position. On
the ground it is LAND.

Deliberately one tick and one command. This is the sequence the commander
starts when the operator hits ABORT, so it runs at the moment something has
already gone wrong; anything it waited for would be another thing to go wrong.
It stops the vehicle and gets out of the way.

It is not the emergency control path. RC is, and it outranks everything here by
taking the vehicle out of GUIDED - which is also why a non-GUIDED mode ends
this immediately rather than trying to command anything. Someone else has the
vehicle and every command would be refused anyway.
"""

from udl_aa_fc import vocabulary
from udl_aa_fc.sequence import Sequence

GUIDED = 'GUIDED'

STOP = {'east': 0.0, 'north': 0.0, 'up': 0.0, 'yaw_rate': 0.0}


class AbortSequence(Sequence):

    def update(self):
        telemetry = self.fs.telemetry()

        if telemetry.state.mode != GUIDED:
            self.log(f'abort: nothing to do - mode is '
                     f'{telemetry.state.mode or "unknown"!r}, not {GUIDED}')
            return self.ABORT

        if telemetry.state.in_air:
            self.log('abort: stopping - zero velocity')
            self.send(vocabulary.SET_VELOCITY, STOP)
        else:
            self.log('abort: on the ground - landing')
            self.send(vocabulary.LAND)

        return self.COMPLETE
