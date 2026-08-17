"""The command vocabulary, transcribed from the flight stack interface.

Kept here rather than spread through the sequences so that a step is checked
against the expected parameter set before it goes on the wire. The vehicle
desk-rejects a wrong parameter set as cond=['MALFORMED'] without consuming a
tick, but a MALFORMED at runtime is a bug in the sequence, not a condition to
handle - catching it locally turns it into an error at construction time.

Bounds mirror config/vehicle_controller.yaml on the vehicle side. They are a
courtesy check only: the vehicle is authoritative and retunes per deployment,
so a local pass is not a guarantee of acceptance.
"""

ARM = 'ARM'
DISARM = 'DISARM'
TAKEOFF = 'TAKEOFF'
LAND = 'LAND'
RTL = 'RTL'
GOTO_GLOBAL = 'GOTO_GLOBAL'
GOTO_LOCAL = 'GOTO_LOCAL'
GOTO_BODY = 'GOTO_BODY'
SET_VELOCITY = 'SET_VELOCITY'
STOP_VELOCITY = 'STOP_VELOCITY'
SET_ACCEL = 'SET_ACCEL'
STOP_ACCEL = 'STOP_ACCEL'
SET_MODE_GUIDED = 'SET_MODE_GUIDED'

# command -> the exact parameter set. Not a subset, not a superset.
PARAMS = {
    ARM: frozenset(),
    DISARM: frozenset(),
    TAKEOFF: frozenset({'altitude'}),
    LAND: frozenset(),
    RTL: frozenset(),
    GOTO_GLOBAL: frozenset({'lon', 'lat', 'alt', 'yaw_deg'}),
    GOTO_LOCAL: frozenset({'east', 'north', 'up', 'yaw_deg'}),
    GOTO_BODY: frozenset({'right', 'forward', 'up', 'yaw_deg'}),
    SET_VELOCITY: frozenset({'east', 'north', 'up', 'yaw_rate'}),
    STOP_VELOCITY: frozenset(),
    SET_ACCEL: frozenset({'east', 'north', 'up'}),
    STOP_ACCEL: frozenset(),
    SET_MODE_GUIDED: frozenset(),
}

# Rejected in every permission-matrix state; GUIDED has to come from a GCS or
# the pilot. Listed so a sequence that tries to self-arbitrate into GUIDED
# fails at construction rather than looping on FC_STATE forever.
UNREACHABLE = frozenset({SET_MODE_GUIDED})


def validate(command, params):
    """Raise if `command` is unknown or `params` is not its exact key set.

    Called on the way out in CommandLink.send, so a wrong parameter set fails
    at the call site with the offending names rather than coming back from the
    vehicle as an anonymous cond=['MALFORMED'].
    """
    expected = PARAMS.get(command)
    if expected is None:
        raise ValueError(f'unknown command {command!r}')

    got = frozenset(params)
    missing = expected - got
    extra = got - expected
    if missing or extra:
        raise ValueError(
            f'{command} takes {sorted(expected) or "no parameters"}; '
            f'missing={sorted(missing)} unexpected={sorted(extra)}')


def check_bounds(command, params):
    """Return a reason string if `params` would fail the vehicle's SANITY axis.

    Returns None when the values look acceptable. Advisory only - see the
    module docstring on why this is not authoritative.

    TODO: implement TAKEOFF altitude, GOTO_* envelopes, SET_VELOCITY and
    SET_ACCEL magnitudes, and the non-finite check that applies to all of them.
    """
    raise NotImplementedError
