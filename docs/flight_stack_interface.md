# Flight stack interface

Everything an external flight commander needs to drive this vehicle over ROS 2
topics, including how to build long multi-step sequences on top of a stack that
is deliberately stateless.

> **Topic names are provisional.** `/fs_adapter/*` and `/vehicle/telemetry` are
> scheduled to move under a single `/adl/...` root during refactoring. The
> message types and semantics below are stable; only the names will change.

---

## 1. The three topics

| Topic | Type | Direction | QoS |
|---|---|---|---|
| `/fs_adapter/command` | `VehicleCommand` | commander → vehicle | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/fs_adapter/status` | `VehicleStatus` | vehicle → commander | BEST_EFFORT, VOLATILE, KEEP_LAST(1) |
| `/vehicle/telemetry` | `VehicleTelemetry` | vehicle → commander | BEST_EFFORT, VOLATILE, KEEP_LAST(1) |

**QoS must be compatible or DDS silently connects nothing.** A RELIABLE
subscriber will not match a BEST_EFFORT publisher. Subscribe to `status` and
`telemetry` as BEST_EFFORT; publish `command` as RELIABLE.

Commands are RELIABLE because dropping one loses an operator intent. Status and
telemetry are latest-value-wins state broadcasts — a late subscriber wants the
current state, never a backlog.

---

## 2. Message shapes

### `VehicleCommand` (you publish)

```
std_msgs/Header header
string    sequence       # your sequence or program name; identity only
uint32    token          # identifies this submission; echoed back verbatim
string    command        # from the vocabulary in section 3
string[]  param_names    # must be exactly the expected set
float64[] param_values   # same length as param_names
```

### `VehicleStatus` (you subscribe)

```
std_msgs/Header header
string   cmd_status   # ACCEPTED | REJECTED | IDLE
string[] cond         # every condition with something to say; empty when clean
string   command      # the command actually dispatched, '' if none
uint32   token        # echoed from your VehicleCommand; 0 if nothing submitted
```

Constants are available on the type: `VehicleStatus.ACCEPTED`, `.REJECTED`, `.IDLE`.

**`cmd_status` and `command` are independent.** `cmd_status` describes what
became of *your* submission; `command` describes what the vehicle actually
dispatched. Read them together:

| `cmd_status` | `command` | Meaning |
|---|---|---|
| `ACCEPTED` | your command | Dispatched. |
| `REJECTED` | `''` | Blocked by a gate. `cond` says which. |
| `REJECTED` | *a different command* | Valid, but safety outranked you. `command` is safety's. |
| `IDLE` | `''` | Nothing submitted, nothing to do. |
| `IDLE` | *a command* | Nothing submitted; safety acted on its own. |

### `VehicleTelemetry` (you subscribe)

One atomic snapshot, published at `telemetry_rate_hz` (default 5 Hz). Groups:
`state`, `home`, `global_position`, `local_position`, `velocity_body`,
`velocity_body_odom`, `velocity_gps`, `imu`, `battery`, `gps_status`,
`heading_deg`, `vfr_hud`, `rc`, `statustext`.

`state` carries the three fields the permission matrix in section 4 is keyed on:

```
bool   armed    # raw, from /mavros/state
string mode     # raw ArduPilot mode string: GUIDED, STABILIZE, RTL, ...
bool   in_air   # DERIVED on the vehicle - see below
```

**Use `state.in_air` directly; do not re-derive it.** It is not an FC telemetry
field — the vehicle latches it from the local ENU altitude with hysteresis (true
above 0.5 m, false below 0.2 m, hold in between) and publishes the result.
Computing your own from `local_position.up` duplicates vehicle logic that can
then disagree with the authoritative copy at the latch boundary, which is
exactly where the `COMMAND_STATE` gate flips.

**Conventions throughout:** ENU (east/north/up) for world frame, RFU
(right/forward/up) for body frame, yaw **CCW positive from East**. MAVROS' FLU
axes and CW-from-North bearings are converted on the way in — you never see
either. `battery.percentage` is **0.0–1.0**, not 0–100.

---

## 3. Command vocabulary

`param_names` must match these sets **exactly** — no missing, no extra, no
duplicates, no misspellings. All values are `float64`.

| `command` | `param_names` |
|---|---|
| `ARM` | *(none)* |
| `DISARM` | *(none)* |
| `TAKEOFF` | `altitude` |
| `LAND` | *(none)* |
| `RTL` | *(none)* |
| `GOTO_GLOBAL` | `lon`, `lat`, `alt`, `yaw_deg` |
| `GOTO_LOCAL` | `east`, `north`, `up`, `yaw_deg` |
| `GOTO_BODY` | `right`, `forward`, `up`, `yaw_deg` |
| `SET_VELOCITY` | `east`, `north`, `up`, `yaw_rate` |
| `STOP_VELOCITY` | *(none)* |
| `SET_ACCEL` | `east`, `north`, `up` |
| `STOP_ACCEL` | *(none)* |
| `SET_MODE_GUIDED` | *(none)* — **see the warning in section 5** |

`GOTO_GLOBAL.alt` and `GOTO_LOCAL.up` are **relative to home**, not AMSL.
`GOTO_BODY` is an offset from the current pose.

---

## 4. What gets you rejected

Four gates plus a boundary check, evaluated in order. The first failure stops
the chain, so `cond` names the gate that stopped you.

| `cond` entry | Gate | Meaning |
|---|---|---|
| `MALFORMED` | boundary | The message itself is unusable — see section 6. |
| `FC_STATE` | axis 1 | Flight mode is not GUIDED. |
| `RC_THROTTLE_STATUS`, `GEOFENCE`, `RC_LOSS`, `BATTERY` | axis 2 | A safety condition is active. |
| `COMMAND_STATE` | axis 3 | Command not permitted in the current armed/in-air state. |
| `SANITY` | axis 4 | A parameter is out of bounds or non-finite. |

`cond` is a **list** — several safety conditions can be named at once, and a
`WARNING`-tier safety entry appears in `cond` without blocking anything.

### Axis 3 — the permission matrix

Only consulted once mode is already GUIDED.

| State | Permitted |
|---|---|
| disarmed, on ground | `ARM` |
| armed, on ground | `TAKEOFF`, `DISARM` |
| armed, in air | `GOTO_GLOBAL`, `GOTO_LOCAL`, `GOTO_BODY`, `SET_VELOCITY`, `STOP_VELOCITY`, `SET_ACCEL`, `STOP_ACCEL`, `LAND`, `RTL` |
| disarmed, in air | **nothing** |

`in_air` is derived on the vehicle from local altitude with hysteresis: it
latches true above 0.5 m and false below 0.2 m. You cannot set it, but you can
read it — it is published as `telemetry.state.in_air`, and that is the same
value this gate is evaluated against.

Note the false branch is altitude-only, so a mid-air disarm leaves `in_air`
true. That is what makes `disarmed, in air` reachable, and in that state
nothing at all is accepted.

### Axis 4 — sanity bounds

From `config/vehicle_controller.yaml`; retune per deployment.

| Command | Bound |
|---|---|
| `TAKEOFF` | `0.5 <= altitude <= 30.0` |
| `GOTO_GLOBAL` | `-90 <= lat <= 90`, `-180 <= lon <= 180`, `0.0 <= alt <= 50.0` |
| `GOTO_LOCAL` | `0.0 <= up <= 50.0` |
| `GOTO_BODY` | `sqrt(right² + forward² + up²) <= 20.0` |
| `SET_VELOCITY` | `sqrt(east² + north² + up²) <= 5.0` |
| `SET_ACCEL` | `sqrt(east² + north² + up²) <= 3.0` |

Any non-finite value (NaN, inf) in any parameter fails this gate.

---

## 5. Protocol rules — read these before writing a sequence

**One pending command at a time, consumed on read.** The vehicle holds exactly
one submission. The tick takes it, arbitrates it once, and clears it. Two
consequences:

- **Publishing faster than the tick loses commands.** The tick runs at
  `tick_rate_hz` (default 20 Hz). Publish two commands inside one 50 ms window
  and the second overwrites the first, which is never arbitrated and never
  answered. The RELIABLE depth-10 queue is DDS transport only — it does not
  queue submissions on the vehicle. **Send one command, wait for its status,
  then send the next.**
- **Nothing is retried and nothing is remembered.** There is no latch. A
  rejection is final for that submission; if you want it again, resubmit with a
  new token. Equally, a rejection does not poison anything — the same command
  can succeed on the very next tick if the state changed.

**Status is a heartbeat, not a notification.** It publishes every tick
regardless — 20 Hz by default — including ticks where nothing was submitted.
Absence of status means the vehicle stack is down, not that nothing happened.

**Nothing tells you a command completed.** This is the single most important
thing for long sequences. `ACCEPTED` means the fsm permitted the command and it
was handed to MAVROS — nothing more. Commands are fire-and-forget; the stack
never reads a service result, never checks arrival, and has no notion of a goal.
**Determining that a `GOTO` arrived, or that a `TAKEOFF` reached altitude, is
entirely your job**, from `/vehicle/telemetry`.

**GUIDED must come from outside.** `SET_MODE_GUIDED` is in the dispatch table
but in **no** permission-matrix state, so it is rejected in every state. The
stack cannot put itself into GUIDED. A GCS, MAVProxy or the pilot must select
it. On the current SITL config GUIDED is not on any `FLTMODE` switch position
either, so it must come from a GCS command.

**Mode changes take up to ~1.27 s to be seen.** `mode` and `armed` come from
`/mavros/state`, measured at ~0.79 Hz. After GUIDED is selected externally,
expect up to ~1.3 s of `cond=['FC_STATE']` before commands start being accepted.
Do not treat the first rejection after a mode change as authoritative.

**Velocity and acceleration are streams with a 3 s dead man.** `SET_VELOCITY`
starts a 10 Hz setpoint stream on the vehicle that persists until
`STOP_VELOCITY`. ArduPilot's `GUID_TIMEOUT` is 3.0 s, so if the stream ever
stops without a `STOP_VELOCITY` the vehicle holds the last setpoint for up to
3 s before timing out. Always close a velocity phase with `STOP_VELOCITY`.

---

## 6. `MALFORMED` — the boundary check

Run before the command reaches the state machine, so it never consumes a tick's
arbitration. Any of these gets `REJECTED` / `cond=['MALFORMED']` immediately:

- `len(param_names) != len(param_values)`
- `command` not in the vocabulary
- duplicate entries in `param_names`
- `set(param_names)` is not exactly the expected set

A well-formed message carrying a **bad value** is not malformed — it passes this
check and fails axis 4 as `SANITY`. The distinction is deliberate: `MALFORMED`
means your message was unusable, `SANITY` means your message was fine and the
numbers were not.

---

## 7. Writing a sequence

The pattern for every step:

1. Pick a fresh `token`. Publish one `VehicleCommand`.
2. Wait for a `VehicleStatus` whose `token` matches yours.
3. On `ACCEPTED`, the command was dispatched — now **poll telemetry** for the
   condition that means the step is done.
4. On `REJECTED`, inspect `cond` and decide: retry, wait, or abort.

Distinguish the two rejection shapes when deciding:

- `REJECTED` with `command=''` — a gate refused you. Fix the cause.
  `FC_STATE` means wait for GUIDED; `COMMAND_STATE` means you are in the wrong
  armed/in-air state; `SANITY` and `MALFORMED` mean your message was wrong and
  retrying it unchanged will fail identically.
- `REJECTED` with a **different** `command` — safety outranked you and that
  command is being flown instead. Your submission was not faulty. Stop the
  sequence and hand control to your safety handling; do not resubmit into an
  active safety condition.

### Worked example: takeoff, fly a leg, land

```
precondition: pilot or GCS selects GUIDED
              wait for telemetry.state.mode == "GUIDED"
              (equivalently: wait for status.cond to stop containing FC_STATE)

token=1  ARM                                  -> ACCEPTED
         wait for telemetry.state.armed == true

token=2  TAKEOFF altitude=5.0                 -> ACCEPTED
         wait for telemetry.state.in_air == true      # gates step 3
         wait for telemetry.local_position.up >= 5.0 - tolerance

token=3  GOTO_LOCAL east=10 north=0 up=5 yaw_deg=0
                                              -> ACCEPTED
         wait for local_position within tolerance of the target

token=4  LAND                                 -> ACCEPTED
         wait for telemetry.state.in_air == false
         wait for telemetry.state.armed == false      # ArduPilot auto-disarms
```

Note step 2 → 3: `GOTO_LOCAL` is only permitted **armed and in air**, and
`in_air` does not latch until altitude exceeds 0.5 m. Sending the `GOTO` too
early returns `cond=['COMMAND_STATE']`. Gate on `state.in_air`, which is the
same value the permission check uses — not on a timer, and not on your own
altitude threshold.

### Timing budget

| Signal | Rate | Implication |
|---|---|---|
| `/fs_adapter/status` | 20 Hz | Your answer arrives within ~50 ms of submission. |
| `/vehicle/telemetry` | 5 Hz | Arrival detection resolves to ~200 ms. |
| `state.mode` / `state.armed` inside telemetry | ~0.79 Hz | Mode and arm transitions lag up to ~1.3 s. |
| `local_position` inside telemetry | ~3.16 Hz | Position resolves to ~320 ms; at 2 m/s that is ~0.6 m. |

Set arrival tolerances against those numbers. A 0.5 m waypoint tolerance is
tighter than the position sample interval at cruise speed and will not converge
reliably.

---

## 8. Current limitations

Known and deliberate as of this writing:

- **The four safety conditions are stubs.** `RC_THROTTLE_STATUS`, `GEOFENCE`,
  `RC_LOSS` and `BATTERY` never fire. Safety-override handling should be written
  now, but it cannot be exercised yet.
- **Outside GUIDED, safety conditions are not even reported.** Axis 1 stops the
  chain before axis 2 runs, so in a non-GUIDED mode `cond` shows only
  `FC_STATE`, never the safety conditions that may also be active.
- **No deadman.** If the commander goes silent the vehicle keeps whatever it was
  doing. Nothing on the vehicle side detects a dead commander yet.
- **`disarmed, in air` accepts nothing.** Reachable, since `in_air` latches on
  altitude alone and a mid-air disarm leaves it true.
- **A failed MAVROS call is not reported.** `ACCEPTED` means the fsm permitted
  the command and the call was made; if the MAVROS service was unavailable the
  call quietly does nothing and you still see `ACCEPTED`. Watch
  `telemetry.state.mode` to confirm the vehicle actually acted.
