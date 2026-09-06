# Safety stack

`udl_aa_ss` — a second commander that outranks the flight commander. This is the
design and the implementation order; it is not yet built.

> **Status.** `constants.py` and `safety_monitor.py` exist, with `battery` live
> and `geofence` / `rc_loss` / `rc_throttle` stubbed. Everything below the
> monitor — commander, sequences, link, and the flight-stack changes — is
> designed here and not yet written.

---

## 1. Why it exists

Safety currently lives inside the flight stack as `udl_aa_fs/safety_monitor.py`,
feeding `fsm_handler`'s axis 2, which ranks active conditions and emits **one
single-tick command**.

That shape cannot express a safety response. A real one is a sequence — square
up, hand to RTL, watch until down — spanning hundreds of ticks. Axis 2 has no
memory, no phases, and no way to hold the vehicle between commands. Meanwhile
`udl_aa_fc` already has exactly that machinery.

So safety moves out into its own package, shaped like `udl_aa_fc`, and
`udl_aa_fs` stops evaluating conditions and becomes a pure arbiter between two
command sources.

---

## 2. Structure

`udl_aa_ss` is `udl_aa_fc` with one substitution: where `gui_link` supplies
"what should run" from an operator pressing a button, `safety_monitor` supplies
it from the vehicle's own state.

```
conditions/*.py      N threads, one question each      → Verdict
        │                                                (message, token)
        ▼
safety_monitor.py    ranks verdicts, returns them       → [Verdict, ...]
        │            ordered. Index 0 is the winner.       index 0 wins
        ▼
safety_commander.py  THE NODE. Owns the 20 Hz tick.     → start / preempt / stop
        │            Lifecycle owner.
        ▼
sequences/*.py       phase machines. Command on          → send(RTL)
        │            transitions, watch in between.
        ▼
fs_link.py           the only file that touches ROS      → /safety/command
```

| Component | Decides |
|---|---|
| a condition | what its own numbers mean, and which `Message` to raise |
| `safety_monitor` | **what** should run — ranks, returns ordered |
| `safety_commander` | **when** — start, preempt, stop; talks to `fs_link` |
| `udl_aa_fs` | whether to execute it at all |

`safety_monitor` never touches `fs_link`. It holds no thresholds and knows
nothing about volts, metres or microseconds.

---

## 3. Messages and severity

A condition never names a sequence directly. It picks a `Message`, and the
`Message` carries both the operator-facing text and the sequence it implies, so
the two are defined on one row and cannot drift apart.

```python
@dataclasses.dataclass(frozen=True)
class SafetyMessage:
    text: str
    sequence: Optional[str] = None      # None = warning, runs nothing
```

The ladder, lowest to highest:

| Sequence | Severity |
|---|---|
| *(none — warning)* | 1 |
| `srtl` | 2 |
| `rtl` | 3 |
| `land` | 4 |
| `emergency_land` | 5 |
| `motor_cutoff` | 6 |

Ranking is over the **sequence**, not the condition, so a battery asking to land
outranks a fence asking to return regardless of which raised it. Warnings rank
lowest, which is what lets them need no special case: a real sequence always
outranks one, and if only warnings are active the winner's sequence is `None`
and nothing runs.

---

## 4. What `safety_monitor` returns

```python
decide() -> List[Verdict]
```

Ordered, **index 0 is the winner**:

```python
key = lambda v: (-SEVERITY[v.message.sequence], PRIORITY[v.condition])
```

Everything after index 0 is **information, never a queue** — it exists so the
operator and both stacks can see every active condition, not just the one being
acted on. An empty list means nothing is active.

`Verdict` gains a `condition` field, stamped by `_ConditionThread` when it
stores the result. The tiebreak needs it and anything reading the list
downstream needs to know who raised each message; the evaluators still do not
set it.

---

## 5. Topics

A separate channel, not a shared one. Because the source is identified by the
topic, sequence names cannot collide between the two stacks and need no
namespacing.

| Topic | Type | Direction |
|---|---|---|
| `/safety/command` | `VehicleCommand` | ss → fs |
| `/safety/status` | `VehicleStatus` | fs → ss |
| `/vehicle/telemetry` | `VehicleTelemetry` | fs → ss (shared) |

QoS matches the fc channel exactly — commands RELIABLE `KEEP_LAST(10)`, status
and telemetry BEST_EFFORT `KEEP_LAST(1)`. Mismatched QoS makes DDS connect
nothing and say nothing.

`udl_aa_fs` is **not sequence-aware**. It executes commands and compares a
string; it never learns what a sequence does.

---

## 6. The latch

### The problem

A safety sequence commands on phase transitions and is **silent in between**.
`smart_rtl` issues `RTL` and then sends nothing for up to 180 s while ArduPilot
flies home; `takeoff` sends two commands across its whole life and is silent for
the entire climb.

So priority alone is not enough. Priority only decides ticks where both sources
submit, and those are the rare ones — on the overwhelming majority of ticks the
safety stack is quiet and the flight commander would win by default, in the
middle of a safety response.

### The mechanism

The safety stack publishes **whenever there is a situation** — a command to act
on, or a warning with nothing to do about it — and stays quiet otherwise. It
does not republish on empty ticks.

### The message is identical to the flight commander's

The two channels use the same conventions. `token` is minted exactly as
[`sequence.py`](../ardu_ws/src/udl_aa_ss/udl_aa_ss/sequence.py) already does it —
`(seq_id << 16) | cmd_id`, a new one per command — so `udl_aa_ss/sequence.py`
stays a verbatim copy of the flight commander's and nothing diverges.

The `sequence` field is **deprecated**. It carries the node name, is used
nowhere but a desk-reject log line, and is not being repurposed. Left as-is; no
action.

### One topic, with and without a command

Both kinds of situation ride on `/safety/command`. The `command` field is filled
only when something is to be acted on.

**An empty command is not an empty message.** It says there is a safety
situation that needs no intervention — a warning. A filled command says act on
this as well.

A real command goes out only on phase transitions, never repeatedly: re-issuing
every tick would overwrite submissions that are then never arbitrated and never
answered, which is what `sequence.py` warns against.

> **Open — the latch is not designed yet.** The safety stack publishes only on a
> situation and never on a quiet tick, so nothing currently tells fs that a
> silent sequence is still alive and still holds the vehicle. Until that is
> settled, step 5 below gives per-tick priority only, which we know is not
> sufficient on its own. Note also that `token` is a command counter, so it
> cannot double as a liveness signal.

---

## 7. Changes in `udl_aa_fs`

- `fs_interface` gains the `/safety/command` subscription and the
  `/safety/status` publisher. `report()` splits per source.
- `fsm_handler` axis 2 changes from *"which condition fires"* to *"who holds the
  vehicle"*. The four-axis chain survives; only that gate's meaning changes.
- `ACTION_SEVERITY`, `CONDITION_PRIORITY` and `_safety_override` are deleted —
  they move to `udl_aa_ss`, which owns conditions now.
- `safety_monitor.py` is deleted. Its one real function, `_eval_fc_state`,
  becomes a pure `fc_state.from_telemetry()`. `udl_aa_fs` then has no threads.

Axes 1, 3 and 4 apply to safety commands too. Those are vehicle truths, not
commander policy — a safety `TAKEOFF` on the ground is still nonsense.

### One constraint every safety sequence must respect

Axis 1 is still GUIDED-only. `RTL` and `LAND` take the vehicle out of GUIDED, so
from that command onward every further command is refused — the safety stack's
included. A sequence that hands over to an autopilot mode must be watch-only
afterwards.

---

## 8. Last-resort failsafe

Not in this stack. `udl_aa_fs` and `udl_aa_ss` are one system on one machine —
same workspace, same launch, same power — so defending fs against ss dying
covers a narrow case while the dominant failure, the whole companion going away,
goes uncovered.

That belongs at the `fs`↔ArduPilot boundary, configured rather than coded:

| Param | Value | Effect |
|---|---|---|
| `MAV_GCS_SYSID` | MAVROS's sysid | makes MAVROS the GCS ArduPilot watches |
| `FS_GCS_ENABLE` | `1` | CC loss → RTL |
| `FS_GCS_TIMEOUT` | `5` | seconds before firing |
| `FS_THR_ENABLE` | `1` | RC loss → RTL |
| `FS_OPTIONS` | `20` | bit 2 + bit 4 |

`FS_OPTIONS` bit 2 (*continue if in Guided on RC failsafe*) is **not optional**:
without it, RC loss drops the vehicle out of GUIDED and axis 1 then refuses
every command from fs and ss alike — the autonomy stack dies at the same moment
the pilot does. Bit 4 (*continue in pilot control on GCS failsafe*) hands over
to the pilot when the companion dies. Both die → neither exception applies → RTL.

Two traps, both silent:

- The failsafe **never arms** unless a heartbeat from `MAV_GCS_SYSID` has been
  seen at least once (`Copter::failsafe_gcs_check()` returns early on
  `gcs_last_seen_ms == 0`). MAVROS defaults to sysid 1, which collides with the
  vehicle's own and never counts as a GCS.
- MAVROS heartbeats **independently of `vehicle_controller`**. Companion power
  loss, kernel panic and a pulled cable take both; a bare node crash does not.
  Fix at the process level — one launch file with `on_exit=Shutdown()`, or a
  systemd `BindsTo=`.

---

## 9. Implementation order

| # | Step | Blocked on |
|---|---|---|
| 1 | `decide()` returns an ordered `List[Verdict]`; `Verdict` gains `condition` | — |
| 2 | `sequence.py` base + five dummy sequences | 1 |
| 3 | `safety_commander.py` — tick, registry, preempt | 2 |
| 4 | `fs_link.py` + `vocabulary.py` (copied, not imported) | — |
| 5 | `/safety/command` + `/safety/status` in `udl_aa_fs`, per-tick priority | 4 |
| 6 | The latch in `fsm_handler` | 5 |
| 7 | Tear down `udl_aa_fs/safety_monitor.py` → `fc_state.py` | 6 |
| 8 | Fill in the three stubbed conditions | — |

Steps 5 and 6 are deliberately staged: 5 gives per-tick priority, which we know
is insufficient on its own, and 6 adds the latch that makes it correct. Shipping
5 without 6 is a conscious interim state, not a finished one.

### Two deliberate deviations from `udl_aa_fc`

**Preempt, not refuse.** `VehicleCommander._start` turns down a start while
something runs. Safety must do the opposite: a higher-severity condition firing
mid-sequence stops the running sequence and starts the more severe one.

**`vocabulary.py` and `sequence.py` are copied, not imported.** Importing them
from `udl_aa_fc` would make the safety stack depend on the commander it
overrides. This makes a third copy of the command/parameter table; that is a
known cost, accepted for now.
