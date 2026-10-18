# Vehicle commander

`udl_aa_fc/vehicle_commander.py` — decides which sequence should be running and
publishes a request for it. Both the nominal and the safety commander; there is
no separate safety node.

It does **not** run sequences. That is `udl_aa_seqs`.

---

## Layers

```
udl_aa_fc          conditions (inline) + gui_link + arbitration
     │             ── sequence request ──►
udl_aa_seqs        runs it; a new request preempts whatever is running
     │             ── VehicleCommand ──►
udl_aa_fs          executes; not sequence aware
     │
   MAVROS ── ArduPilot
```

Three-layer architecture: deliberative (`udl_aa_fc`), executive
(`udl_aa_seqs`), reactive control (`udl_aa_fs`).

---

## Tick

```
evaluate conditions inline  →  poll gui  →  arbitrate  →  publish or don't
```

That is the entire decision: **publish, or don't.** No `_start`, no `_stop`, no
`update()`, no registry, no lifecycle. Preemption happens in `udl_aa_seqs` when
a new request arrives, not here.

### Conditions are called inline

```python
evaluate(telemetry, limits, state) -> Verdict
```

No threads, no wrapper, no monitor. Each evaluator is a pure table walk over a
cached telemetry snapshot, so calling four of them costs microseconds and the
verdicts are computed fresh each tick rather than read from somewhere.

**The signature is what matters, not the plumbing.** It is unchanged from the
threaded design, so a condition that later becomes expensive — polygonal
geofencing is the obvious one — can be moved onto its own thread by wrapping
that one condition, with nothing else in the stack noticing.

**The commander holds one `state` dict per condition**, persisting across ticks.
Only `rc_loss` uses it today (last channel values and when they last moved), but
it is what lets a condition keep history without the caller knowing how it is
driven. Easy to forget, and its absence breaks exactly one condition silently.

### Parameters

Declared generically from the yaml, so the commander forwards `limits` without
naming a single threshold. A commander that knows what a volt is has the wrong
job.

---

## Arbitration

A separate module, so the policy is readable in one place and testable with no
ROS:

```python
arbitrate(candidates: List[Tuple[str, SafetyMessage]]) -> Optional[str]
```

Candidates are every sequence being asked for this tick — from conditions and
from the operator. The message rides along for logging; **v1 ranks on sequence
priority alone.** `None` means nothing should be running.

Rules:

- **Safety outranks the operator.** A condition asking for a sequence always
  beats a gui request.
- **Safety sequences rank against each other** by the severity ladder.
- **Abort is highest.** The operator can always abort, including a running
  safety sequence. That keeps the logic simple, and RC remains the real backstop
  since it takes the vehicle out of GUIDED regardless.

*(#TODO — "who called it" is in the table's remit but not implemented. v1 ranks
on the sequence only.)*

---

## The return path

`udl_aa_seqs` publishes a **sequence status** topic; the commander subscribes
through the same link it publishes requests on, so both directions live in one
file with one place for the QoS to be wrong.

| Topic | Type | Direction | QoS |
|---|---|---|---|
| `/sequence/request` | `SequenceRequest` | fc → seqs | RELIABLE, KEEP_LAST(10) |
| `/sequence/status` | `SequenceStatus` | seqs → fc | BEST_EFFORT, KEEP_LAST(1) |

```
# SequenceRequest.msg
std_msgs/Header header
string    sequence       # registry name: 'takeoff', 'smart_rtl', 'abort', 'manual'
uint32    token          # request id, monotonic; echoed in SequenceStatus
string[]  param_names    # manual carries its axis here
float64[] param_values
```

```
# SequenceStatus.msg
std_msgs/Header header

string RUNNING=RUNNING
string COMPLETE=COMPLETE
string ABORT=ABORT

string sequence          # '' when nothing is running
string state
uint32 token             # echoed from the request
```

**The sequence is named, not numbered, and the token does not carry the name.**
The flight stack packs `(seq_id << 16) | cmd_id` only because `VehicleCommand`
has no identity field to use; here there is one, so packing would put identity
in two places that can disagree. A name also survives `ros2 topic echo` legibly,
and means adding a sequence is one row in one registry rather than a name-to-id
table both nodes must agree on.

There is no tick counter in the token either. That existed to prove liveness
across silent ticks under the latch, and there is no latch — a running sequence
is its own evidence.

**The token is how the commander knows the running sequence is its own.** It
mints one per request; `udl_aa_seqs` echoes it back. Matching them distinguishes
*the sequence I asked for is now running* from a stale status, and tells a
completed sequence from one still going.

**Abort is a sequence name like any other** — `AbortSequence` already exists in
the registry, so it needs no field of its own; only its priority is special.

Params travel as parallel arrays, mirroring `VehicleCommand`, so `udl_aa_seqs`
zips them into the dict `on_start(params)` already expects.

Status publishes **every tick, including when nothing is running**. The
commander needs "nothing is running" as a positive fact; an event-only topic
cannot give it that after a dropped message.

---

## Deciding whether to publish

Two gates, in order.

**1. Is anything running?** Read from the status topic, matched by token.

| Running | Candidate | Publish? |
|---|---|---|
| nothing | anything | **yes** |
| something | `abort` | **yes** — abort always wins, including over safety |
| something | safety, higher priority than the incumbent | **yes** |
| something | safety, equal or lower | no |
| something | operator | no |

So a running sequence is **denied to everything** except an abort or a strictly
more severe safety sequence. Equal priority never displaces, which is what stops
a condition flickering across its threshold from restarting anything, and stops
a clearing condition from abandoning the response to it.

**2. Has the request changed?** Publish **on change only**. A standing condition
asks every tick; republishing the same request at 20 Hz would make
`udl_aa_seqs` responsible for deduplicating, which is the commander's problem to
not create.

The one exception is **manual / d-pad** — a sequence whose arguments are the
point, published repeatedly by design. Each publish is a different request
rather than a repeat of the same one.

---

## What was removed, and why

| Removed | Reason |
|---|---|
| `safety_monitor.py` | ranking twice is ranking twice; two stages can disagree, one cannot |
| `safety_commander.py` | one commander does both nominal and safety |
| `_ConditionThread` | evaluators are table walks; a thread per one is unwarranted until one is slow |
| cross-tick verdict storage | verdicts are computed fresh each tick |
| the latch | a started sequence runs until told to stop — the running sequence *is* the persistence |
| preemption here | it belongs where sequences run |

---

---

## Escalation stops at handover

A severity ladder implies you can always escalate. **You cannot, once a sequence
has handed the vehicle to an autopilot mode.** Observed end to end:

```
battery VERY_LOW   -> smart_rtl    GOTO_LOCAL, then RTL      mode becomes RTL
battery CRITICAL   -> land         land: refused - mode is GUIDED
```

`RTL` and `LAND` take the vehicle out of GUIDED, and from that point every
command in the vocabulary is refused — by the sequence's own preflight, and by
axis 1 of the flight stack behind it. There is no way around it either:
`SET_MODE_GUIDED` is listed in `vocabulary.UNREACHABLE` precisely because GUIDED
has to come from a GCS or the pilot.

So `LAND` outranking `RTL` in the table only holds *before* either is issued.
After that the autopilot owns the vehicle and the ladder is advisory.

Defensible — ArduPilot's RTL will put the vehicle down regardless, just later
and further away than a LAND would. But it means **a sequence that hands over
spends the stack's one decision**, which argues for `smart_rtl`'s `VERY_LOW`
threshold being conservative rather than eager.

---

## Open

1. **Rejection handling.** A rejection ends a sequence, and a standing condition
   will ask for it again as soon as the candidate changes and changes back.
   Publish-on-change blunts it — the same candidate is not re-requested, so
   there is no 20 Hz loop — but nothing retries a sequence that failed for a
   transient reason either. This is what a token was meant to resolve.

2. **Stale-command and sequence-id handling in the flight stack — TBD.** fs
   treats the token as opaque: stored, passed through `arbitrate` untouched,
   echoed back as `int(token)`. It never unpacks `seq_id`, and `msg.sequence`
   reaches only a desk-reject log line.

   Two separable pieces, and only one is wanted:

   - **The latch is not required.** Holding the vehicle for a `seq_id` across
     silent ticks belonged to the old design where two commanders published to
     fs independently. With one commander and one runner, fs sees a single
     publisher and has nothing to arbitrate between.

   - **Staleness gating is required, and by timestamp, not by counter.**
     `_command_callback` overwrites `_pending` unconditionally and DDS preserves
     per-publisher order, so fs already holds the newest command — a `cmd_id`
     high-water check would have nothing to reject. What survives is *age*: if
     the executor stalls, fs can arbitrate a two-second-old position target
     against a vehicle that has moved. Only `header.stamp` sees that. It is also
     restart-safe, where a counter is not — `cmd_id` restarts at 0 when the
     runner restarts, and a high-water mark would then silently lock out every
     command until it climbed past the old count.

   The clock is valid for this: unlike `mavros_interface`, which uses local
   receive time to avoid needing agreement with the FC, `/vehicle/command` comes
   from our own node on the same machine under the same ROS clock. Reject with
   `cond=['STALE']` rather than dropping silently, and put the threshold in a
   ROS parameter beside `fc_state_timeout_s`.

3. **`emergency_land` and `motor_cutoff` have no sequence.** They sit in the
   severity ladder and nothing in `constants.Message` points at them yet.

4. **Ranking does not consider who asked.** An operator `smart_rtl` and a
   battery `smart_rtl` are indistinguishable — see the TODO in `arbitrate.py`.
