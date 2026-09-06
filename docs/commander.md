# Safety commander

`udl_aa_ss/safety_commander.py` — runs one safety sequence at a time and decides
when it stops. Same shape as `udl_aa_fc/vehicle_commander.py`, with one
substitution: where that polls `gui_link` for an operator's button press, this
asks `safety_monitor` what the vehicle's own state is calling for.

---

## Tick

```
decide()  →  report  →  link check  →  rejection  →  arbitrate  →  update()
```

- **`decide()`** returns `List[Verdict]`, ordered. Index 0 is the winner;
  everything after it is information, never a queue.
- **report** logs every active condition, on change rather than per tick.
- **link check** — `fs.live` is two plain reads, so every sequence gets it free.
- **rejection** is checked *before* `update()`: a rejected command was never
  dispatched, so ticking the sequence again would have it waiting on telemetry
  that will never move.
- **arbitrate** starts, preempts, or leaves alone.
- **update()** returns `RUNNING` / `COMPLETE` / `ABORT`.

## Arbitration

Severity, not equality:

```python
if severity[wanted] > severity[self._active_name]:
```

Three consequences, all intended:

- A condition flickering across its threshold cannot restart a sequence.
- A **clearing** condition does not abandon the response to it — a battery that
  sagged under load and recovered when the vehicle slowed has not stopped being
  a reason to come home.
- A *less* severe condition cannot take the vehicle from a more severe one
  already handling it.

## Two deliberate differences from the flight commander

Both follow from the trigger being a **standing condition** rather than a
one-shot press.

**Preempt, not refuse.** `VehicleCommander._start` turns down a start while
something runs. Here a more severe condition takes the vehicle from a less
severe one — that is the point of a severity ladder.

**Severity decides, not equality.** The monitor re-derives its answer every
tick, so following index 0 literally would start and stop a sequence repeatedly
as a condition flickers.

---

## TBD

### 1. Standing conditions restart a terminated sequence every tick

**Confirmed defect.** `_stop()` is terminal in the flight commander because the
operator has to press the button again. Here the trigger is held down forever,
so `_stop` → condition still active → `_arbitrate` restarts on the next tick.

Measured: 5 ticks, 5 restarts.

```
smart_rtl started
smart_rtl failed
smart_rtl started
smart_rtl failed
...
```

It fires in the most likely real case — safety triggers while not in GUIDED, so
`smart_rtl._preflight` ABORTs, restarts, ABORTs. Same with a rejection: stop →
restart → resend → rejected → repeat, hammering `/safety/command` at tick rate.

**This is what the token is for.** Once the commander has acted on a token, a
condition still presenting that same token must not re-start the sequence. The
token is stubbed at 0 today, so the commander cannot yet tell a new firing from
a continuing one.

### 2. A failed `_start` after `_stop` leaves nothing running

```python
self._stop(f'preempted by {wanted}')
self._start(wanted)          # returns early if unregistered
```

A more severe condition with no registered sequence kills the working less
severe response and leaves the vehicle uncommanded.

Live today: `Message.FENCE_BREACHED` and `Message.RC_LOST` point at `'rtl'`, and
only `smart_rtl` is registered — so a fence breach during a battery `smart_rtl`
does exactly this.

Fix: resolve the sequence *before* stopping the incumbent.

### 3. The commander should not know battery thresholds

`LIMITS` hardcodes `batt_invalid_v`, `batt_critical_v`, `batt_very_low_v` and
`batt_low_v` purely to supply ROS parameter defaults. That puts volts in a file
whose only business is sequence names and lifecycle.

The defaults belong with the condition that reads them, or the commander should
declare parameters generically from the yaml without naming any of them.

### 4. Only one sequence is registered

`SEQUENCES` holds `smart_rtl` alone; `rtl`, `land`, `emergency_land` and
`motor_cutoff` are commented out. Any message pointing at them hits the
throttled `no sequence registered` error.
