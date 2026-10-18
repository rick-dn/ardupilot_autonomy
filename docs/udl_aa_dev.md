# udl_aa_dev

A development stack for flying sequences quickly. It exposes the MAVROS
adapter as two ROS 2 topics and nothing else — no state machine, no safety
monitor, no arbitration — so a sequence is an ordinary Python script you run
from a shell.

This package exists alongside `udl_aa_fs`, not instead of it. `udl_aa_fs` is
the flight stack: it wraps the same adapter in `fsm_handler`, `safety_monitor`,
`conditions/` and a 20 Hz tick, and every command is arbitrated before it
reaches the vehicle. That machinery is what you want in the air and what gets
in the way when you are trying out an idea. `udl_aa_dev` strips it to the
shortest path between a script and the flight controller.

**Use it for:** trying a manoeuvre in SITL, bringing up a new sensor,
prototyping a sequence before it earns a place in `udl_aa_seqs`.

**Do not use it for:** anything that needs a command refused. Nothing here
checks state, bounds, geofence or battery.

---

## 1. Layout

```
udl_aa_dev/
├── config/udl_aa_dev.yaml       # rates and the FC link timeout
├── launch/
│   ├── sitl.launch.py           # vehicle_node
│   └── flight.launch.py         # vehicle_node + rc_monitor
└── udl_aa_dev/
    ├── mavros_adapter.py        # MAVROS wrapper — not a node
    ├── vehicle_node.py          # the only node that touches the vehicle
    ├── rc_monitor.py            # flight only; RC switch → sequence
    └── sequences/
        ├── takeoff.py
        ├── goto_local.py
        └── land.py
```

Shell entry points live in the repo root, beside the other stacks':

```
scripts/launch_dev.sh            # SITL
scripts/launch_flight.sh         # physical
```

`mavros_adapter.py` is a verbatim copy of `udl_aa_fs/mavros_interface.py` with
the class renamed `MavrosInterface` → `MavrosAdapter`. **Frame conventions,
service calls and telemetry are therefore identical between the two stacks**,
and a fix to one should be ported to the other by hand — there is no shared
module.

---

## 2. The two topics

| Topic | Type | Direction | QoS |
|---|---|---|---|
| `/udl_dev/command` | `VehicleCommand` | sequence → vehicle | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/udl_dev/telemetry` | `VehicleTelemetry` | vehicle → sequence | BEST_EFFORT, VOLATILE, KEEP_LAST(1) |

**QoS must be compatible or DDS silently connects nothing.** A RELIABLE
subscriber will not match a BEST_EFFORT publisher. Subscribe to `telemetry` as
BEST_EFFORT; publish `command` as RELIABLE.

The `/udl_dev/*` root keeps this package clear of `udl_aa_fs`'s `/vehicle/*`
and `/fs_adapter/*`, so both stacks can be up at once without fighting over a
topic. They will still both command the same vehicle — run one at a time.

Telemetry publishes at 5 Hz, set by `telemetry_rate_hz`. It is deliberately
slower than you might expect: the quickest MAVROS group feeding it measures
~3 Hz, so publishing faster only emits byte-identical messages.

### There is no status topic

`udl_aa_fs` answers every command on `/fs_adapter/status`. This package does
not. **A sequence receives no acknowledgement of anything.** A command that
`vehicle_node` cannot dispatch is logged on the node side and is invisible to
the sender.

Sequences therefore confirm what happened by watching telemetry — mode became
`GUIDED`, `armed` went true, altitude reached the target. That is the whole
feedback channel, and it is the first thing to add back if this ever grows up.

---

## 3. Message shapes

### `VehicleCommand` (a sequence publishes)

```
std_msgs/Header header
string    sequence       # your sequence name; identity only, never addressing
uint32    token          # unused here — set 0
string    command        # from the vocabulary in section 4
string[]  param_names    # zipped with param_values into keyword arguments
float64[] param_values
```

`token` exists because the message is shared with `udl_aa_fs`, where it is
echoed back in `VehicleStatus`. With no status topic it has nothing to echo
into, so the sequences set it to `0`.

### `VehicleTelemetry` (a sequence subscribes)

One snapshot, grouped by category. The fields the bundled sequences use:

```
state.armed            bool
state.mode             string     # '' means no live FC link — see below
state.in_air           bool       # derived, with hysteresis; no FC field for it
local_position.east    float64    # ENU, metres from the EKF origin
local_position.north   float64
local_position.up      float64
local_position.yaw_deg float64    # CCW positive from East
rc.channels            uint16[]   # raw PWM µs; empty until the first RC message
```

Also carried: `home`, `global_position`, `velocity_body`,
`velocity_body_odom`, `velocity_gps`, `imu`, `battery`, `gps_status`,
`heading_deg`, `vfr_hud`, `statustext`.

### `state.mode == ''` means the link is dead

When `/mavros/state` has been silent longer than `fc_state_timeout_s` (5.0 s),
the adapter reports its startup values rather than the last data it saw. That
makes "link dead" and "not started yet" one state instead of two.

**Every other field in that blank snapshot is a legal value and will mislead
you.** Position reads as the origin, battery as flat, `up` as 0.0 — on the
ground, disarmed, no GPS. `mode` is the only discriminator, because no flight
controller ever reports an empty mode string. Check it before trusting
anything else:

```python
if self.tel.state.mode == '':
    return          # no link — not "on the ground, disarmed"
```

---

## 4. Command vocabulary

World-frame quantities are **ENU** (east/north/up). Body-frame quantities are
**RFU** (right/forward/up). Yaw is **CCW positive from East**, in degrees.

| Command | Parameters | Effect |
|---|---|---|
| `ARM` | — | Arm |
| `DISARM` | — | Disarm |
| `SET_MODE_GUIDED` | — | Mode → GUIDED |
| `TAKEOFF` | `altitude` | Climb to altitude (m, relative) |
| `GOTO_GLOBAL` | `lon`, `lat`, `alt`, `yaw_deg` | WGS-84 waypoint; `alt` relative to home |
| `GOTO_LOCAL` | `east`, `north`, `up`, `yaw_deg` | ENU waypoint from the EKF origin |
| `GOTO_BODY` | `right`, `forward`, `up`, `yaw_deg` | Offset from the current pose |
| `SET_VELOCITY` | `east`, `north`, `up`, `yaw_rate` | ENU velocity; `yaw_rate` rad/s CCW |
| `STOP_VELOCITY` | — | One zero-velocity setpoint |
| `SET_ACCEL` | `east`, `north`, `up` | ENU acceleration |
| `STOP_ACCEL` | — | One zero-acceleration setpoint |
| `LAND` | — | Mode → LAND |
| `RTL` | — | Mode → RTL |

Parameter names must match exactly — they are splatted into the adapter method
as keyword arguments. A misspelled or missing name surfaces as a `TypeError`
logged by `vehicle_node`; the sequence never hears about it.

### Setpoints are published once, never streamed

A position, velocity or acceleration command is sent as a single message. The
flight controller's `GUID_TIMEOUT` (3 s) stops the vehicle when setpoints stop
arriving, so a sequence that wants to hold a velocity must refresh it, and a
sequence that dies is stopped by the flight controller itself.

That property is the reason nothing here streams a keepalive: a standing
stream of zero setpoints would hold the vehicle in GUIDED and override
commands issued from anywhere else.

---

## 5. Writing a sequence

**Sequences are standalone. There is no base class and no helper module.**
Each file owns its node, its publisher, its subscriber, and its own copy of
`send()` and `wait_until()`.

This is a deliberate trade. Each sequence reads top to bottom with no
indirection, and editing one cannot break another — at the cost of repeating
~30 lines of setup per file. If you change the topic names or QoS, you change
them in every sequence.

The shape:

```python
class MySequence(Node):
    def __init__(self):
        super().__init__('my_sequence')
        self.tel = None
        self.cmd_pub = self.create_publisher(VehicleCommand, COMMAND_TOPIC, command_qos)
        self.create_subscription(VehicleTelemetry, TELEMETRY_TOPIC,
                                 self._telemetry_callback, telemetry_qos)

    def run(self):                      # linear, start to finish
        if not self.wait_until(lambda: self.tel.state.mode != '', 15.0, 'FC link'):
            return
        self.send('SET_MODE_GUIDED')
        ...

def main(args=None):
    rclpy.init(args=args)
    node = MySequence()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

**Spinning on a daemon thread is what makes `run()` linear.** Callbacks keep
arriving on the executor thread while `run()` blocks in `time.sleep()`, so the
flight logic reads as a flight plan rather than a callback graph.

Targets are module constants at the top of each file, not arguments. Edit and
re-run.

### Adding one

1. Write `sequences/<name>.py`.
2. Add `'<name> = udl_aa_dev.sequences.<name>:main'` to `console_scripts` in
   `setup.py`.
3. Add a target number to the `case` block in `scripts/launch_dev.sh`.

### Reading a vision node

A sequence that needs vision adds one subscription in its own `__init__`:

```python
self.create_subscription(Detection, '/vision/target', self._target_cb, 1)
```

Nothing else in the package learns about it. `vehicle_node` stays unaware, and
`package.xml` only changes if the vision node publishes a custom message type.

---

## 6. Running it — SITL

Two terminals. **SITL and MAVROS must already be running** — neither script
starts them, same as `launch_fs.sh`.

```bash
bash scripts/launch_dev.sh --build        # terminal 1: vehicle_node
bash scripts/launch_dev.sh --target 1     # terminal 2: one sequence
```

| Target | Executable | What it does |
|---|---|---|
| *(none)* | `ros2 launch udl_aa_dev sitl.launch.py` | The node |
| 1 | `takeoff` | GUIDED, arm, climb to 5 m |
| 2 | `goto_local` | Fly to E+10 m at 5 m, then stop |
| 3 | `land` | LAND mode, wait for touchdown and disarm |

Start the node once and run sequences against it as many times as you like.
`--build` cleans and rebuilds the package first.

`sitl.launch.py` deliberately does **not** start `rc_monitor`: there is no RC
in SITL, so sequences are triggered from the shell.

### Interrupting

Ctrl-C ends the script. **It does not stop the vehicle** — the flight
controller keeps flying the last setpoint it was given, so interrupting a climb
leaves it climbing. `GUID_TIMEOUT` catches velocity and acceleration commands
after 3 s, but not a `TAKEOFF` or a `GOTO`.

`land` is the exception: LAND is a flight-controller mode, not a setpoint, so
the descent continues whether or not the script is alive.

---

## 7. Running it — physical flight

```bash
bash scripts/launch_flight.sh --build
```

Brings up `vehicle_node` **and** `rc_monitor`. MAVROS is assumed to be running
already — on the Pi it is `mavros.service`.

| RC input | Spawns |
|---|---|
| `channels[7]` rises above 1800 µs | `ros2 run udl_aa_dev takeoff` |
| `channels[5]` rises above 1800 µs | `ros2 run udl_aa_dev land` |

`rc_monitor` spawns exactly the command `launch_dev.sh --target N` runs, so a
sequence proven in SITL is the same executable the switch starts. And because
the topics are identical, `launch_dev.sh --target N` still works against the
flight node for ground testing.

### rc_monitor is carried over unmodified

It is the flight-tested file from the Pi
(`~/ardu_ws/src/ardupilot_autonomy/ardupilot_autonomy/rc_monitor.py`), changed
only in the two executable names it spawns. **Its known quirks were kept
deliberately** — this is the version that has flown, and modernising it is work
for when physical flight is properly in scope:

1. `last_switch*` start at `1000`, so **booting with a switch already high
   reads as a rising edge and launches immediately**.
2. An empty `channels` array is not distinguished from a low switch.
3. A falling edge does nothing — there is no abort.
4. `Popen` handles are not kept, so two flicks start two sequences, both
   commanding the vehicle.

It also subscribes to `/mavros/rc/in` directly rather than reading
`/udl_dev/telemetry`, so on the flight launch `vehicle_node` is not the only
MAVROS client. The subscription is read-only, so it does not contend for
control.

---

## 8. What is deliberately absent

Everything in this list exists in `udl_aa_fs` and was left out on purpose.
Read it as the price of the simplicity, not as a backlog.

| Missing | Consequence |
|---|---|
| `fsm_handler` | No command is ever refused on state grounds |
| `safety_monitor`, `conditions/` | Geofence, battery, RC loss and throttle are not watched |
| Sanity bounds | A 500 m takeoff altitude is dispatched as readily as a 5 m one |
| The 20 Hz tick | Nothing is re-evaluated; commands are one-shot |
| `VehicleStatus` | No acknowledgement — see section 2 |
| Desk-checking | A malformed command is a logged `TypeError`, not a reply |
| Single-owner enforcement | Two sequences can run at once, both commanding |

The last one is worth repeating: **nothing stops you running `takeoff` by hand
while `rc_monitor` has already started something.** Both publish to the same
topic and the flight controller takes whichever setpoint arrived last.

---

## 9. Configuration

`config/udl_aa_dev.yaml`, loaded by both launch files:

```yaml
vehicle_node:
  ros__parameters:
    telemetry_rate_hz: 5.0
    logger_rate_hz: 1.0
    fc_state_timeout_s: 5.0
```

`fc_state_timeout_s` is declared by `MavrosAdapter`, not by the node, and
governs the blank-telemetry behaviour in section 3. It must exceed the
`/mavros/state` period — measured ~1.06 s, worst observed 1.27 s. 3.0 s flapped
in practice and reported the link down on ordinary jitter; 5.0 s tolerates
three missed heartbeats.

`rc_monitor` takes no parameters. Its thresholds and channel indices are
hardcoded, as they are on the Pi.

---

## See also

- [`flight_stack_interface.md`](flight_stack_interface.md) — the full
  `udl_aa_fs` topic contract, including the status path this package omits
- [`sitl_reference.md`](sitl_reference.md) — measured MAVROS topic rates,
  GUIDED behaviour, failsafes and arming in SITL
- [`safety_stack.md`](safety_stack.md) — the conditions this package does not
  evaluate
