# ArduPilot Autonomy

A ROS 2 workspace for testing and developing autonomous flight sequences on
ArduPilot, over MAVROS. By [Useful Dynamics](https://usefuldynamics.io).

This is a development and test bench, not a finished product. The flight stack is
deliberately stateless: it executes one command at a time and reports what it
sees. Sequencing, retries, and mission logic live above the topic contract.

## Architecture

A **hybrid deliberative/reactive** stack in the **three-tiered (3T)** form —
deliberation, sequencing, reactive skills — one node per tier. See
[`docs/architecture.md`](docs/architecture.md) for the philosophy and the
diagrams.

```
browser GUI ──MQTT ws:9001──┐
                            ├── broker :1883 ── udl_aa_fc
            udl_aa_gcs/*  ──┘      (gui_link)   deliberative - decides WHAT
                                                     │
                                     /sequence/request, /sequence/status
                                                     │
                                                udl_aa_seqs
                                                executive - RUNS one at a time
                                                     │
                                      /vehicle/command, /vehicle/status
                                                     │
                                                 udl_aa_fs
                                                 reactive - EXECUTES
                                                     │
                                             MAVROS ── ArduPilot
```

The commander decides which sequence should be running and runs none of them.
The sequence node runs exactly one at a time and is the only thing that issues
commands. The flight stack executes commands and knows nothing about sequences.

| Package | Role |
|---|---|
| `ardu_ws/src/udl_aa_msgs` | Interface definitions. Build first; dependents link against the generated code. |
| `ardu_ws/src/udl_aa_fc` | Flight commander. Evaluates the safety conditions, polls the operator, arbitrates, and publishes one sequence request. |
| `ardu_ws/src/udl_aa_seqs` | Sequences. Every phase machine, one running at a time, plus five standalone test scripts. |
| `ardu_ws/src/udl_aa_fs` | Flight stack. `vehicle_controller` node — owns the MAVROS connection and the four-axis arbitration. |
| `tools/gui` | Browser ground station (`udl-aa-gcs.html`), MQTT over websocket. |

### Topic contract

| Topic | Type | Direction |
|---|---|---|
| `/sequence/request` | `SequenceRequest` | commander → sequences |
| `/sequence/status` | `SequenceStatus` | sequences → commander |
| `/vehicle/command` | `VehicleCommand` | sequences → stack |
| `/vehicle/status` | `VehicleStatus` | stack → sequences |
| `/vehicle/telemetry` | `VehicleTelemetry` | stack → both |

Telemetry is a broadcast with no addressee. The sequence node needs it to decide
when a phase is done and the commander needs it to evaluate the safety
conditions, so both subscribe directly rather than one relaying to the other.

The GUI leg is separate, carried over MQTT rather than ROS: `udl_aa_gcs/cmd`,
`udl_aa_gcs/telemetry`, `udl_aa_gcs/log`.

## Inside the flight stack

### Four-axis FSM handler

Every command is arbitrated through four explicit pass/fail gates, evaluated in
order and short-circuiting on the first failure. The ordering *is* the command
chain — an axis is only reached once every axis above it has passed.

| # | Axis | Gate |
|---|---|---|
| 1 | `fc_state` | The stack commands only in GUIDED. Any other mode stops the chain outright — safety included. |
| 2 | `safety` | **Vestigial.** Safety moved up to the commander; these conditions are inactive stubs in the flight stack and never fire. See [`docs/commander.md`](docs/commander.md). |
| 3 | `command state` | An `(armed, in_air)` permission table. `ARM` when disarmed; `TAKEOFF`/`DISARM` when armed on ground; goto/velocity/accel/land/RTL only in air. |
| 4 | `sanity` | The submitted arguments against configured limits — altitude bounds, body-step distance, velocity and acceleration magnitude, finite-value checks. |

The handler is pure: no thread of its own, no `rclpy` dependency, nothing
remembered between calls. It is invoked synchronously from the controller's
tick, so the same inputs always produce the same decision.

It returns a four-part decision — the submission's outcome
(`ACCEPTED` / `REJECTED` / `IDLE`), the accumulated condition names, the one
command to dispatch, and the caller's token passed back for matching. Outcome
and command are independent: a `REJECTED` carrying a command means safety
outranked the submission.

## Safety

Safety lives in the commander, not the flight stack. A condition grades the
vehicle's state into a band, the band names a message, and the message carries
both the operator-facing wording and the sequence it implies — so what the
operator reads and what the vehicle does are defined on one row and cannot drift
apart.

| Condition | Bands | Asks for |
|---|---|---|
| `battery` | `LOW` / `VERY_LOW` / `CRITICAL` | warning / `smart_rtl` / `land` |
| `geofence` | `NEAR` / `BREACHED` | warning / `rtl` — **stub** |
| `rc_loss` | `LOST` | `rtl` — **stub** |
| `rc_throttle` | `LOW` / `MEDIUM` / `HIGH` | warning only — **stub** |

One table ranks every candidate, the operator's own requests included, so
"safety outranks the operator" falls out of the ordering rather than needing a
separate check. `abort` sits above everything: the operator can always stop the
vehicle, including a running safety sequence.

Thresholds are ROS parameters in [`udl_aa_fc/config/safety.yaml`](ardu_ws/src/udl_aa_fc/config/safety.yaml),
declared generically so the commander names none of them — a commander that
knows what a volt is has the wrong job.

**Current status:** `battery` is live end to end. `geofence`, `rc_loss` and
`rc_throttle` are stubs that never fire. `emergency_land` and `motor_cutoff`
exist in the severity ladder but have no sequence yet.

The last-resort failsafe is ArduPilot's own, configured rather than coded — see
[`docs/safety_stack.md`](docs/safety_stack.md) for the parameters and the two
ways it silently fails to arm.

## Sequences in development

Both are flight-proven in their earlier standalone form and are being ported
onto the `/vehicle` topic contract. They live in `udl_aa_seqs/sequences/backup/`
and are **not yet plumbed into the runner** — they will be included shortly.

### Follow person — *flight tested*

Image-based visual servoing onto a detected person. A four-state constant-velocity
Kalman filter tracks the bounding-box centroid, smoothing detection jitter and
coasting through dropped frames; outlier rejection discards detections that jump
too far from the prediction, and a lost-target timeout falls back to hover.
Velocity commands are yaw-corrected for camera mounting angle.

The detector is **not** included — any node publishing
`vision_msgs/Detection2DArray` on `/detections/persons` will drive it.

- [Flight test](https://www.youtube.com/shorts/YHh4Hv-ZMoA)
- [SITL test](https://www.youtube.com/watch?v=sQa5da0TWBg)

### ArUco landing — *tested in SITL*

Precision landing onto a marker, phased `SCAN → HOVER_CAPTURE → APPROACH →
autopilot LAND`. The capture phase gates on a sliding detection window with
recency and coherence floors, then arms or abandons on a confidence threshold,
so a brief false positive cannot commit the vehicle to a descent. Approach
aligns yaw within a few degrees before descending at a fixed rate to the
handover altitude, where the autopilot's own LAND takes over.

- [SITL test](https://www.youtube.com/watch?v=ZdOmPALt-c0)

## Dependencies

- ROS 2 Humble
- MAVROS
- ArduPilot SITL or real hardware
- An MQTT broker with a websocket listener on 9001 (TCP 1883) — nothing reaches
  the GUI without it
- `pip install paho-mqtt geographiclib numpy`

## Build

Message definitions first — a stale generated interface is the usual cause of an
import that fails only at runtime.

```bash
bash scripts/build_msgs.sh            # udl_aa_msgs, always from clean
bash scripts/launch_fs.sh --build     # builds, then launches the flight stack
bash scripts/launch_seqs.sh --build   # builds, then launches the sequence runner
bash scripts/launch_fc.sh --build     # builds, then launches the commander
```

Rebuild every dependent in a freshly sourced shell after any `.msg` change.

## Running

Bottom up. Each layer's liveness gate is a subscriber count on the topic it
publishes to, so starting from the top means a node that publishes into nothing
and silently looks idle rather than failing.

```bash
# 1. MAVROS against SITL
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14555

# 2. Flight stack
bash scripts/launch_fs.sh

# 3. Sequence runner
bash scripts/launch_seqs.sh

# 4. Flight commander
bash scripts/launch_fc.sh

# 5. Open tools/gui/udl-aa-gcs.html in a browser
```

### Test scripts

Standalone: each owns its node, runs start to finish, and confirms nothing. They
bypass the runner and the commander entirely.

```bash
bash scripts/launch_seqs.sh --target 1   # test_goto_local   ENU diamond around origin
bash scripts/launch_seqs.sh --target 2   # test_goto_body    RFU cross, spin at each end
bash scripts/launch_seqs.sh --target 3   # test_velocity     ENU diamond on timed legs
bash scripts/launch_seqs.sh --target 4   # test_goto_global  WGS-84 waypoints from GCS clicks
bash scripts/launch_seqs.sh --target 5   # aruco_lander      scan, settle, servo down
```

## Documentation

- [`docs/architecture.md`](docs/architecture.md) — the pattern, the philosophy
  behind it, diagrams to refer to, and what is still needed for production
- [`docs/commander.md`](docs/commander.md) — the commander's tick, the
  arbitration rule, the sequence request/status contract, and what is still open
- [`docs/safety_stack.md`](docs/safety_stack.md) — the condition model, and
  ArduPilot's own failsafe parameters with the two ways they silently fail to arm
- [`docs/flight_stack_interface.md`](docs/flight_stack_interface.md) — the full
  topic interface, and how to build multi-step sequences on a stateless stack
- [`docs/sitl_reference.md`](docs/sitl_reference.md) — SITL parameters and
  measured topic rates

## License

Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
([CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)).

Share and adapt for non-commercial purposes, with attribution, under the same
license. For commercial licensing: hello@usefuldynamics.io
