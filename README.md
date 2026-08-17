# ArduPilot Autonomy

A ROS 2 workspace for testing and developing autonomous flight sequences on
ArduPilot, over MAVROS. By [Useful Dynamics](https://usefuldynamics.io).

This is a development and test bench, not a finished product. The flight stack is
deliberately stateless: it executes one command at a time and reports what it
sees. Sequencing, retries, and mission logic live in the flight commander, above
the topic contract.

## Architecture

```
browser GUI ──MQTT ws:9001──┐
                            ├── broker :1883 ── udl_aa_fc ──/vehicle/command──┐
            udl_aa_gcs/*  ──┘      (gui_link)   (commander)                   │
                                                                        udl_aa_fs
                                        ◄──/vehicle/telemetry, /vehicle/status─┤
                                                                              │
                                                                  MAVROS ── ArduPilot
```

| Package | Role |
|---|---|
| `ardu_ws/src/udl_aa_fs` | Flight stack. `vehicle_controller` node — owns the MAVROS connection, FSM arbitration, and safety monitoring. |
| `ardu_ws/src/udl_aa_msgs` | Interface definitions. Build first; dependents link against the generated code. |
| `ardu_ws/src/udl_aa_fc` | Flight commander. `udl_aa_fc` node, plus four standalone test scripts. |
| `tools/gui` | Browser ground station (`udl-aa-gcs.html`), MQTT over websocket. |

### Topic contract

| Topic | Type | Direction |
|---|---|---|
| `/vehicle/command` | `VehicleCommand` | commander → stack |
| `/vehicle/status` | `VehicleStatus` | stack → commander |
| `/vehicle/telemetry` | `VehicleTelemetry` | stack → commander |

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
| 2 | `safety` | Throttle status, geofence, RC loss, battery. An active condition issues its own command and stops the chain, bypassing axes 3 and 4. |
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

When several safety conditions fire at once, severity decides
(`LAND` > `RTL` > `WAYPOINT_CMD` > `WARNING`), with condition priority as
tiebreak. A `WARNING`-tier entry names itself without winning — it is
information, not an override. Every active condition is reported, not just the
winner, since that list is what the operator sees.

### Safety awareness

The safety monitor runs its own thread, decoupled from the controller's tick
rate, and evaluates all conditions in a single pass. It exposes one snapshot
object via a non-blocking atomic read, so the tick always arbitrates over an
internally consistent set rather than picking up half its values from one
evaluation cycle and half from the next.

**Current status:** the framework, arbitration, and severity ranking are in
place and exercised end to end. `fc_state` is fully wired — mode, armed, and
in-air all derive from live telemetry. The individual safety sequences
(geofence, battery, RC loss, throttle status) are **still being developed** and
currently sit as inactive stubs that never trigger.

## Sequences in development

Both are flight-proven in their earlier standalone form and are being ported
onto the `/vehicle` topic contract. They live in `udl_aa_fc/sequences/backup/`
and are **not yet plumbed into the commander** — they will be included shortly.

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
bash scripts/build_msgs.sh          # udl_aa_msgs, always from clean
bash scripts/launch_fs.sh --build   # builds, then launches the flight stack
bash scripts/launch_fc.sh --build   # builds, then launches the commander
```

Rebuild both dependents in a freshly sourced shell after any `.msg` change.

## Running

```bash
# 1. MAVROS against SITL
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14555

# 2. Flight stack
bash scripts/launch_fs.sh

# 3. Flight commander
bash scripts/launch_fc.sh

# 4. Open tools/gui/udl-aa-gcs.html in a browser
```

### Test scripts

Standalone: each owns its node, runs start to finish, and confirms nothing. They
bypass the commander entirely.

```bash
bash scripts/launch_fc.sh --target 1   # test_goto_local   ENU diamond around origin
bash scripts/launch_fc.sh --target 2   # test_goto_body    RFU cross, spin at each end
bash scripts/launch_fc.sh --target 3   # test_velocity     ENU diamond on timed legs
bash scripts/launch_fc.sh --target 4   # test_goto_global  WGS-84 waypoints from GCS clicks
```

## Documentation

- [`docs/flight_stack_interface.md`](docs/flight_stack_interface.md) — the full
  topic interface, and how to build multi-step sequences on a stateless stack
- [`docs/sitl_reference.md`](docs/sitl_reference.md) — SITL parameters and
  measured topic rates

## License

Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
([CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)).

Share and adapt for non-commercial purposes, with attribution, under the same
license. For commercial licensing: hello@usefuldynamics.io
