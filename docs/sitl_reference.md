# ArduPilot SITL reference

Captured 2026-08-13 from a running SITL instance (`param show` in the MAVProxy
console, mode STABILIZE), alongside `ros2 topic hz` measurements taken against
the same session.

This is a **curated** extract, not the verbatim dump — only the parameters that
bear on this autonomy stack, annotated with why they matter. To capture the
complete set (~1000 parameters) next to this file:

```
# from the MAVProxy console
param show > /data/projects/ardupilot_autonomy/docs/sitl_params_full.txt
```

---

## 1. Measured MAVROS topic rates

| Topic | Rate | Period | std dev |
|---|---|---|---|
| `/mavros/state` | 0.786 Hz | 1.27 s | 0.0045 s |
| `/mavros/local_position/odom` | 3.16 Hz | 0.317 s | 0.017 s |

Both are metronomic, not ragged, so this is deliberate pacing rather than
packet loss on the link.

### Why they're these rates

Every stream-rate parameter is zero, on all three MAVLink channels:

```
MAV1_ADSB 0   MAV1_EXT_STAT 0   MAV1_EXTRA1 0   MAV1_EXTRA2 0   MAV1_EXTRA3 0
MAV1_PARAMS 0 MAV1_POSITION 0   MAV1_RAW_CTRL 0 MAV1_RAW_SENS 0 MAV1_RC_CHAN 0
MAV2_* ... all 0
MAV3_* ... all 0
```

(`MAVn_*` is the modern name for what older docs call `SRn_*`.)

So ArduPilot is **not** scheduling any streams of its own. Everything arriving
at MAVROS is there because MAVROS asked for it via `SET_MESSAGE_INTERVAL` /
`REQUEST_DATA_STREAM` at startup. **Consequence: raising these rates is a MAVROS
configuration change, not an ArduPilot parameter change.** Setting `MAV1_POSITION`
here will not speed up `/mavros/local_position/odom`.

### Consequences for this stack

- `fc_state` reads `mode` and `armed` from `/mavros/state`, so **axis 1 of the
  fsm samples a ~0.79 Hz signal**. A mode change takes up to ~1.27 s to reach
  the gate, in both directions. The 20 Hz tick buys nothing there.
- `in_air` derives from `local_position.up` at 3.16 Hz, ~317 ms per sample.
  Climbing at 2 m/s covers ~0.63 m between samples — wider than the entire
  hysteresis band (0.5 m up, 0.2 m down). The latch typically jumps straight
  through the band in a single sample.
- The 20 Hz tick and the 10 Hz safety-monitor thread are both heavily
  oversampling. `VehicleTelemetry` should publish at ~4–5 Hz, not 20 Hz.

---

## 2. GUIDED behaviour

| Parameter | Value | Note |
|---|---|---|
| `GUID_TIMEOUT` | 3.0 | **Guided setpoints expire after 3 s of silence.** |
| `GUID_OPTIONS` | 0 | |

`mavros_interface.set_velocity` / `set_acceleration` stream at 10 Hz, well
inside this. But if the streaming timer is destroyed — which `stop_velocity()`
and `stop_acceleration()` do deliberately — the vehicle holds the last setpoint
for up to 3 s before ArduPilot times it out. The explicit zero-setpoint publish
before teardown is what avoids that.

## 3. Flight modes

| Parameter | Value |
|---|---|
| `INITIAL_MODE` | 0 (Stabilize) |
| `FLTMODE_CH` | 5 |
| `FLTMODE1` | 7 Circle |
| `FLTMODE2` | 9 Land |
| `FLTMODE3` | 6 RTL |
| `FLTMODE4` | 3 Auto |
| `FLTMODE5` | 5 Loiter |
| `FLTMODE6` | 0 Stabilize |

**GUIDED is not on any switch position.** It can only be entered by a GCS or
companion command. Since the fsm gates every command on `mode == GUIDED`, and
`SET_MODE_GUIDED` is in no `ALLOWED_COMMANDS` set, nothing the stack sends can
put the vehicle into a state where the stack can command it — GUIDED has to come
from MAVProxy or a GCS.

## 4. Failsafes — what the safety_monitor stubs will read

### `RC_THROTTLE_STATUS`
| Parameter | Value |
|---|---|
| `FS_THR_ENABLE` | 1 (always RTL) |
| `FS_THR_VALUE` | 975 |
| `RC3_MIN` / `RC3_TRIM` / `RC3_DZ` | 1000 / 1500 / 30 |

### `GEOFENCE`
| Parameter | Value |
|---|---|
| `FENCE_ENABLE` | **0 — disabled** |
| `FENCE_ACTION` | 1 (RTL or Land) |
| `FENCE_TYPE` | 7 (max alt, circle, polygons) |
| `FENCE_ALT_MAX` / `FENCE_ALT_MIN` | 100.0 / -10.0 |
| `FENCE_RADIUS` / `FENCE_MARGIN` | 150.0 / 2.0 |
| `FENCE_TOTAL` | 0 |

The fence is configured but switched off, so there is currently nothing for a
`GEOFENCE` condition to observe.

### `BATTERY`
| Parameter | Value |
|---|---|
| `BATT_MONITOR` | 4 (analog V + I) |
| `BATT_LOW_VOLT` | 10.5 |
| `BATT_CRT_VOLT` | 0.0 |
| `BATT_FS_LOW_ACT` | **0 — None** |
| `BATT_FS_CRT_ACT` | **0 — None** |
| `BATT_CAPACITY` | 5000000 |
| `SIM_BATT_VOLTAGE` | 12.6 |

ArduPilot will take **no** action on low battery. If a `BATTERY` condition is
implemented in the safety monitor it is the only thing protecting the vehicle.

### `RC_LOSS`
| Parameter | Value |
|---|---|
| `RC_FS_TIMEOUT` | 1.0 |
| `RC_OPTIONS` | 32 |
| `SIM_RC_FAIL` | 0 (disabled — set to inject loss) |

### Other failsafes
| Parameter | Value |
|---|---|
| `FS_GCS_ENABLE` / `FS_GCS_TIMEOUT` | 0 (disabled) / 5.0 |
| `FS_EKF_ACTION` / `FS_EKF_THRESH` | 1 (Land) / 0.8 |
| `FS_DR_ENABLE` / `FS_DR_TIMEOUT` | 2 (RTL) / 30 |
| `FS_CRASH_CHECK` | 1 |
| `FS_OPTIONS` | 16 |

## 5. Arming

| Parameter | Value |
|---|---|
| `ARMING_CHECK` | **0 — all pre-arm checks disabled** |
| `ARMING_NEED_LOC` | 0 |
| `ARMING_RUDDER` | 2 |
| `DISARM_DELAY` | 0 |

SITL convenience. Must not be carried to a real airframe.

## 6. Navigation limits vs the fsm's sanity bounds

`config/vehicle_controller.yaml` holds the stack's limits; ArduPilot holds its
own. Whichever is lower binds.

| Quantity | ArduPilot | fsm limit | Binding |
|---|---|---|---|
| Horizontal speed | `WPNAV_SPEED` 1000 cm/s = 10 m/s | `max_velocity_mps` 5.0 | **fsm** |
| Acceleration | `WPNAV_ACCEL` 250 cm/s² = 2.5 m/s² | `max_accel_mps2` 3.0 | **ArduPilot** |
| Climb rate | `WPNAV_SPEED_UP` 250 cm/s = 2.5 m/s | — | ArduPilot |
| Descent rate | `WPNAV_SPEED_DN` 150 cm/s = 1.5 m/s | — | ArduPilot |
| Altitude | `FENCE_ALT_MAX` 100 m (fence disabled) | `goto_alt_max_m` 50.0 | **fsm** |

`max_accel_mps2 = 3.0` is above what ArduPilot will honour — the fsm will accept
an acceleration the vehicle then silently clips to 2.5 m/s².

Other relevant limits:

| Parameter | Value |
|---|---|
| `ANGLE_MAX` | 3000 (30°) |
| `PILOT_SPEED_UP` / `PILOT_SPEED_DN` / `PILOT_ACCEL_Z` | 250 / 0 / 250 |
| `PILOT_TKOFF_ALT` | 0.0 |
| `RTL_ALT` / `RTL_ALT_FINAL` / `RTL_LOIT_TIME` | 1500 (15 m) / 0 / 5000 |
| `LAND_SPEED` / `LAND_SPEED_HIGH` / `LAND_ALT_LOW` | 50 (0.5 m/s) / 0 / 1000 (10 m) |
| `WPNAV_RADIUS` | 200 (2 m) |

## 7. Position sources

| Parameter | Value |
|---|---|
| `AHRS_EKF_TYPE` | 3 (EKF3) |
| `EK3_SRC1_POSXY` / `VELXY` / `VELZ` | 3 (GPS) |
| `EK3_SRC1_POSZ` | 1 (Baro) |
| `EK3_SRC1_YAW` | 1 (Compass) |
| `GPS1_RATE_MS` | 200 (5 Hz) |
| `SIM_GPS1_HZ` | 5 |
| `SIM_GPS1_NUMSATS` / `SIM_GPS1_ACC` / `SIM_GPS1_LAG_MS` | 10 / 0.3 / 100 |

GPS updates at 5 Hz, which bounds the 3.16 Hz odom rate from below — odom cannot
be made much faster than 5 Hz without changing the GPS rate too.

## 8. Airframe and simulation

| Parameter | Value |
|---|---|
| `FRAME_CLASS` / `FRAME_TYPE` | 1 (Quad) / 1 (X) |
| `SCHED_LOOP_RATE` | 400 |
| `SIM_RATE_HZ` | 1200 |
| `SIM_SPEEDUP` | 1.0 |
| `SIM_OPOS_LAT` / `LNG` / `ALT` / `HDG` | -35.36326218 / 149.16523743 / 584.0 / 353.0 |
| `SIM_WIND_SPD` / `SIM_WIND_DIR` | 0.0 / 180.0 |
| `MOT_THST_HOVER` | 0.3388 |
| `SERIAL0_BAUD` / `PROTOCOL` | 115200 / MAVLink2 |
| `SERIAL1_BAUD`, `SERIAL2_BAUD` | 57600, MAVLink2 |

Default SITL origin is Canberra (CMAC).

## 9. Precision landing

| Parameter | Value |
|---|---|
| `PLND_ENABLED` | 1 |
| `PLND_TYPE` | 3 (SITL_Gazebo) |
| `PLND_EST_TYPE` | 1 (KalmanFilter) |
| `PLND_ALT_MAX` / `PLND_ALT_MIN` | 8.0 / 0.75 |
| `RC8_OPTION` | 39 (PrecLoiter enable) |

Enabled, but nothing in the current build uses it — `aruco_landing_node.py` is
commented out of `setup.py`'s entry points.

---

## Open items this raised

1. **Mode-change latency ~1.27 s** is the single most consequential number here,
   because axis 1 gates everything. Fixing it means changing what MAVROS
   requests, not an ArduPilot parameter.
2. **GUIDED is on no switch position**, so the stack depends on an external GCS
   to become commandable at all.
3. **`max_accel_mps2` (3.0) exceeds `WPNAV_ACCEL` (2.5)** — the fsm accepts values
   the vehicle will clip.
4. **`ARMING_CHECK = 0` and both battery failsafe actions are None** — acceptable
   in SITL, not on hardware.
