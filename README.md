# ArduPilot Autonomy

A Python ROS 2 package for autonomous drone control using ArduPilot and MAVROS.

## Overview

This package provides a clean, service-based interface for controlling ArduPilot drones through MAVROS. It implements position, velocity, and acceleration control modes with state machine validation, following design patterns from production autonomy systems.

# px4_autonomy by Useful Dynamics
[🚀 Visit Website](https://usefuldynamics.io)

### Repository Traffic

![Visitor Count](https://visitor-badge.laobi.icu/badge?page_id=rick-dn.px4-autonomy&color=blue) ![Views and Clones](https://raw.githubusercontent.com/rick-dn/px4-autonomy/github-repo-stats/views_clones_graph.png)

## Update: Autonomous Flight Sequences

Added automated flight sequences for takeoff and coverage scanning. Actions, FSM state validation, and thread safety (semaphores/locks) are planned for next iteration.

**Run sequences:**
```bash
# Automated takeoff to 5m
ros2 run ardupilot_autonomy takeoff_sequence

# takeoff to custom altitude
ros2 run ardupilot_autonomy takeoff_sequence --ros-args -p takeoff_altitude:=10.0

# Spiral coverage scan (run after takeoff)
ros2 run ardupilot_autonomy scan_sequence

# Configure scan parameters
ros2 run ardupilot_autonomy scan_sequence --ros-args \
  -p scan_radius:=10.0 \
  -p waypoint_spacing:=2.5 \
  -p spiral_turns:=3
```

## Update: Yaw & Position Validation (March 2026)

Validated NED yaw and position commands on physical hardware. Spiral scan worked in SITL but was unreliable physically due to `WPNAV_RADIUS` default (200cm) causing waypoint acceptance before movement. Fixed by setting `WPNAV_RADIUS=50`. Tested directional flight (N/E/S/W 10m) with yaw aligned to direction of travel — confirmed working on physical hardware.

<<<<<<< HEAD
---
=======
>>>>>>> 059e8afc4ff68945ac827a1d54279b6901f406bb

## Features

- **Position Control**: GPS-based waypoint navigation with yaw control
- **Velocity Control**: Body-frame velocity commands for dynamic maneuvers
- **Acceleration Control**: NED-frame acceleration for agile flight
- **State Machine**: FSM-based validation to prevent unsafe operations
- **Fire-and-Forget**: Asynchronous command pattern for responsive control

## Installation
```bash
cd ~/ardu_ws/src
git clone <your-repo>
cd ~/ardu_ws
colcon build --packages-select ardupilot_autonomy
source install/setup.bash
```

## Dependencies

- ROS 2 Humble
- ArduPilot SITL or real hardware
- MAVROS
- geographiclib (Python): `pip install geographiclib`

## Usage

### Start MAVROS
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14555
```

### Start Vehicle Interface
```bash
ros2 run ardupilot_autonomy vehicle_interface
```

### Basic Flight Sequence

**1. Set GUIDED mode and arm:**
```bash
ros2 service call /vehicle/set_guided_mode std_srvs/srv/Trigger
ros2 service call /vehicle/arm std_srvs/srv/Trigger
```

**2. Takeoff:**
```bash
ros2 param set /vehicle_interface takeoff_altitude 10.0
ros2 service call /vehicle/takeoff std_srvs/srv/Trigger
```

**3. Navigate to position (North/East offsets from home):**
```bash
ros2 param set /vehicle_interface goto_north 50.0
ros2 param set /vehicle_interface goto_east 30.0
ros2 param set /vehicle_interface goto_up 10.0
ros2 param set /vehicle_interface goto_yaw 45.0
ros2 service call /vehicle/goto_position std_srvs/srv/Trigger
```

**4. Velocity control:**
```bash
ros2 param set /vehicle_interface vel_x 1.0    # Forward 1 m/s
ros2 param set /vehicle_interface vel_y 0.0
ros2 param set /vehicle_interface vel_z 0.0
ros2 param set /vehicle_interface vel_yaw_rate 0.0
ros2 service call /vehicle/velocity_start std_srvs/srv/Trigger

# Stop velocity
ros2 service call /vehicle/velocity_stop std_srvs/srv/Trigger
```

**5. Land or RTL:**
```bash
ros2 service call /vehicle/land std_srvs/srv/Trigger
# OR
ros2 service call /vehicle/rtl std_srvs/srv/Trigger
```

## Available Services

| Service | Description |
|---------|-------------|
| `/vehicle/set_guided_mode` | Switch to GUIDED mode |
| `/vehicle/arm` | Arm motors |
| `/vehicle/disarm` | Disarm motors |
| `/vehicle/takeoff` | Takeoff to specified altitude |
| `/vehicle/goto_position` | Navigate to GPS position (using N/E offsets) |
| `/vehicle/goto_neu` | Navigate to local NEU position |
| `/vehicle/velocity_start` | Start velocity control |
| `/vehicle/velocity_stop` | Stop velocity control |
| `/vehicle/accel_start` | Start acceleration control |
| `/vehicle/accel_stop` | Stop acceleration control |
| `/vehicle/land` | Land at current position |
| `/vehicle/rtl` | Return to launch |

## Architecture
```
ardupilot_autonomy/
├── mavros_interface.py    # MAVROS communication wrapper
├── vehicle_interface.py   # Main orchestrator with services
└── state_machine.py       # FSM validation logic
```

## Future Development

- **Robust State Machine**: Expand to 20+ states with full transition graph validation
- **ROS 2 Actions**: Implement action servers with real-time feedback, monitoring loops, and cancellation support for takeoff, land, goto, and orbit operations
- **Thread Safety**: Add semaphores and locks for concurrent action handling

## License

This project is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.

**You are free to:**
- Share and adapt the code for non-commercial purposes

**Under the following terms:**
- Attribution required
- Non-commercial use only
- Share derivatives under the same license

For commercial licensing inquiries, contact: your.email@example.com

[Full License Text](https://creativecommons.org/licenses/by-nc-sa/4.0/)
