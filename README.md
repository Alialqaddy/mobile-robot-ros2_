<div align="center">

# Intelligent Mobile Robot — ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy%20%7C%20Iron-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com)
[![License](https://img.shields.io/badge/License-MIT-2ea44f?style=for-the-badge)](LICENSE)

**Hardware-independent ROS2 architecture for a differential-drive teach-and-repeat autonomous robot.**  
Fully validated in simulation — only two files change for real Raspberry Pi deployment.

[Bare-metal companion project →](https://github.com/Alialqaddy/intelligent-mobile-robot)

</div>

---

## What This Project Does

- Implements a complete **ROS2 node architecture** for a physical differential-drive robot
- Replicates the **teach-and-repeat navigation pipeline**: record a path → extract waypoints → follow autonomously
- Validates the full system **without any hardware** using mock nodes and simulated odometry
- Provides **live RViz2 visualization** of the planned path and the robot's live odometry trajectory
- Follows a **real recorded path** (`last_path.csv`) captured from an actual robot session with **4.2 cm end-to-end error**

---

## Media

---

### RViz2 — Live Path Following

<p align="center">
  <img src="docs/Screenshot 2026-05-20 054913.png" alt="RViz2 Path Visualization" width="700"/>
</p>

> Red line = planned waypoints from `last_path.csv` (real robot recording, 20 waypoints, 1.83 m)  
> Green line = live odometry trajectory drawn in real time  
> Position error at end of path: **4.2 cm ✓**

---

### RViz2 — Ultrasonic Sensor Cone

<p align="center">
  <img src="docs/Screenshot 2026-05-20 055714.png" alt="RViz2 with Ultrasonic Cone" width="700"/>
</p>

> Orange cone = simulated HC-SR04 ultrasonic sensor field of view  
> Cone shrinks when obstacle is detected (1.0 m → 0.25 m)

---

### Demo Video — RViz2 While Drawing


<p align="center">
  <a href="https://www.youtube.com/watch?v=ni19Xvr9r-Q">
      <img src="docs/Screenshot 2026-05-20 054805.png" alt="Watch Demo Video" width="700"/>
  </a>
</p>

---

### ROS2 Node Graph

<p align="center">
  <img src="docs/Screenshot 2026-05-20 060540.png" alt="ROS2 Node Graph" width="900"/>
</p>

---

### Terminal Output

<p align="center">
  <img src="docs/Screenshot 2026-05-20 053357.png" alt="Path Completion Terminal" width="800"/>
</p>

<p align="center">
  <img src="docs/Screenshot 2026-05-20 053342.png" alt="Motor Controller Terminal" width="800"/>
</p>

<p align="center">
  <img src="docs/Screenshot 2026-05-20 053326.png" alt="Encoder Odometry Terminal" width="800"/>
</p>

---

## Architecture

### Node Graph

```
┌──────────────────────────────┐   /cmd_vel (Twist)    ┌─────────────────────────────────┐
│   waypoint_follower_node     │ ─────────────────────► │   mock_motor_controller_node    │
│                              │                         │                                 │
│  • Loads waypoint CSV        │ ◄───────────────────── │  • Converts Twist → left/right  │
│  • Timer state machine       │   /odom (Odometry)     │    PWM (mock: prints to log)    │
│  • /follower/start service   │                         │  • Publishes /motor_state       │
│  • /follower/abort service   │ ◄───────────────────── │  • Publishes /motor_direction   │
└──────────┬───────────────────┘   /obstacle_alert       └──────────────┬──────────────────┘
           │                                                             │ /motor_state
           │                                                             ▼
┌──────────┴──────────────────┐                          ┌─────────────────────────────────┐
│   mock_ultrasonic_node      │                          │   encoder_odometry_node          │
│                             │                          │                                  │
│  • Configurable mode:       │                          │  • Simulates encoder ticks       │
│    always_clear             │                          │    from motor speed              │
│    inject_slow              │                          │  • Midpoint-rule integration     │
│    inject_stop              │                          │  • Publishes /odom               │
│  • 20 Hz timer              │                          │  • Publishes /odom_path (RViz2)  │
└─────────────────────────────┘                          └─────────────────────────────────┘
```

### Topics and Services

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `waypoint_follower_node` | `mock_motor_controller_node` |
| `/motor_state` | `Float32MultiArray` | `mock_motor_controller_node` | `encoder_odometry_node` |
| `/motor_direction` | `std_msgs/String` | `mock_motor_controller_node` | `encoder_odometry_node` |
| `/odom` | `nav_msgs/Odometry` | `encoder_odometry_node` | `waypoint_follower_node` |
| `/odom_path` | `nav_msgs/Path` | `encoder_odometry_node` | RViz2 |
| `/obstacle_alert` | `std_msgs/String` | `mock_ultrasonic_node` | `waypoint_follower_node` |
| `/planned_path` | `nav_msgs/Path` | `waypoint_follower_node` | RViz2 |
| `/follower/status` | `std_msgs/String` | `waypoint_follower_node` | monitoring |
| `/ultrasonic/range` | `sensor_msgs/Range` | `mock_ultrasonic_node` | monitoring |

| Service | Type | Server |
|---|---|---|
| `/follower/start` | `std_srvs/Trigger` | `waypoint_follower_node` |
| `/follower/abort` | `std_srvs/Trigger` | `waypoint_follower_node` |

### Waypoint Follower State Machine

```
                     /follower/start called
                            │
                     ┌──────▼──────┐
                     │    IDLE     │◄─── /follower/abort (from any state)
                     └──────┬──────┘
                            │ heading error > 5°?
              ┌─────────────┴─────────────┐
              │ YES                        │ NOh
              ▼                            ▼
       ┌─────────────┐             ┌─────────────┐
       │   TURNING   │────────────►│   DRIVING   │
       │             │ aligned     │             │
       └─────────────┘             └──────┬──────┘
              ▲                           │
              │       waypoint reached?   │
              │              ├── YES, more waypoints → back to TURNING
              │              └── YES, last waypoint ──────────────────►┌──────┐
              └──────────────────────────────────────────────────────  │ DONE │
                                                                        └──────┘
     During DRIVING:
       /obstacle_alert == STOP  →  publish Twist(0,0,0), stay in DRIVING
       /obstacle_alert == SLOW  →  reduce linear.x to SLOW_SPEED (0.25)
       /obstacle_alert == CLEAR →  resume normal DRIVE_SPEED (0.30)
```

### Odometry Integration

Midpoint-rule dead reckoning — identical to the bare-metal `path_extract.py`:

```python
dL = delta_left_ticks  × M_PER_TICK × sign(left_speed)   # M_PER_TICK = 0.014 m
dR = delta_right_ticks × M_PER_TICK × sign(right_speed)
ds     = (dL + dR) / 2
dtheta = (dR - dL) / BASELINE                             # BASELINE = 0.31 m

x     += ds × cos(theta + dtheta/2)                       # midpoint angle
y     += ds × sin(theta + dtheta/2)
theta  = wrap(theta + dtheta)                             # keep in [-π, +π]
```

Straight-segment suppression: if both wheels move in the same direction and `|left_speed - right_speed| < 0.12`, set `dtheta = 0` to reduce noise from minor encoder imbalance.

---

## Quick Start

### Requirements

- ROS2 Jazzy (Ubuntu 24.04) or ROS2 Iron (Ubuntu 22.04)
- Python 3.10+
- `colcon` build tool

### Install

```bash
# Create workspace
mkdir -p ~/imr_ws/src
cd ~/imr_ws/src

# Clone this repository as the package directory
git clone https://github.com/Alialqaddy/mobile-robot-ros2.git imr_ros2

# Build
cd ~/imr_ws
colcon build --packages-select imr_ros2
source install/setup.bash
```

### Run

Open four terminals. Source `~/imr_ws/install/setup.bash` in each.

```bash
# Terminal 1 — Motor controller (mock: prints PWM, no GPIO)
ros2 run imr_ros2 mock_motor_controller_node

# Terminal 2 — Encoder odometry
ros2 run imr_ros2 encoder_odometry_node

# Terminal 3 — Waypoint follower (loads real recorded path)
ros2 run imr_ros2 waypoint_follower_node \
  --ros-args -p path_file:=$HOME/imr_ws/src/imr_ros2/paths/last_path.csv

# Terminal 4 — Start autonomous following
ros2 service call /follower/start std_srvs/srv/Trigger "{}"
```

### Optional — Obstacle Simulation

```bash
# Inject a STOP obstacle at t=5s, clear at t=8s
ros2 run imr_ros2 mock_ultrasonic_node \
  --ros-args -p mode:=inject_stop \
             -p inject_after_sec:=5.0 \
             -p clear_after_sec:=3.0
```

### Optional — RViz2 Visualization

```bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** → `odom`
2. **Add** → By topic → `/planned_path` → Path → color **red** (255, 0, 0)
3. **Add** → By topic → `/odom_path` → Path → color **green** (0, 255, 0)
4. Call `/follower/start` — red line appears instantly, green line grows in real time

---

## Validation Results

All tests run on WSL2 (Ubuntu 24.04, ROS2 Jazzy) without any physical hardware.

| Test | Result |
|---|---|
| Package build (colcon) | ✓ Clean build, 0 errors |
| Node startup (all 4 nodes) | ✓ No crashes |
| Topic graph connectivity | ✓ All topics bridged correctly |
| `/cmd_vel` → direction classification | ✓ FORWARD / SPIN_LEFT / SPIN_RIGHT / STOP correct |
| Odometry — forward motion | ✓ x increases, y = 0, θ = 0° |
| Odometry — in-place spin | ✓ x/y frozen, θ sweeps correctly |
| Square path (0.5 m × 4 segments) | ✓ End-to-end error: 4.9 cm |
| Real recorded path (20 waypoints, 1.83 m) | ✓ End-to-end error: **4.2 cm** |
| Obstacle STOP mid-segment | ✓ Robot pauses, resumes on CLEAR — non-blocking |
| Service guard (duplicate `/follower/start`) | ✓ Rejected with correct message |
| RViz2 visualization | ✓ Red planned + green live trajectory overlapping |
| Latched `/planned_path` | ✓ Appears in RViz2 even when subscribed after publish |

---

## Hardware Transition

When the physical Raspberry Pi robot is available, **only two files change**.  
All nodes, topics, services, and state machines are hardware-ready as-is.

| File | Current (mock) | Replace with (hardware) |
|---|---|---|
| `mock_motor_controller_node.py` | Computes PWM, logs to terminal | Writes to L298N H-bridge via `gpiozero` GPIO pins |
| `encoder_odometry_node.py` | Simulates ticks from `/motor_state` | Reads rising-edge interrupts on GPIO 5 (left) and GPIO 25 (right) |

The `waypoint_follower_node`, `mock_ultrasonic_node`, all services, all topics, and all odometry math carry directly to hardware without modification.

---

## Hardware Specifications

| Component | Detail |
|---|---|
| Main controller | Raspberry Pi 4B |
| Drive system | 4× DC motors, differential drive |
| Motor driver | L298N H-bridge (IN1=GPIO17, IN2=GPIO27, ENA=GPIO12; IN3=GPIO22, IN4=GPIO23, ENB=GPIO13) |
| Wheel encoders | Optical encoders — Left: GPIO 5, Right: GPIO 25 (rising edge, bounce_time=1 ms) |
| Obstacle sensor | HC-SR04 ultrasonic — TRIG=GPIO24, ECHO=GPIO26 (3.3V via voltage divider) |
| Camera | Raspberry Pi Camera Module v2 (CSI interface, `picamera2`) |
| Wheel separation (BASELINE) | 0.31 m |
| Distance per encoder tick (M_PER_TICK) | 0.014 m |
| Tick rate at full speed | ~42 ticks/s per wheel at PWM = 1.0 |

---

## Project Structure

```
mobile-robot-ros2/
├── imr_ros2/
│   ├── mock_motor_controller_node.py   # /cmd_vel subscriber, PWM calculation, /motor_state publisher
│   ├── encoder_odometry_node.py        # Tick simulation, midpoint-rule odometry, /odom publisher
│   ├── waypoint_follower_node.py       # Timer state machine, CSV loader, /cmd_vel publisher
│   └── mock_ultrasonic_node.py         # Configurable obstacle injector, /obstacle_alert publisher
├── paths/
│   └── last_path.csv                   # 20 waypoints from a real robot recording session
├── docs/                               # Screenshots and demo media (add yours here)
├── package.xml                         # ROS2 package manifest
├── setup.py                            # Entry points for all nodes
└── setup.cfg                           # Ament install configuration
```

---

## Related Project

This repository is the ROS2 layer of a two-part project.

| Repository | Description |
|---|---|
| **[mobile-robot-ros2](https://github.com/Alialqaddy/mobile-robot-ros2)** ← you are here | ROS2 node architecture, hardware-independent mock layer |
| **[intelligent-mobile-robot](https://github.com/Alialqaddy/intelligent-mobile-robot)** | Original bare-metal Python implementation on Raspberry Pi 4B |

---

## License

MIT License — see [LICENSE](LICENSE) for details.
