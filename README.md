# Autonomous Obstacle Avoidance Robot

An autonomous robot simulation using sensor-based reactive AI for real-time obstacle avoidance. Built with Webots simulator and the e-puck robot platform, controlled by a Python script implementing proportional control inspired by Braitenberg vehicles.

## Features

- Reactive AI with no pre-programmed paths or maps
- Proportional speed control for smooth curved avoidance paths
- Five-situation classification (CLEAR, FRONT_BLOCKED, LEFT_BLOCKED, RIGHT_BLOCKED, NARROW_PASSAGE)
- Stuck detection with automatic escape manoeuvre
- Speed smoothing via first-order low-pass filter
- Turn commitment to prevent oscillation
- Real-time path trail visualisation using Supervisor API

## Requirements

- [Webots R2023b](https://cyberbotics.com/) (or later)
- Python 3.x (bundled with Webots)

## Setup & Running

1. Open Webots
2. File → Open World → select `worlds/obstacle_avoidance.wbt`
3. Press Play to start the simulation
4. Observe the robot navigating the arena in the 3D viewport
5. Monitor console output for sensor readings and decisions

## Project Structure

```
├── controllers/
│   └── obstacle_avoidance/
│       └── obstacle_avoidance.py       # Main controller script
├── worlds/
│   └── obstacle_avoidance.wbt          # Webots world definition
├── Pictures/
│   ├── robot_system_architecture_Extra.png
│   └── robot_sensor_layout_light.png
├── Test Results/                        # Screenshots and console logs
├── Project_Report.tex                   # LaTeX report source
├── Project_Report.pdf                   # Compiled report
└── README.md
```

## Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `SENSOR_THRESHOLD` | 80.0 | Detection threshold (must exceed noise floor of ~76) |
| `BASE_SPEED` | 3.14 rad/s | Normal cruising speed (50% of max) |
| `MAX_SPEED` | 6.28 rad/s | Maximum motor velocity |
| `STUCK_THRESHOLD` | 30 steps | Time before escape triggers (~2 seconds) |
| `SMOOTHING_FACTOR` | 0.7 | Speed smoothing (0=instant, 1=no change) |

## Results

- **100% collision avoidance** across all six test scenarios
- **320 seconds** of sustained autonomous operation
- **64 ms** detection-to-response latency
- Smooth curved navigation paths confirmed by trail visualisation

![Full Arena Navigation](Test%20Results/obstacle_avoidance_test_result5.png)

## References

- Braitenberg, V. (1984). *Vehicles: Experiments in Synthetic Psychology*. MIT Press.
- Brooks, R.A. (1986). A robust layered control system for a mobile robot. *IEEE Journal on Robotics and Automation*, 2(1).
- Cyberbotics Ltd. (2023). *Webots User Guide*. https://cyberbotics.com/doc/guide/index
- Mondada, F. et al. (2009). The e-puck, a robot designed for education in engineering.
