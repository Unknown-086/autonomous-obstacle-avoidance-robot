# Autonomous Obstacle Avoidance Robot Using Sensor-Based AI

## A Webots Simulation with E-puck Robot and Python Controller

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [System Design](#2-system-design)
3. [Implementation](#3-implementation)
4. [Testing and Evaluation](#4-testing-and-evaluation)
5. [Results and Discussion](#5-results-and-discussion)
6. [Conclusion](#6-conclusion)
7. [References](#7-references)
8. [Appendices](#8-appendices)

---

## 1. Introduction

### 1.1 Project Aim

The aim of this project is to develop and simulate an autonomous robot capable of navigating a cluttered environment while detecting and avoiding obstacles in real time. The robot employs a reactive artificial intelligence approach, making decisions exclusively based on live sensor data rather than pre-programmed paths or map-based planning algorithms.

### 1.2 Objectives

The specific objectives of this project are:

1. To design a simulation environment containing a walled arena with multiple static obstacles of varying size and position.
2. To implement a Python-based controller that reads infrared proximity sensor data and translates it into appropriate motor commands.
3. To develop a reactive decision-making algorithm that classifies navigation situations and computes proportional motor responses for smooth, collision-free movement.
4. To incorporate edge-case handling, including stuck detection and escape manoeuvres, ensuring robust long-duration operation.
5. To evaluate the system's performance through structured test scenarios measuring collision avoidance accuracy, response time, and navigation smoothness.

### 1.3 Background and Motivation

Obstacle avoidance is a fundamental problem in mobile robotics and represents one of the earliest demonstrations of intelligent behaviour in autonomous systems. The reactive approach adopted in this project draws inspiration from Braitenberg vehicles (Braitenberg, 1984), where simple sensor-motor connections produce complex emergent behaviours without explicit planning.

The project utilises the Webots robotics simulator (Cyberbotics, 2023) with the e-puck educational robot platform. Webots provides a physically accurate simulation environment with realistic sensor models, enabling the development and testing of control algorithms without requiring physical hardware. The e-puck robot, originally developed at EPFL, is widely used in robotics education due to its well-documented sensor array and differential drive locomotion system.

### 1.4 Scope

This project is limited to:
- A two-dimensional navigation problem on a flat surface
- Static obstacles only (no moving objects)
- Reactive control without memory or learning
- Simulation-based evaluation using Webots

---

## 2. System Design

### 2.1 System Architecture Overview

The system follows a classic three-layer reactive architecture consisting of an Input Layer (sensors), a Processing Layer (AI logic), and an Output Layer (actuators). This architecture enables real-time responsiveness as each control cycle reads sensors, processes decisions, and outputs motor commands within a single simulation timestep.

### 2.2 System Architecture Diagram

```
+------------------------------------------------------------------+
|                     MAIN CONTROL LOOP                              |
|                  (executes every 64ms)                             |
+------------------------------------------------------------------+
         |                    |                      |
         v                    v                      v
+----------------+   +-------------------+   +------------------+
|  INPUT LAYER   |   | PROCESSING LAYER  |   |  OUTPUT LAYER    |
|  (Sensors)     |   | (AI Logic)        |   |  (Actuators)     |
+----------------+   +-------------------+   +------------------+
| 8 IR Proximity |   | Situation         |   | Left Wheel Motor |
| Sensors        |-->| Classification    |-->| Right Wheel Motor|
| (ps0 - ps7)   |   |                   |   |                  |
|                |   | Proportional      |   | Velocity Control |
| Raw values:    |   | Speed Computation |   | Range: -6.28 to  |
| 0 - 1000      |   |                   |   | +6.28 rad/s      |
|                |   | Speed Smoothing   |   |                  |
|                |   |                   |   |                  |
|                |   | Stuck Detection   |   |                  |
+----------------+   +-------------------+   +------------------+
```

### 2.3 Input Layer (Sensors)

The e-puck robot is equipped with eight infrared (IR) proximity sensors distributed around its circumference. These sensors emit infrared light and measure the intensity of the reflection, providing distance estimates to nearby objects. The sensor arrangement provides 360-degree coverage with higher resolution at the front of the robot where obstacle detection is most critical.

**Sensor Layout (top view, robot facing forward):**

```
        Front
    ps7         ps0
   (front-left)  (front-right)

  ps6              ps1
 (left-front)    (right-front)

  ps5              ps2
  (left)          (right)

    ps4         ps3
  (rear-left)  (rear-right)
        Rear
```

**Sensor Characteristics:**
- Output range: 0 (no obstacle) to approximately 1000 (obstacle touching)
- Ambient noise level: approximately 60-76 (observed baseline with no obstacles)
- Detection range: approximately 0 to 8 cm
- Update rate: synchronised with simulation timestep (64 ms)

### 2.4 Processing Layer (AI Logic)

The processing layer implements a hybrid rule-based and proportional control algorithm. It operates in two stages:

1. **Situation Classification:** Sensor readings are compared against a configurable threshold to determine the current navigation context (clear path, front obstacle, side obstacle, or enclosed space).

2. **Proportional Speed Computation:** Rather than applying binary on/off responses, the motor speeds are computed proportionally to the obstacle proximity. This produces smooth, gradual avoidance manoeuvres rather than jerky reactions.

Additional processing includes:
- **Speed smoothing:** A low-pass filter prevents abrupt speed changes between consecutive timesteps.
- **Turn commitment:** Once a turn direction is chosen, it is maintained for a minimum number of steps to prevent oscillation.
- **Stuck detection:** A counter monitors prolonged blocked states and triggers an escape manoeuvre if normal avoidance logic cannot resolve the situation.

### 2.5 Output Layer (Actuators)

The e-puck uses a differential drive system with two independently controlled wheel motors. By varying the speed ratio between the left and right wheels, the robot can execute forward motion, in-place rotation, and curved turns of any radius:

| Left Speed | Right Speed | Resulting Motion |
|-----------|-------------|-----------------|
| Equal positive | Equal positive | Straight forward |
| Positive | Negative | Turn right (on the spot) |
| Negative | Positive | Turn left (on the spot) |
| Higher left | Lower right | Curve to the right |
| Lower left | Higher right | Curve to the left |
| Equal negative | Equal negative | Straight reverse |

---

## 3. Implementation

### 3.1 Development Environment

| Component | Specification |
|-----------|--------------|
| Simulator | Webots R2023b (Cyberbotics) |
| Robot Platform | E-puck (version 1) |
| Programming Language | Python 3 |
| API | Webots Supervisor API (`controller` module) |
| Simulation Timestep | 64 milliseconds |

### 3.2 Project File Structure

```
Autonomous Robotic AI/
├── worlds/
│   └── obstacle_avoidance.wbt       # Simulation world definition
├── controllers/
│   └── obstacle_avoidance/
│       └── obstacle_avoidance.py    # Robot controller script
├── Test Results/                     # Screenshots and console logs
├── Assignment Proposal.txt
└── Reference Plan.txt
```

Webots requires a specific naming convention: the controller folder name, the Python script name, and the controller field in the world file must all match exactly (`obstacle_avoidance`).

### 3.3 World File Implementation

The simulation environment is defined in `obstacle_avoidance.wbt` using Webots' VRML-based world description language. The world consists of the following elements:

**Arena Configuration:**

```vrml
RectangleArena {
  floorSize 1.5 1.5
  floorTileSize 0.25 0.25
  wallHeight 0.05
}
```

The arena is a 1.5 m x 1.5 m square enclosure with 0.05 m high walls. The floor uses a 0.25 m checkered tile pattern for visual reference. The wall height is sufficient for the e-puck's IR sensors to detect (sensor height is approximately 0.03 m from ground level).

**Obstacle Placement:**

Seven static box obstacles are placed throughout the arena with varying sizes and colours:

| Obstacle | Position (x, y) | Dimensions | Colour |
|----------|-----------------|------------|--------|
| Box 1 | (0.4, 0.3) | 0.10 x 0.10 m | Red |
| Box 2 | (-0.3, 0.5) | 0.12 x 0.08 m | Blue |
| Box 3 | (0.0, -0.4) | 0.15 x 0.10 m | Green |
| Box 4 | (-0.5, -0.2) | 0.10 x 0.12 m | Yellow |
| Box 5 | (0.5, -0.5) | 0.08 x 0.15 m | Orange |
| Box 6 | (0.2, 0.0) | 0.10 x 0.10 m | Purple |
| Box 7 | (0.3, -0.15) | 0.12 x 0.08 m | Cyan |

Each obstacle is defined as a `Solid` node with a visual `Shape` (for rendering) and a `boundingObject` (for collision detection). For example:

```vrml
DEF BOX1 Solid {
  translation 0.4 0.3 0.025
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.8 0.1 0.1
        roughness 0.5
        metalness 0
      }
      geometry Box {
        size 0.1 0.1 0.05
      }
    }
  ]
  name "obstacle1"
  boundingObject Box {
    size 0.1 0.1 0.05
  }
}
```

**Design Constraints Applied:**
- No obstacle is placed within 0.3 m of the robot's starting position
- No obstacle is placed within 0.15 m of any wall
- Minimum gap between any two obstacles exceeds the e-puck's diameter (0.074 m)

**Robot Placement:**

```vrml
DEF EPUCK E-puck {
  translation -0.5 0.5 0
  rotation 0 0 1 -0.785
  name "e-puck"
  controller "obstacle_avoidance"
  supervisor TRUE
}
```

The robot starts in the upper-left region of the arena, rotated approximately 45 degrees to face toward the centre. The `supervisor TRUE` field enables the Supervisor API, which is used for the path trail visualisation feature. This provides a clear initial path of at least 0.5 m before encountering any obstacle, allowing sensors to stabilise before the first avoidance decision.

### 3.4 Controller Implementation

#### 3.4.1 Configuration Parameters

The controller exposes several tunable parameters at the top of the file:

```python
MAX_SPEED = 6.28          # Maximum motor velocity in rad/s (e-puck limit)
BASE_SPEED = 3.14         # Normal cruising speed (50% of max)
SENSOR_THRESHOLD = 80.0   # Proximity value above which an obstacle is detected
LOG_INTERVAL = 50         # Print sensor data every N timesteps (~3.2s at 64ms)
ENABLE_LOGGING = True     # Set to False for clean video recording
STUCK_THRESHOLD = 30      # Steps before triggering escape manoeuvre (~2 seconds)
SMOOTHING_FACTOR = 0.7    # Speed transition smoothing (0=instant, 1=no change)
TURN_COMMITMENT_STEPS = 15  # Minimum steps to commit to a turn direction
```

These parameters allow systematic experimentation with detection sensitivity, speed, and responsiveness without modifying the core algorithm.

#### 3.4.2 Robot Initialisation

```python
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Initialise wheel motors for velocity control
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialise proximity sensors (ps0 to ps7)
sensors = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    sensors.append(sensor)
```

The `Supervisor` class extends `Robot` with additional capabilities for modifying the simulation world at runtime, used here for the trail visualisation. The motors are set to velocity control mode by assigning an infinite target position (`float('inf')`), which allows direct velocity commands. Each proximity sensor is enabled with the simulation timestep to ensure synchronised readings.

#### 3.4.3 Situation Classification Algorithm

The classification function groups the eight sensor readings into logical zones and determines the current navigation situation:

```python
def classify_situation(sensor_values):
    front_left = sensor_values[7]
    front_right = sensor_values[0]
    diag_left = sensor_values[6]
    diag_right = sensor_values[1]
    side_left = sensor_values[5]
    side_right = sensor_values[2]

    front_blocked = (front_left > SENSOR_THRESHOLD or front_right > SENSOR_THRESHOLD
                     or diag_left > SENSOR_THRESHOLD or diag_right > SENSOR_THRESHOLD)
    left_blocked = (side_left > SENSOR_THRESHOLD or diag_left > SENSOR_THRESHOLD)
    right_blocked = (side_right > SENSOR_THRESHOLD or diag_right > SENSOR_THRESHOLD)

    if front_blocked and left_blocked and right_blocked:
        return "NARROW_PASSAGE"
    elif front_blocked:
        return "FRONT_BLOCKED"
    elif left_blocked and not right_blocked:
        return "LEFT_BLOCKED"
    elif right_blocked and not left_blocked:
        return "RIGHT_BLOCKED"
    elif left_blocked and right_blocked:
        return "NARROW_PASSAGE"
    else:
        return "CLEAR"
```

The classification priority ensures that the most constrained situation (NARROW_PASSAGE) is identified first, preventing inappropriate responses when obstacles surround the robot on multiple sides simultaneously.

#### 3.4.4 Proportional Speed Computation

Rather than applying fixed turn speeds, the controller computes motor velocities proportionally based on obstacle proximity:

```python
# For LEFT_BLOCKED situation:
intensity = max(normalised[5], normalised[6])
left_speed = BASE_SPEED
right_speed = BASE_SPEED * (1.0 - 1.5 * intensity)
```

This produces a continuous spectrum of responses: a distant obstacle causes a gentle curve, while a close obstacle produces a sharp turn. The 1.5 multiplier ensures adequate avoidance force even for moderate readings.

For FRONT_BLOCKED situations, the algorithm compares the total sensor readings on each side and turns toward the less-obstructed direction:

```python
left_sum = normalised[5] + normalised[6] + normalised[7]
right_sum = normalised[0] + normalised[1] + normalised[2]

if left_sum <= right_sum:
    committed_turn_direction = "left"
    left_speed = -0.3 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
else:
    committed_turn_direction = "right"
    left_speed = 0.5 * MAX_SPEED
    right_speed = -0.3 * MAX_SPEED
```

#### 3.4.5 Speed Smoothing

A first-order low-pass filter smooths speed transitions between consecutive control cycles:

```python
def smooth_speed(current, previous):
    return SMOOTHING_FACTOR * previous + (1.0 - SMOOTHING_FACTOR) * current
```

With a smoothing factor of 0.7, the motor speed retains 70% of its previous value and adopts 30% of the newly computed value. This eliminates mechanical jerkiness and produces motion that appears more natural and deliberate.

#### 3.4.6 Turn Commitment Mechanism

To prevent oscillation when the robot is equidistant from obstacles on both sides, a commitment counter ensures that once a turn direction is selected, it is maintained for at least 15 timesteps (approximately 1 second):

```python
if turn_commitment_counter > 0:
    turn_commitment_counter -= 1
    # Continue previously committed direction
else:
    # Choose new direction and set counter
    turn_commitment_counter = TURN_COMMITMENT_STEPS
```

This mechanism is particularly important when the robot approaches a wall at a shallow angle, where sensor readings may alternate between left and right dominance on consecutive steps.

#### 3.4.7 Stuck Detection and Escape Manoeuvre

In rare cases where the robot becomes trapped (e.g., in a concave corner where proportional control produces insufficient turning), the stuck detection system activates:

```python
if situation in ("FRONT_BLOCKED", "NARROW_PASSAGE"):
    stuck_counter += 1
else:
    stuck_counter = 0

if stuck_counter >= STUCK_THRESHOLD:
    escape_phase = "reverse"
    escape_counter = 5
```

The escape manoeuvre consists of two phases:
1. **Reverse phase (5 steps):** The robot backs away from the obstacle at half maximum speed.
2. **Spin phase (10-20 steps, random duration):** The robot rotates in a randomly chosen direction to reorient itself away from the obstruction.

The random elements prevent repetitive escape patterns that could lead to recurring stuck states in symmetric environments.

#### 3.4.8 Approach Modulation

Even when the situation is classified as CLEAR, the robot gradually reduces speed as front sensor values increase (below the detection threshold):

```python
front_max = max(normalised[0], normalised[7], normalised[1], normalised[6])
approach_factor = 1.0 - 0.4 * front_max
left_speed = BASE_SPEED * max(approach_factor, 0.6)
right_speed = BASE_SPEED * max(approach_factor, 0.6)
```

This provides a "cautious approach" behaviour where the robot decelerates before an obstacle formally triggers an avoidance response, resulting in smoother overall navigation.

#### 3.4.9 Path Trail Visualisation

Using the Supervisor API, the controller dynamically places small red spheres at the robot's position every 10 simulation steps, creating a visible trail of the robot's path:

```python
trail_root = robot.getRoot()
trail_children = trail_root.getField("children")

# Inside main loop:
if trail_counter % TRAIL_INTERVAL == 0:
    robot_node = robot.getSelf()
    pos = robot_node.getPosition()
    trail_children.importMFNodeFromString(-1, f'''
    DEF TRAIL Solid {{
      translation {pos[0]} {pos[1]} 0.001
      children [
        Shape {{
          appearance PBRAppearance {{ baseColor 1 0 0 }}
          geometry Sphere {{ radius 0.008 }}
        }}
      ]
    }}
    ''')
```

This feature enables visual verification of the robot's navigation path and is particularly useful for evaluating coverage patterns and identifying areas where the robot struggles.

### 3.5 Main Control Loop

The complete control loop executes the following sequence every 64 milliseconds:

1. Read all 8 sensor values
2. Place trail marker (every 10 steps)
3. Check for active escape manoeuvre (if active, execute escape and skip normal logic)
4. Classify current situation based on sensor readings
5. Update stuck counter (increment if blocked, reset if clear)
6. Trigger escape if stuck threshold exceeded
7. Compute proportional motor speeds based on situation
8. Apply speed smoothing filter
9. Set motor velocities
10. Log status at configured interval

The `robot.step(timestep)` call advances the simulation by one timestep and returns -1 when the simulation terminates, providing a clean exit condition.

---

## 4. Testing and Evaluation

### 4.1 Testing Methodology

The system was evaluated through six structured test scenarios, each designed to assess a specific aspect of the robot's navigation capability. Tests were run with the default threshold of 80.0 (except Test T6 which varied the threshold). The path trail feature provided visual confirmation of the robot's trajectory in each test. Results were assessed against predefined pass criteria using both console log analysis and visual inspection.

### 4.2 Test Scenarios

#### Test T1: Open Arena Navigation

| Parameter | Detail |
|-----------|--------|
| **Setup** | All 7 obstacles removed from the world file; robot placed at start position with threshold = 80 |
| **Purpose** | Verify that the robot maintains a straight trajectory when no obstacles are present and handles wall encounters correctly |
| **Pass Criteria** | Robot travels in a straight line without oscillation until reaching a wall, then executes a clean turn |
| **Expected Result** | Constant BASE_SPEED on both motors; situation remains CLEAR until wall approach |
| **Actual Result** | The robot maintained a steady forward speed (L=3.05, R=3.05 rad/s) in CLEAR state for the first 400 steps (25.6 seconds). All sensor readings remained in the 60-76 range (ambient noise). At step 450, the robot reached a corner where multiple walls triggered NARROW_PASSAGE (sensors: 203, 323, 104, 114, 340, 199). The escape manoeuvre activated at step 456, completed by step 471. After escaping, the robot navigated along walls using FRONT_BLOCKED and LEFT_BLOCKED classifications, turning smoothly at each wall. The trail showed clean straight lines with curved turns at arena boundaries. Zero collisions throughout the run of 1900 steps (121.6 seconds). |

#### Test T2: Single Wall Approach (with obstacles)

| Parameter | Detail |
|-----------|--------|
| **Setup** | Full arena with all 7 obstacles; robot at default start position; threshold = 80 |
| **Purpose** | Verify obstacle detection and avoidance during normal navigation with multiple obstacles present |
| **Pass Criteria** | Robot detects walls and obstacles, turns without collision, maintains smooth navigation |
| **Expected Result** | Mix of CLEAR, FRONT_BLOCKED, and LEFT/RIGHT_BLOCKED situations resolved without collision |
| **Actual Result** | The robot demonstrated smooth navigation for 2150 steps (137.6 seconds). It operated primarily in CLEAR state with periodic detections: LEFT_BLOCKED at step 350 (sensor ps5 = 83), FRONT_BLOCKED at step 750 (ps6 = 107), and further wall/obstacle encounters resolved by proportional turning. Notably, no escape manoeuvres were triggered — all avoidance was handled by the proportional control and turn commitment systems alone. The trail path showed the robot navigating around obstacles with smooth curved avoidance paths rather than sharp angular turns. Zero collisions recorded. |

#### Test T3: Corner Trap

| Parameter | Detail |
|-----------|--------|
| **Setup** | Robot navigated into a 90-degree corner during Test 1 run; threshold = 80 |
| **Purpose** | Verify escape from the most challenging geometric scenario |
| **Pass Criteria** | Robot escapes the corner without collision, using the escape manoeuvre if necessary |
| **Expected Result** | NARROW_PASSAGE classification followed by stuck-triggered escape manoeuvre |
| **Actual Result** | At step 450, the robot entered a corner with very high sensor readings on multiple sides (NARROW_PASSAGE: ps0=203, ps1=323, ps2=104, ps5=114, ps6=340, ps7=199). The stuck counter reached threshold at step 456, triggering the first escape manoeuvre. After the first escape (completed step 471), the robot found itself still partially trapped with even higher readings (ps0=1783, ps1=1902, ps5=610, ps6=2050), triggering a second escape at step 501. A third escape occurred at step 552 before the robot successfully cleared the corner by step 650, resuming normal LEFT_BLOCKED navigation. The trail in the corner area shows the characteristic reverse-and-spin pattern of the escape manoeuvre. Despite three consecutive escape events, the robot never collided with the walls. |

#### Test T4: Narrow Gap Navigation

| Parameter | Detail |
|-----------|--------|
| **Setup** | Robot encountered narrow gaps between obstacles during Test 2/5 runs; threshold = 80 |
| **Purpose** | Verify behaviour in constrained passages between obstacles |
| **Pass Criteria** | Robot either navigates through the gap cleanly or turns away gracefully |
| **Expected Result** | NARROW_PASSAGE classification with reduced speed or graceful avoidance |
| **Actual Result** | During the full arena tests, the robot encountered narrow passages between the purple/cyan obstacles (Box 6 and Box 7, positioned at (0.2, 0.0) and (0.3, -0.15) respectively). The trail visualisation shows the robot detecting obstacles on both sides simultaneously and either threading through at reduced speed or choosing to turn away. In most encounters, the proportional control was sufficient to navigate between obstacles without triggering escape manoeuvres. The close-up trail pattern between obstacles shows controlled curved paths maintaining clearance from both sides. Zero collisions in constrained areas. |

#### Test T5: Full Arena Endurance

| Parameter | Detail |
|-----------|--------|
| **Setup** | Complete environment with all 7 obstacles and arena walls; robot at default start position; threshold = 80 |
| **Purpose** | Verify sustained collision-free navigation over an extended period |
| **Pass Criteria** | Robot navigates for at least 120 seconds with zero collisions and no permanent stuck states |
| **Expected Result** | Continuous autonomous navigation with varied situations classified throughout the run |
| **Actual Result** | The robot navigated successfully for 5000 steps (320 seconds / 5.3 minutes) with zero collisions. Situation distribution across the run: predominantly CLEAR (~70%), LEFT_BLOCKED (~15%), FRONT_BLOCKED (~12%), NARROW_PASSAGE (~3%). Two escape manoeuvres were triggered at steps 3560 and 3622 when the robot encountered a wall-corner area, both resolved within 25 steps. The trail visualisation shows comprehensive arena coverage with the robot visiting all quadrants and navigating around all 7 obstacles. Navigation remained stable throughout with no degradation in performance over time. The robot never became permanently stuck. |

#### Test T6: Threshold Sensitivity Analysis

| Parameter | Detail |
|-----------|--------|
| **Setup** | Full arena; SENSOR_THRESHOLD varied: 50 and 120 (compared against default 80 from Test 5) |
| **Purpose** | Evaluate the effect of detection sensitivity on navigation behaviour |
| **Pass Criteria** | Observable differences in behaviour; identify optimal threshold range |
| **Expected Result** | Lower threshold = more cautious; higher threshold = more aggressive |
| **Actual Result** | |

| Threshold | Behaviour Observed | Escape Events | Collisions | Assessment |
|-----------|-------------------|---------------|-----------|------------|
| **50** | Completely dysfunctional. Ambient sensor noise (60-76) exceeds threshold, causing every reading to classify as NARROW_PASSAGE. Robot triggered escape manoeuvres every 20-30 steps continuously. Unable to maintain forward motion. Trail shows chaotic back-and-forth pattern with no meaningful navigation. | 60+ in 3400 steps | 0 | Too sensitive — unusable |
| **80** (default) | Balanced operation. Robot distinguishes clearly between ambient noise and actual obstacles. Smooth navigation with proportional control handling most situations. Escape only triggered in genuine corner traps. | 2 in 5000 steps | 0 | Optimal |
| **120** | Smooth operation with fewer detection events. Robot passed closer to obstacles before reacting. More time spent in CLEAR state. One escape event at step 1965 when it got very close to an obstacle before detecting it. Slightly reduced safety margin but still collision-free. | 1 in 3300 steps | 0 | Functional but reduced safety margin |

**Key Finding:** The threshold must be set above the ambient sensor noise floor (~76) to avoid false positives. The optimal range is 80-120, with 80 providing the best balance between safety and efficiency.

### 4.3 Evaluation Criteria Summary

| Criterion | Result |
|-----------|--------|
| Accuracy of obstacle detection | All obstacles and walls detected reliably at threshold = 80 |
| Response time to obstacles | Within 1 simulation step (64 ms) of threshold crossing |
| Collision avoidance success rate | 100% across all test scenarios (0 collisions) |
| Navigation smoothness | Proportional control produced smooth curved paths (visible in trail) |
| Long-duration stability | 320 seconds of continuous operation with no permanent stuck states |

---

## 5. Results and Discussion

### 5.1 Performance Analysis

The implemented system demonstrated reliable autonomous navigation across all test scenarios with a 100% collision avoidance rate. The reactive AI approach proved sufficient for the static obstacle environment, providing consistent real-time responses to environmental stimuli.

**Measured performance metrics:**
- Detection-to-response latency: 64 ms (one simulation timestep)
- Collision rate: 0% across all tests (6 scenarios, total runtime > 15 minutes)
- Maximum sustained operation: 320 seconds (5000 steps) without collision
- Escape manoeuvre activation: 2 events in 5000-step endurance test (0.04% of steps)
- Ambient sensor noise: 60-76 units (establishes minimum viable threshold)

### 5.2 Effectiveness of Proportional Control

The proportional speed computation proved to be the most significant design decision. In Test T2 and T5, the robot resolved the majority of obstacle encounters (LEFT_BLOCKED, RIGHT_BLOCKED, FRONT_BLOCKED) using proportional control alone, without requiring escape manoeuvres. The console data confirms this:

- LEFT_BLOCKED responses show graduated right-wheel speed reduction (e.g., R=2.54, R=2.56, R=2.61) proportional to obstacle distance
- FRONT_BLOCKED responses correctly chose the less-obstructed direction based on sensor comparison
- The trail visualisation shows smooth curved avoidance paths rather than sharp angular turns

The intensity-based formula produces speeds ranging from full cruising (3.14 rad/s) down to reverse (-3.14 rad/s) as obstacles get closer, providing a continuous rather than binary response.

### 5.3 Role of Speed Smoothing

The low-pass filter (factor = 0.7) proved essential for producing natural motion. The console logs show the robot maintaining steady speeds (L=3.05, R=3.05) during CLEAR states rather than oscillating. The smoothing also prevents abrupt transitions: when moving from CLEAR to LEFT_BLOCKED, the speed change is gradual over 2-3 steps rather than instantaneous.

### 5.4 Turn Commitment and Anti-Oscillation

The 15-step turn commitment counter successfully prevented oscillation in all tests. In Test T2 and T5, multiple FRONT_BLOCKED events were resolved cleanly with a committed turn followed by smooth resumption. No instances of rapid left-right alternation were observed in the console logs.

### 5.5 Stuck Detection Reliability

The stuck detection system (30-step threshold) performed as intended:
- **Test T1/T3 (corner trap):** Correctly activated when the robot was genuinely trapped in a corner with sensor readings exceeding 1000-2000 on multiple sensors
- **Test T5 (endurance):** Only 2 activations in 5000 steps, both in genuine wall-corner scenarios
- **Test T2:** Zero activations — all obstacles handled by proportional control alone
- **False positive rate:** 0% at threshold = 80

### 5.6 Threshold Sensitivity

The threshold analysis (Test T6) revealed a critical finding: the e-puck's IR sensors produce ambient noise readings of 60-76 even with no obstacles present. This means:

- **Threshold = 50:** Falls below the noise floor, causing every sensor reading to register as an obstacle. The robot classified every situation as NARROW_PASSAGE and triggered escape manoeuvres every 20-30 steps. This rendered the system completely non-functional.
- **Threshold = 80:** Sits comfortably above the noise floor (76 max observed) while remaining sensitive enough to detect obstacles at useful range. This produced optimal behaviour.
- **Threshold = 120:** Requires obstacles to be closer before detection, reducing safety margin but still producing collision-free navigation.

**Implication:** The threshold must be calibrated relative to the sensor noise floor, not to absolute obstacle distance. A threshold of 80 provides only 4-16 units of margin above noise, which is sufficient due to the sharp proximity response curve of IR sensors (readings jump rapidly from ~70 to 100+ as objects enter detection range).

### 5.7 Limitations

The following limitations were identified during evaluation:

1. **Static environment only:** The system has not been tested with moving obstacles and would likely struggle with dynamic environments due to the lack of predictive capability.
2. **No path planning:** The robot does not build a map or plan optimal routes. The trail visualisation confirms it may revisit areas while leaving others unexplored.
3. **Sensor range limitation:** The e-puck's IR sensors have a maximum detection range of approximately 8 cm, which limits the robot's ability to plan ahead at higher speeds.
4. **Threshold sensitivity:** A threshold below the sensor noise floor (~76) renders the system non-functional, limiting the tunable range.
5. **Corner vulnerability:** The system requires 2-3 escape attempts to clear tight corners, though it ultimately succeeds without collision.

---

## 6. Conclusion

### 6.1 Summary

This project successfully designed, implemented, and evaluated an autonomous obstacle avoidance robot using reactive AI principles. The system demonstrates that effective real-time navigation can be achieved through relatively simple sensor-motor mappings combined with proportional control, speed smoothing, and commitment-based decision logic.

The key achievements of this project are:
1. **100% collision avoidance** across all six test scenarios, including challenging corner traps and narrow passages
2. **320 seconds of sustained autonomous operation** in a cluttered environment with 7 obstacles
3. **Smooth navigation** demonstrated by curved trail paths rather than jerky angular motion
4. **Robust escape mechanism** that resolves even triple-corner-trap scenarios within seconds
5. **Threshold analysis** revealing the critical relationship between sensor noise floor and detection sensitivity

The system met all project objectives: autonomous navigation, real-time obstacle detection, collision avoidance through appropriate decision-making, and smooth efficient movement.

### 6.2 Future Improvements

Several enhancements could be explored in future work:

1. **Wall-following behaviour:** Adding a dedicated wall-following mode would improve arena coverage by allowing the robot to systematically explore the perimeter.
2. **Dynamic obstacles:** Extending the algorithm to handle moving obstacles by incorporating velocity estimation from consecutive sensor readings.
3. **Machine learning optimisation:** Using reinforcement learning to automatically tune the threshold, smoothing factor, and speed coefficients based on collision-free performance metrics.
4. **Multi-robot coordination:** Deploying multiple e-puck robots with communication to demonstrate cooperative navigation and coverage.
5. **Sensor fusion:** Combining IR proximity data with camera input for more robust obstacle classification (distinguishing walls from objects, identifying gaps).
6. **Odometry integration:** Adding wheel encoder feedback to maintain heading awareness and enable basic area-coverage strategies.

---

## 7. References

1. Braitenberg, V. (1984). *Vehicles: Experiments in Synthetic Psychology*. MIT Press.

2. Cyberbotics Ltd. (2023). *Webots User Guide*. Available at: https://cyberbotics.com/doc/guide/index

3. Mondada, F., Bonani, M., Raemy, X., Pugh, J., Cianci, C., Klaptocz, A., Magnenat, S., Zufferey, J.-C., Floreano, D., and Martinoli, A. (2009). The e-puck, a robot designed for education in engineering. *Proceedings of the 9th Conference on Autonomous Robot Systems and Competitions*, 1(1), pp. 59-65.

4. Webots Reference Manual (2023). *E-puck Proto Documentation*. Cyberbotics Ltd.

5. Brooks, R.A. (1986). A robust layered control system for a mobile robot. *IEEE Journal on Robotics and Automation*, 2(1), pp. 14-23.

---

## 8. Appendices

### Appendix A: Source Code Reference

The complete source code for this project is contained in the following files within the project directory:

- **Controller script:** `controllers/obstacle_avoidance/obstacle_avoidance.py`
- **World definition:** `worlds/obstacle_avoidance.wbt`

### Appendix B: Running Instructions

**Prerequisites:**
- Webots R2023b or later (free download from cyberbotics.com)
- Python 3.x configured in Webots (Tools > Preferences > Python command)

**Steps to Run:**
1. Open Webots application
2. Select File > Open World
3. Navigate to `worlds/obstacle_avoidance.wbt` and open it
4. The simulation loads automatically with the robot and obstacles visible
5. Press the Play button (or Ctrl+Shift+P) to start the simulation
6. Observe the robot navigating the arena in the 3D viewport
7. Monitor console output (bottom panel) for sensor readings and decisions

**Configuration Adjustments:**
- To change detection sensitivity: modify `SENSOR_THRESHOLD` in `obstacle_avoidance.py`
- To disable logging for video: set `ENABLE_LOGGING = False`
- To reload after changes: press Ctrl+Shift+R (Reset Simulation) then Play

**Recording:**
- Screenshots: File > Take Screenshot
- Video: File > Make Movie (or use external screen recording software)
- Console logs: Select and copy from the Webots console panel
