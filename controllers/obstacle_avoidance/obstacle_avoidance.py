"""
Autonomous Obstacle Avoidance Controller for E-puck Robot
Uses reactive AI with proportional speed control for smooth navigation.

Sensor Layout (top view, robot facing up):
    ps7 [front-left]    ps0 [front-right]
    ps6 [left-front]    ps1 [right-front]
    ps5 [left]          ps2 [right]
    ps4 [rear-left]     ps3 [rear-right]
"""

from controller import Supervisor
import random

# ============================================================
# CONFIGURATION - Adjust these values for testing/tuning
# ============================================================
MAX_SPEED = 6.28          # Maximum motor velocity in rad/s (e-puck limit)
BASE_SPEED = 3.14         # Normal cruising speed (50% of max)
SENSOR_THRESHOLD = 80.0   # Proximity value above which an obstacle is detected
LOG_INTERVAL = 50         # Print sensor data every N timesteps (~3.2s at 64ms)
ENABLE_LOGGING = True     # Set to False for clean video recording
STUCK_THRESHOLD = 30      # Steps before triggering escape manoeuvre (~2 seconds)
SMOOTHING_FACTOR = 0.7    # Speed transition smoothing (0=instant, 1=no change)
TURN_COMMITMENT_STEPS = 15  # Minimum steps to commit to a turn direction

# ============================================================
# ROBOT INITIALISATION
# ============================================================
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

# ============================================================
# TRAIL SETUP
# ============================================================
trail_root = robot.getRoot()
trail_children = trail_root.getField("children")
trail_counter = 0
TRAIL_INTERVAL = 10  # drop a dot every 10 steps

# ============================================================
# STATE VARIABLES
# ============================================================
prev_left_speed = 0.0
prev_right_speed = 0.0
turn_commitment_counter = 0
committed_turn_direction = None  # "left" or "right"
stuck_counter = 0
step_count = 0
escape_phase = None  # None, "reverse", or "spin"
escape_counter = 0


# ============================================================
# HELPER FUNCTIONS
# ============================================================
def classify_situation(sensor_values):
    """
    Classify the current navigation situation based on sensor readings.
    Returns one of: CLEAR, FRONT_BLOCKED, LEFT_BLOCKED, RIGHT_BLOCKED, NARROW_PASSAGE
    """
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


def compute_speeds(sensor_values, situation):
    """
    Compute left and right motor speeds using proportional control.
    Closer obstacles produce stronger avoidance responses.
    """
    global turn_commitment_counter, committed_turn_direction

    # Normalise sensor values to [0, 1]
    normalised = [min(v / 1000.0, 1.0) for v in sensor_values]

    left_speed = BASE_SPEED
    right_speed = BASE_SPEED

    if situation == "CLEAR":
        # Approach modulation: slow down if front sensors detect something below threshold
        front_max = max(normalised[0], normalised[7], normalised[1], normalised[6])
        approach_factor = 1.0 - 0.4 * front_max
        left_speed = BASE_SPEED * max(approach_factor, 0.6)
        right_speed = BASE_SPEED * max(approach_factor, 0.6)

    elif situation == "FRONT_BLOCKED":
        if turn_commitment_counter > 0:
            # Continue committed turn direction
            turn_commitment_counter -= 1
            if committed_turn_direction == "left":
                left_speed = -0.3 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            else:
                left_speed = 0.5 * MAX_SPEED
                right_speed = -0.3 * MAX_SPEED
        else:
            # Choose new turn direction based on which side has less obstruction
            left_sum = normalised[5] + normalised[6] + normalised[7]
            right_sum = normalised[0] + normalised[1] + normalised[2]

            if left_sum <= right_sum:
                # Turn left (less obstacle on left side)
                committed_turn_direction = "left"
                left_speed = -0.3 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            else:
                # Turn right (less obstacle on right side)
                committed_turn_direction = "right"
                left_speed = 0.5 * MAX_SPEED
                right_speed = -0.3 * MAX_SPEED

            turn_commitment_counter = TURN_COMMITMENT_STEPS

    elif situation == "LEFT_BLOCKED":
        # Proportional right turn based on obstacle intensity
        intensity = max(normalised[5], normalised[6])
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED * (1.0 - 1.5 * intensity)
        right_speed = max(right_speed, -0.5 * MAX_SPEED)

    elif situation == "RIGHT_BLOCKED":
        # Proportional left turn based on obstacle intensity
        intensity = max(normalised[1], normalised[2])
        left_speed = BASE_SPEED * (1.0 - 1.5 * intensity)
        right_speed = BASE_SPEED
        left_speed = max(left_speed, -0.5 * MAX_SPEED)

    elif situation == "NARROW_PASSAGE":
        # Slow down and bias toward less obstructed side
        left_sum = normalised[5] + normalised[6] + normalised[7]
        right_sum = normalised[0] + normalised[1] + normalised[2]
        left_speed = BASE_SPEED * 0.4
        right_speed = BASE_SPEED * 0.4
        if left_sum > right_sum:
            left_speed *= 1.5
            right_speed *= 0.5
        else:
            left_speed *= 0.5
            right_speed *= 1.5

    # Clamp speeds to motor limits
    left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
    right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

    return left_speed, right_speed


def smooth_speed(current, previous):
    """Apply low-pass filter to smooth speed transitions."""
    return SMOOTHING_FACTOR * previous + (1.0 - SMOOTHING_FACTOR) * current


def log_status(step, situation, sensor_values, left_speed, right_speed):
    """Print current status to console for evaluation."""
    sensor_str = ", ".join([f"{v:.0f}" for v in sensor_values])
    print(f"[Step {step:>5}] Situation: {situation:<15} | "
          f"Sensors: [{sensor_str}] | "
          f"Speeds: L={left_speed:>6.2f}  R={right_speed:>6.2f}")


# ============================================================
# MAIN CONTROL LOOP
# ============================================================
print("=" * 60)
print("  Autonomous Obstacle Avoidance Controller Started")
print(f"  Sensor Threshold: {SENSOR_THRESHOLD}")
print(f"  Base Speed: {BASE_SPEED:.2f} rad/s")
print(f"  Logging: {'ON' if ENABLE_LOGGING else 'OFF'}")
print("=" * 60)

while robot.step(timestep) != -1:
    step_count += 1

    # --- Read all sensor values ---
    sensor_values = [sensors[i].getValue() for i in range(8)]

    # --- Draw trail dot ---
    trail_counter += 1
    if trail_counter % TRAIL_INTERVAL == 0:
        robot_node = robot.getSelf()
        pos = robot_node.getPosition()
        trail_children.importMFNodeFromString(-1, f'''
        DEF TRAIL Solid {{
          translation {pos[0]} {pos[1]} 0.001
          children [
            Shape {{
              appearance PBRAppearance {{
                baseColor 1 0 0
                roughness 1
                metalness 0
              }}
              geometry Sphere {{
                radius 0.008
              }}
            }}
          ]
        }}
        ''')

    # --- Check if in escape manoeuvre ---
    if escape_phase is not None:
        escape_counter -= 1

        if escape_phase == "reverse":
            left_speed = -0.5 * MAX_SPEED
            right_speed = -0.5 * MAX_SPEED
            if escape_counter <= 0:
                # Switch to spin phase
                escape_phase = "spin"
                escape_counter = random.randint(10, 20)
                committed_turn_direction = random.choice(["left", "right"])

        elif escape_phase == "spin":
            if committed_turn_direction == "left":
                left_speed = -0.5 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            else:
                left_speed = 0.5 * MAX_SPEED
                right_speed = -0.5 * MAX_SPEED
            if escape_counter <= 0:
                # End escape
                escape_phase = None
                stuck_counter = 0
                turn_commitment_counter = 0
                print(f"[Step {step_count:>5}] ** ESCAPE COMPLETE - Resuming normal navigation **")

        # Apply escape speeds (no smoothing during escape)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        prev_left_speed = left_speed
        prev_right_speed = right_speed
        continue

    # --- Normal navigation ---
    situation = classify_situation(sensor_values)

    # --- Stuck detection ---
    if situation in ("FRONT_BLOCKED", "NARROW_PASSAGE"):
        stuck_counter += 1
    else:
        stuck_counter = 0

    if stuck_counter >= STUCK_THRESHOLD:
        # Trigger escape manoeuvre
        escape_phase = "reverse"
        escape_counter = 5
        print(f"[Step {step_count:>5}] ** STUCK DETECTED - Initiating escape manoeuvre **")
        left_motor.setVelocity(-0.5 * MAX_SPEED)
        right_motor.setVelocity(-0.5 * MAX_SPEED)
        prev_left_speed = -0.5 * MAX_SPEED
        prev_right_speed = -0.5 * MAX_SPEED
        continue

    # --- Compute motor speeds ---
    left_speed, right_speed = compute_speeds(sensor_values, situation)

    # --- Apply speed smoothing ---
    left_speed = smooth_speed(left_speed, prev_left_speed)
    right_speed = smooth_speed(right_speed, prev_right_speed)

    # --- Set motor velocities ---
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # --- Update state ---
    prev_left_speed = left_speed
    prev_right_speed = right_speed

    # --- Logging ---
    if ENABLE_LOGGING and step_count % LOG_INTERVAL == 0:
        log_status(step_count, situation, sensor_values, left_speed, right_speed)
