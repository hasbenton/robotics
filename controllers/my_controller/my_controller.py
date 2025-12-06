

import math
from statistics import mean
from controller import Robot, Motor, PositionSensor, Lidar, Accelerometer, Gyro, Compass, DistanceSensor

ROBOT_X = 0.0  # x-coordinate in meters
ROBOT_Y = 0.0  # y-coordinate in meters
ROBOT_THETA = 0.0  # Orientation (radians, 0 = facing positive x)

TIME_STEP = 32
MAX_VELOCITY = 26



# Define variables
front_left_motor = None
front_right_motor = None
rear_left_motor = None
rear_right_motor = None

avoidance_speed = [0.0, 0.0]
base_speed = 6.0
motor_speed = [0.0, 0.0]

#Scales position movement to distane metric
POSITION_MULTIPLIER = 23.199033988
TRACK_WIDTH = 0.300 #Metres

distance_sensors = [None] * 4
distance_sensors_value = [0.0] * 4

coefficients = [[9.0,  15.0, -9.0, -15.0], [-9.0, -15.0,  9.0,  15.0]]
COLLISION_THRESHOLD = 0.8

# Initialize Webots
robot = Robot()

# Get a handler to the motors
front_left_motor = robot.getDevice("fl_wheel_joint")
front_right_motor = robot.getDevice("fr_wheel_joint")
rear_left_motor = robot.getDevice("rl_wheel_joint")
rear_right_motor = robot.getDevice("rr_wheel_joint")

front_left_position = robot.getDevice("front left wheel motor sensor")
front_right_position = robot.getDevice("front right wheel motor sensor")
rear_left_position = robot.getDevice("rear left wheel motor sensor")
rear_right_position = robot.getDevice("rear right wheel motor sensor")

front_left_position.enable(10)
front_right_position.enable(10)
rear_left_position.enable(10)
rear_right_position.enable(10)

front_left_last = 0
front_right_last = 0
rear_left_last = 0
rear_right_last = 0

front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))

front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

# Enable distance sensors
distance_sensors[0] = robot.getDevice("fl_range")
distance_sensors[1] = robot.getDevice("rl_range")
distance_sensors[2] = robot.getDevice("fr_range")
distance_sensors[3] = robot.getDevice("rr_range")

for i in range(4):
    distance_sensors[i].enable(TIME_STEP)

# ==========================================
# [Addition 1] Define U-turn timer
# ==========================================
# The number 80 here represents steps. With TIME_STEP=32ms, 80 steps ≈ 2.5 seconds.
# If it can't turn 180 degrees, increase this number; if it turns too much, decrease it.
turn_180_counter = 0 
TURN_STEPS_DURATION = 5 

# Main loop
while robot.step(TIME_STEP) != -1:

    # Get sensor values
    for i in range(4):
        distance_sensors_value[i] = distance_sensors[i].getValue()
        
    fl_val = distance_sensors_value[0]
    fr_val = distance_sensors_value[2]
    
    # ==========================================
    # [Modification] Highest priority: enforce rotation if in U-turn mode
    # ==========================================
    if turn_180_counter > 0:
        turn_180_counter -= 1 # Decrement countdown
        
        # Force in-place left-back rotation (can be changed to right turn if preferred)
        # Left wheels reverse, right wheels forward
        motor_speed[0] = -MAX_VELOCITY * 0.8
        motor_speed[1] = MAX_VELOCITY * 0.8
        
        if turn_180_counter == 0:
            print(">>> U-turn completed, resuming normal logic")
            
    else:
        # ==========================================
        # Normal logic mode (only executed when not turning around)
        # ==========================================
        
        # 1. Trigger condition: detect if entering a dead end
        # If both front sensors are very close (e.g., both less than 0.6, closer than normal obstacle avoidance)
        # 0.6 here is to ensure triggering only when truly hitting a corner, avoiding false triggers during normal avoidance
        DEAD_END_LIMIT = 0.6 
        
        if fl_val < DEAD_END_LIMIT and fr_val < DEAD_END_LIMIT:
            print("!!! Detected dead end/corner -> initiating 180-degree U-turn procedure !!!")
            turn_180_counter = TURN_STEPS_DURATION # Set counter to take over control
            # No need to set speed here; the above if will take over in the next loop
            
        else:
            # 2. If not a dead end, execute normal obstacle avoidance (Braitenberg + previous logic)
            
            # Braitenberg basic obstacle avoidance
            for i in range(2):
                avoidance_speed[i] = 0.0
                for j in range(4):
                    d = (2.0 - distance_sensors_value[j])
                    avoidance_speed[i] += (d * d) * coefficients[i][j]
                motor_speed[i] = base_speed + avoidance_speed[i]
            
            # Your previous left-right fine-tuning logic (optional, or use Braitenberg above directly)
            # If you want to keep the previous fine-tuning, you can add it here, but Braitenberg is recommended for normal situations
            # Braitenberg may only fail at very close distances, where DEAD_END_LIMIT above will trigger U-turn
            
            # If you want to keep the previous "obstacle ahead, choose one side to turn" logic as a middle layer:
            if fl_val < COLLISION_THRESHOLD and fr_val < COLLISION_THRESHOLD:
                if abs(fl_val - fr_val) > 0.2:
                    # Large difference, fine-tune
                        if fl_val > fr_val:
                            motor_speed[0] = -MAX_VELOCITY * 0.5 
                            motor_speed[1] = MAX_VELOCITY * 0.5
                        else:
                            motor_speed[0] = MAX_VELOCITY * 0.5
                            motor_speed[1] = -MAX_VELOCITY * 0.5

    # ==========================================
    # Unified speed limiting and application
    # ==========================================
    for i in range(2):
        motor_speed[i] = max(-MAX_VELOCITY, min(motor_speed[i], MAX_VELOCITY))

    front_left_motor.setVelocity(motor_speed[0])
    front_right_motor.setVelocity(motor_speed[1])
    rear_left_motor.setVelocity(motor_speed[0])
    rear_right_motor.setVelocity(motor_speed[1])

    
    # ==========================================
    # Update Position
    # ==========================================
    front_left_change = front_left_position.getValue() - front_left_last
    front_right_change = front_right_position.getValue() - front_right_last
    rear_left_change = rear_left_position.getValue() - rear_left_last
    rear_right_change = rear_right_position.getValue() - rear_right_last


    #Distance Traveled
    distance =  mean([front_left_change, front_right_change, rear_left_change, rear_right_change]) / POSITION_MULTIPLIER

    #Angle change
    left_distance =  mean([front_left_change, rear_left_change]) / POSITION_MULTIPLIER
    right_distance =  mean([front_right_change, rear_right_change]) / POSITION_MULTIPLIER

    angle_change = (left_distance - right_distance) / TRACK_WIDTH

    ROBOT_THETA += angle_change

    ROBOT_X += distance * math.cos(ROBOT_THETA)
    ROBOT_Y += distance * math.sin(ROBOT_THETA)

    front_left_last = front_left_position.getValue()
    front_right_last = front_right_position.getValue()
    rear_left_last = rear_left_position.getValue()
    rear_right_last = rear_right_position.getValue()
