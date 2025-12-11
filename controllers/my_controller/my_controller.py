import math
import numpy as np
from statistics import mean
from controller import Robot, Motor, PositionSensor, Lidar, DistanceSensor
import lidar as l  # Only depends on the lidar module
import A
import sys

# Basic Configuration
TIME_STEP = 32          # Webots time step (ms)
MAX_VELOCITY = 2         # Max motor speed from original code
DEAD_END_LIMIT = 0.2      # Dead end detection threshold (meters)
COLLISION_THRESHOLD = 0.8 # Collision detection threshold
base_speed = 6.0          # Base forward speed
POSITION_MULTIPLIER = 23.199033988  # Position scaling factor
TRACK_WIDTH = 0.300       # Wheel track width (meters)

# Robot initial pose (global variables)
initial_pos = sys.argv[1].split()
ROBOT_X = float(initial_pos[0])
ROBOT_Y = float(initial_pos[1])
ROBOT_THETA = float(initial_pos[2])

# Navigation Goal Point
GOAL_WORLD = (0, 1.5)

# State Machine
STATE_PLANNING = 0
STATE_FOLLOWING = 1
STATE_IDLE = 2
STATE_RECOVERING = 3 # Dedicated recovery state to prevent getting stuck in the main loop
current_state = STATE_PLANNING
current_path = []   # Stores the path calculated by A* [(gx, gy), (gx, gy)...]
path_index = 0      # Current index along the path
recover_timer = 0    # Recovery countdown timer

def init_robot_devices(robot):
    """Initialize devices (Fully matching device names in the user's working code)"""
    # Configure LiDAR
    lidar = robot.getDevice('laser')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()  # Enable 3D point cloud

    # Initialize four motors
    front_left_motor = robot.getDevice("fl_wheel_joint")
    front_right_motor = robot.getDevice("fr_wheel_joint")
    rear_left_motor = robot.getDevice("rl_wheel_joint")
    rear_right_motor = robot.getDevice("rr_wheel_joint")
    
    # Check if motors are found
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]
    motor_names = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]
    for name, motor in zip(motor_names, motors):
        if motor is None:
            raise ValueError(f"❌ Motor not found: {name}")
            
    # Initialize four position sensors
    front_left_position = robot.getDevice("front left wheel motor sensor")
    front_right_position = robot.getDevice("front right wheel motor sensor")
    rear_left_position = robot.getDevice("rear left wheel motor sensor")
    rear_right_position = robot.getDevice("rear right wheel motor sensor")
    
    # Check position sensors
    sensors = [front_left_position, front_right_position, rear_left_position, rear_right_position]
    sensor_names = [
        "front left wheel motor sensor", 
        "front right wheel motor sensor", 
        "rear left wheel motor sensor", 
        "rear right wheel motor sensor"
    ]
    for name, sensor in zip(sensor_names, sensors):
        if sensor is None:
            raise ValueError(f"❌ Position sensor not found: {name}")
    # Enable position sensors
    front_left_position.enable(TIME_STEP)
    front_right_position.enable(TIME_STEP)
    rear_left_position.enable(TIME_STEP)
    rear_right_position.enable(TIME_STEP)

    # Initialize distance sensors
    distance_sensors = [
        robot.getDevice("fl_range"),
        robot.getDevice("rl_range"),
        robot.getDevice("fr_range"),
        robot.getDevice("rr_range")
    ]
    ds_names = ["fl_range", "rl_range", "fr_range", "rr_range"]
    for name, ds in zip(ds_names, distance_sensors):
        if ds:
            ds.enable(TIME_STEP)

    # Set motors to velocity mode
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    return {
        "motors": motors,  # [Front Left, Front Right, Rear Left, Rear Right]
        "position_sensors": sensors,  # [Front Left, Front Right, Rear Left, Rear Right]
        "distance_sensors": distance_sensors,
        "lidar": lidar
    }

def normalize_angle(angle):  #Angle normalization (-pi to pi)
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def main():
    # Correct global variable declaration position
    global ROBOT_X, ROBOT_Y, ROBOT_THETA, current_state, current_path, path_index
    
    # Initialize robot instance
    robot = Robot()
    
    # Initialize all devices
    try:
        devices = init_robot_devices(robot)
    except ValueError as e:
        print(e)
        return
    
    rec = robot.getDevice("receiver")
    emit = robot.getDevice("emitter")
    rec.enable(5)

    # Initialize particle filter 
    PARTICLES = l.initialise(ROBOT_X, ROBOT_Y, ROBOT_THETA)

    # Initialize variables
    front_left_last = front_right_last = rear_left_last = rear_right_last = 0
    turn_180_counter = 0
    TURN_STEPS_DURATION = 5 
    coefficients = [[9.0, 15.0, -9.0, -15.0], [-9.0, -15.0, 9.0, 15.0]]

    last_angle_error = 0.0
    grid = [[0 for _ in range(100)] for _ in range(100)]
    
    # Visit heatmap
    visit_map = np.zeros((100, 100))
    
    # Variables for stuck detection
    last_fl_val = 0.0
    last_fr_val = 0.0
    stuck_counter = 0 # Records the number of frames the robot has been stuck
    
    # trap detection variables
    trap_timer = 0
    trap_start_x = ROBOT_X
    trap_start_y = ROBOT_Y
    
    recovery_turn = 0
    count = 0
    # Main loop
    while robot.step(TIME_STEP) != -1:
        count += 1
        # ALLOWS THE LIDAR TO SET UP
        if count < 100 :
            continue
        # Read distance sensor values
        distance_sensors_value = [0.0]*4
        for i in range(4):
            if devices["distance_sensors"][i]:
                distance_sensors_value[i] = devices["distance_sensors"][i].getValue()
        
        fl_val = distance_sensors_value[0]
        fr_val = distance_sensors_value[2]
        
        motor_speed = [0.0, 0.0]  # [Left wheel speed, Right wheel speed]
        
        # Stuck detection
        if current_state != STATE_IDLE and current_state != STATE_RECOVERING:
            
            # Visual Stasis (Prevents wheel spin in place)
            # Very little change in sensor readings means the robot hasn't moved relative to the wall
            sensor_diff = abs(fl_val - last_fl_val) + abs(fr_val - last_fr_val)
            is_visually_static = sensor_diff < 0.04
            
            # Physical Stall
            # Get current encoder values, calculate actual wheel movement
            curr_fl = devices["position_sensors"][0].getValue()
            curr_fr = devices["position_sensors"][1].getValue()
            encoder_move = (abs(curr_fl - front_left_last) + abs(curr_fr - front_right_last)) / 2.0
            is_physically_stuck = encoder_move < 0.002
            
            # Close to wall
            # If too close to the wall (less than 15cm), regardless of being stuck, forcing a reverse maneuver.
            is_too_close = (fl_val < 0.15) or (fr_val < 0.15)
            
            # Comprehensive Judgment (Accumulate stuck count if any condition is met)
            cond1 = is_visually_static and (fl_val < 0.25 or fr_val < 0.25)
            cond2 = is_physically_stuck
            cond3 = is_too_close
            
            if cond1 or cond2 or cond3:
                stuck_counter += 1
            else:
                # Slowly reduce to prevent occasional normal stops from resetting it.
                stuck_counter = max(0, stuck_counter - 1)
        
        last_fl_val = fl_val
        last_fr_val = fr_val

        # Trigger Threshold: Accumulate to 30 points (approx. 1 second)
        if stuck_counter > 30:
            print(f"🚨 Confirmed stuck against wall (Dist:{fl_val:.2f}/{fr_val:.2f}) -> Starting Recovery!")
            stuck_counter = 0
            current_state = STATE_RECOVERING
            recovery_phase = 1 
            recover_timer = 20
        
        if current_state != STATE_IDLE and current_state != STATE_RECOVERING:
            trap_timer += 1
            
            if trap_timer > 100:
                dist_moved = math.sqrt((ROBOT_X - trap_start_x)**2 + (ROBOT_Y - trap_start_y)**2)
                
                if dist_moved < 0.05: # Haven't moved 20cm in 3 seconds
                    print(f"🚨 Detected shaking in place (3s displacement {dist_moved:.2f}m) -> Forced Recovery!")
                    current_state = STATE_RECOVERING
                    recovery_phase = 1
                    recover_timer = 50
                
                # Reset timer and start point
                trap_timer = 0
                trap_start_x = ROBOT_X
                trap_start_y = ROBOT_Y
        else:
            # If in recovery or idle, reset detection
            trap_timer = 0
            trap_start_x = ROBOT_X
            trap_start_y = ROBOT_Y
            
        # Forceful Recovery
        if current_state == STATE_RECOVERING:
            if recovery_phase == 1:
                # Reverse
                motor_speed = [-3.0, -3.0] 
                recover_timer -= 1
                if recover_timer <= 0:
                    recovery_phase = 2
                    recover_timer = 30
                    
                    if fl_val > fr_val:
                        recovery_turn = 1  # Lock to left turn
                    else:
                        recovery_turn = -1 # Lock to right turn
            
            elif recovery_phase == 2:
                # Ignores sensors
                if recovery_turn == 1:
                    motor_speed = [-5.0, 5.0] # Left turn
                else:
                    motor_speed = [5.0, -5.0] # Right turn
                
                recover_timer -= 1
                if recover_timer <= 0:
                    print(">>> Recovery complete, resetting state")
                    current_state = STATE_PLANNING
                    # plan_cooldown = 0 # Not used in final code
                    last_angle_error = 0.0
                    recovery_phase = 0
                    recovery_turn = 0
        
        # 2. U-turn logic (Highest priority)
        elif turn_180_counter > 0:
            turn_180_counter -= 1
            # In-place left turn (Left wheels reverse, Right wheels forward)
            motor_speed[0] = -MAX_VELOCITY * 0.8
            motor_speed[1] = MAX_VELOCITY * 0.8

        else:
            # Dead end detection (Triggers U-turn)
            if fl_val < DEAD_END_LIMIT and fr_val < DEAD_END_LIMIT:
                turn_180_counter = TURN_STEPS_DURATION
                motor_speed = [0, 0]
            else:
                if fl_val < 0.25 or fr_val < 0.25:
                    ## In-place rotation for obstacle avoidance
                    if fl_val < fr_val: motor_speed = [6.0, -6.0]
                    else: motor_speed = [-6.0, 6.0]
                    if current_state == STATE_FOLLOWING: current_state = STATE_PLANNING
                        
                # Execute A* Navigation Logic
                else:
                    # Path Planning
                    if current_state == STATE_PLANNING:
                        # Coordinate conversion: Meters -> Grid
                        start_node = (l.meters_to_grid(ROBOT_X), l.meters_to_grid(ROBOT_Y))
                        goal_node = (l.meters_to_grid(GOAL_WORLD[0]), l.meters_to_grid(GOAL_WORLD[1]))
                        
                        # Ensure grid exists
                        if grid is not None:
                            # Turns visited paths into walls
                            nav_grid = [row[:] for row in grid]
                            
                            # Mark cells with excessive visits (>50) as obstacles
                            for y in range(100):
                                for x in range(100):
                                    if visit_map[y][x] > 50:
                                        nav_grid[y][x] = 1
                            
                            # Ensure the Goal point itself is not marked as a wall
                            nav_grid[goal_node[1]][goal_node[0]] = 0
                            # Ensure the Start point itself is not marked as a wall
                            nav_grid[start_node[1]][start_node[0]] = 0

                            # Use the nav_grid with virtual walls for planning
                            path = A.a_star(nav_grid, start_node, goal_node)
                        else:
                            path = None
                
                        if path:
                            current_path = path
                            path_index = 0
                            current_state = STATE_FOLLOWING
                        else:
                            motor_speed = [base_speed, base_speed]
                
                    # Path Following
                    elif current_state == STATE_FOLLOWING:
                        if path_index < len(current_path):
                            # Get target point and convert back to meters
                            target_grid = current_path[path_index]
                            target_x = l.grid_to_meters(target_grid[0])
                            target_y = l.grid_to_meters(target_grid[1])
                            
                            # Calculate angle difference
                            dx = target_x - ROBOT_X
                            dy = target_y - ROBOT_Y
                            target_angle = math.atan2(dy, dx)
                            angle_diff = normalize_angle(target_angle - ROBOT_THETA)
                            
                            # Control Logic
                            cruise = 8.0
                            turn = angle_diff * 4.0
                            turn = max(-5.0, min(turn, 5.0))
                            
                            left_speed = cruise - turn
                            right_speed = cruise + turn
                            
                            # Apply push force when near a wall (< 0.35m)
                            SAFE_DIST = 0.35
                            PUSH_GAIN = 15.0
                            
                            if fl_val < SAFE_DIST:
                                push = (SAFE_DIST - fl_val) * PUSH_GAIN
                                left_speed += push
                                right_speed -= push
                            elif fr_val < SAFE_DIST:
                                push = (SAFE_DIST - fr_val) * PUSH_GAIN
                                left_speed -= push
                                right_speed += push
                                
                            motor_speed = [left_speed, right_speed]
                            
                            # Reached detection
                            if math.sqrt(dx*dx + dy*dy) < 0.25:
                                path_index += 1
                        else:
                            print("Destination Reached!")
                            current_state = STATE_IDLE
                            motor_speed = [0, 0]
                            last_angle_error = 0.0
                            
                    # Idle
                    elif current_state == STATE_IDLE:
                        motor_speed = [0, 0]

                # Reset error
                if current_state != STATE_FOLLOWING:
                    last_angle_error = 0.0

        # Speed limiting (Preventing exceeding max velocity)
        motor_speed[0] = max(-MAX_VELOCITY, min(motor_speed[0], MAX_VELOCITY))
        motor_speed[1] = max(-MAX_VELOCITY, min(motor_speed[1], MAX_VELOCITY))

        # Set four motor speeds (Front/Rear Left share left speed, Front/Rear Right share right speed)
        devices["motors"][0].setVelocity(motor_speed[0])  # Front Left
        devices["motors"][1].setVelocity(motor_speed[1])  # Front Right
        devices["motors"][2].setVelocity(motor_speed[0])  # Rear Left
        devices["motors"][3].setVelocity(motor_speed[1])  # Rear Right

        # Update robot position
        front_left_change = devices["position_sensors"][0].getValue() - front_left_last
        front_right_change = devices["position_sensors"][1].getValue() - front_right_last
        rear_left_change = devices["position_sensors"][2].getValue() - rear_left_last
        rear_right_change = devices["position_sensors"][3].getValue() - rear_right_last

        # Calculate distance traveled
        distance = mean([front_left_change, front_right_change, rear_left_change, rear_right_change]) / POSITION_MULTIPLIER
        # Calculate angle change
        left_distance = mean([front_left_change, rear_left_change]) / POSITION_MULTIPLIER
        right_distance = mean([front_right_change, rear_right_change]) / POSITION_MULTIPLIER
        angle_change = (left_distance - right_distance) / TRACK_WIDTH

        # Run Particle Filter to correct position
        grid, PARTICLES, est_pos = l.run_lidar(
            [distance, angle_change], 
            ROBOT_X, ROBOT_Y, ROBOT_THETA, 
            devices["lidar"], 
            PARTICLES, 
            grid
        )

        # Update robot pose with Particle Filter's estimated result
        ROBOT_X = est_pos[0]
        ROBOT_Y = est_pos[1]
        ROBOT_THETA = est_pos[2]
        
        # Update Visit Heatmap
        # Calculate current grid cell coordinates
        gx = l.meters_to_grid(ROBOT_X)
        gy = l.meters_to_grid(ROBOT_Y)
        
        # Increase the visit count for the current cell (the longer it stays, the higher the value)
        if 0 <= gx < 100 and 0 <= gy < 100:
            visit_map[gy][gx] += 1
            
        # Make old footprints slowly disappear
        # prevent the path from being permanently blocked
        visit_map *= 0.999
        
        # Update Last values
        front_left_last = devices["position_sensors"][0].getValue()
        front_right_last = devices["position_sensors"][1].getValue()
        rear_left_last = devices["position_sensors"][2].getValue()
        rear_right_last = devices["position_sensors"][3].getValue()

        # LiDAR grid map generation
        if robot.getTime() % (20 * TIME_STEP / 1000) < TIME_STEP / 1000:
            l.save_grid_map(grid)  # Generate grid_map.txt file
            l.visualize_grid(grid) # Console print the grid map

        if rec.getQueueLength() > 0:
            if rec.getString() == "maps":
                emit.send(robot.name + "|" + str(grid))
            else:
                result = str(rec.getString())
                names = result.split("|")
                if len(names) == 3:
                    if names[0] or names[1] == robot.name:
                        newMap = names[2]
                        newMap = newMap[2:-2].split("], [")
                        for i in range(len(newMap)):
                            splitres = newMap[i].split(", ")
                            newMap[i] = list(map(int, splitres))
                        grid = newMap
            rec.nextPacket()


if __name__ == "__main__":
    main()
