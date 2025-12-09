import math
from statistics import mean
from controller import Robot, Motor, PositionSensor, Lidar, DistanceSensor
import lidar as l  # Only depends on the lidar module

# ========== Basic Configuration (Fully adopting original working code parameters) ==========
TIME_STEP = 32            # Webots time step (ms)
MAX_VELOCITY = 26         # Max motor speed from original code
DEAD_END_LIMIT = 0.6      # Dead end detection threshold (meters)
COLLISION_THRESHOLD = 0.8 # Collision detection threshold
base_speed = 6.0          # Base forward speed
POSITION_MULTIPLIER = 23.199033988  # Position scaling factor
TRACK_WIDTH = 0.300       # Wheel track width (meters)

# Robot initial pose (global variables)
ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_THETA = 0.0

def init_robot_devices(robot):
    """Initialize devices (Fully matching device names in the user's working code)"""
    # 1. Configure LiDAR (keeping 3D point cloud feature)
    lidar = robot.getDevice('laser')
    lidar.enable(TIME_STEP)
    lidar.enablePointCloud()  # Enable 3D point cloud (original code logic)
    print("✅ LiDAR initialization successful (3D point cloud enabled)")

    # 2. Initialize four motors (Original working code motor names)
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
    print("✅ Four motors initialized successfully")

    # 3. Initialize four position sensors (Original working code sensor names)
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
    print("✅ Four position sensors initialized successfully")

    # 4. Initialize distance sensors (Original code's 4 sensors)
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
            print(f"✅ Distance sensor {name} initialized successfully")
        else:
            print(f"⚠️ Distance sensor not found: {name} (Obstacle avoidance might be affected)")

    # 5. Set motors to velocity mode (Original code logic)
    for motor in motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)

    return {
        "motors": motors,  # [Front Left, Front Right, Rear Left, Rear Right]
        "position_sensors": sensors,  # [Front Left, Front Right, Rear Left, Rear Right]
        "distance_sensors": distance_sensors,
        "lidar": lidar
    }

def main():
    # Correct global variable declaration position (Resolves SyntaxError)
    global ROBOT_X, ROBOT_Y, ROBOT_THETA
    
    # Initialize robot instance
    robot = Robot()
    print(f"\n🚀 Controller started successfully! Time step: {TIME_STEP}ms")
    
    # Initialize all devices
    try:
        devices = init_robot_devices(robot)
    except ValueError as e:
        print(e)
        return
    
    # Initialize particle filter (Calling lidar's initialise function)
    PARTICLES = l.initialise(ROBOT_X, ROBOT_Y, ROBOT_THETA)
    print(f"✅ Particle filter initialized (Count: {len(PARTICLES)})")

    # Initialize variables (Fully adopting original code)
    front_left_last = front_right_last = rear_left_last = rear_right_last = 0
    turn_180_counter = 0
    TURN_STEPS_DURATION = 5  # U-turn duration in steps
    # Original code's Braitenberg obstacle avoidance coefficients
    coefficients = [[9.0, 15.0, -9.0, -15.0], [-9.0, -15.0, 9.0, 15.0]]

    # Main loop (Core logic completely retained from original code)
    while robot.step(TIME_STEP) != -1:
        # 1. Read distance sensor values
        distance_sensors_value = [0.0]*4
        for i in range(4):
            if devices["distance_sensors"][i]:
                distance_sensors_value[i] = devices["distance_sensors"][i].getValue()
        
        fl_val = distance_sensors_value[0]
        fr_val = distance_sensors_value[2]
        
        # 2. U-turn logic (Highest priority)
        motor_speed = [0.0, 0.0]  # [Left wheel speed, Right wheel speed]
        if turn_180_counter > 0:
            turn_180_counter -= 1
            # In-place left turn (Left wheels reverse, Right wheels forward)
            motor_speed[0] = -MAX_VELOCITY * 0.8
            motor_speed[1] = MAX_VELOCITY * 0.8
            if turn_180_counter == 0:
                print(">>> U-turn completed, resuming normal avoidance logic")
        else:
            # Dead end detection (Triggers U-turn)
            if fl_val < DEAD_END_LIMIT and fr_val < DEAD_END_LIMIT:
                print("!!! Dead end/corner detected → Initiating 180-degree U-turn !!!")
                turn_180_counter = TURN_STEPS_DURATION
            else:
                # Normal Braitenberg obstacle avoidance logic
                avoidance_speed = [0.0, 0.0]
                for i in range(2):
                    for j in range(4):
                        d = (2.0 - distance_sensors_value[j])
                        avoidance_speed[i] += (d * d) * coefficients[i][j]
                    motor_speed[i] = base_speed + avoidance_speed[i]
                
                # Fine-tuned avoidance adjustment (Original code logic)
                if fl_val < COLLISION_THRESHOLD and fr_val < COLLISION_THRESHOLD:
                    if abs(fl_val - fr_val) > 0.2:
                        if fl_val > fr_val:
                            motor_speed[0] = -MAX_VELOCITY * 0.5
                            motor_speed[1] = MAX_VELOCITY * 0.5
                        else:
                            motor_speed[0] = MAX_VELOCITY * 0.5
                            motor_speed[1] = -MAX_VELOCITY * 0.5

        # 3. Speed limiting (Preventing exceeding max velocity)
        motor_speed[0] = max(-MAX_VELOCITY, min(motor_speed[0], MAX_VELOCITY))
        motor_speed[1] = max(-MAX_VELOCITY, min(motor_speed[1], MAX_VELOCITY))

        # 4. Set four motor speeds (Front/Rear Left share left speed, Front/Rear Right share right speed)
        devices["motors"][0].setVelocity(motor_speed[0])  # Front Left
        devices["motors"][1].setVelocity(motor_speed[1])  # Front Right
        devices["motors"][2].setVelocity(motor_speed[0])  # Rear Left
        devices["motors"][3].setVelocity(motor_speed[1])  # Rear Right

        # 5. Update robot position (Original code localization logic)
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

        # Update robot pose
        ROBOT_THETA += angle_change
        ROBOT_X += distance * math.cos(ROBOT_THETA)
        ROBOT_Y += distance * math.sin(ROBOT_THETA)

        # Update position sensors' last-read values
        front_left_last = devices["position_sensors"][0].getValue()
        front_right_last = devices["position_sensors"][1].getValue()
        rear_left_last = devices["position_sensors"][2].getValue()
        rear_right_last = devices["position_sensors"][3].getValue()

        # 6. LiDAR grid map generation (Restoring saving + console printing)
        grid = l.run_lidar([distance, angle_change], ROBOT_X, ROBOT_Y, ROBOT_THETA, devices["lidar"], PARTICLES)
        # Save + visualize every 20 steps (to avoid excessive printing)
        if robot.getTime() % (20 * TIME_STEP / 1000) < TIME_STEP / 1000:
            l.save_grid_map(grid)  # Generate grid_map.txt file
            l.visualize_grid(grid) # Console print the grid map

if __name__ == "__main__":
    main()
