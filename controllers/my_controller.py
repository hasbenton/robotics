import math
from controller import Robot, Motor, DistanceSensor

TIME_STEP = 32
MAX_VELOCITY = 26

def main():
    # Initialize motor variables
    front_left_motor = None
    front_right_motor = None
    rear_left_motor = None
    rear_right_motor = None
    
    # Movement control variables
    avoidance_speed = [0.0, 0.0]
    base_speed = 6.0
    motor_speed = [0.0, 0.0]
    
    # Distance sensor configuration
    distance_sensors = [None] * 4
    distance_sensors_value = [0.0] * 4
    COLLISION_THRESHOLD = 0.8
    
    # Braitenberg coefficients for obstacle avoidance
    coefficients = [
        [9.0, 15.0, -9.0, -15.0],   # Left motor coefficients
        [-9.0, -15.0, 9.0, 15.0]    # Right motor coefficients
    ]
    
    # Initialize Webots robot instance
    robot = Robot()
    
    # Get motor devices and configure them
    front_left_motor = robot.getDevice("fl_wheel_joint")
    front_right_motor = robot.getDevice("fr_wheel_joint")
    rear_left_motor = robot.getDevice("rl_wheel_joint")
    rear_right_motor = robot.getDevice("rr_wheel_joint")
    
    # Set motors to velocity control mode
    for motor in [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]:
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
    
    # Initialize and enable distance sensors
    distance_sensors[0] = robot.getDevice("fl_range")  # Front left
    distance_sensors[1] = robot.getDevice("rl_range")  # Rear left
    distance_sensors[2] = robot.getDevice("fr_range")  # Front right
    distance_sensors[3] = robot.getDevice("rr_range")  # Rear right
    
    for sensor in distance_sensors:
        sensor.enable(TIME_STEP)
    
    # 180-degree turn control variables
    turn_180_counter = 0
    TURN_STEPS_DURATION = 5  # Adjust based on actual turning performance
    DEAD_END_LIMIT = 0.6     # Closer threshold for dead end detection
    
    # Main control loop
    while robot.step(TIME_STEP) != -1:
        # Read distance sensor values
        for i in range(4):
            distance_sensors_value[i] = distance_sensors[i].getValue()
        
        fl_val = distance_sensors_value[0]  # Front left distance
        fr_val = distance_sensors_value[2]  # Front right distance
        
        # Priority 1: Execute 180-degree turn if in turn mode
        if turn_180_counter > 0:
            turn_180_counter -= 1
            # Set rotation speeds (left wheels reverse, right wheels forward)
            motor_speed[0] = -MAX_VELOCITY * 0.8
            motor_speed[1] = MAX_VELOCITY * 0.8
            
            if turn_180_counter == 0:
                print(">>> Turn completed, resuming normal operation")
        
        else:
            # Check for dead end condition (both front sensors very close)
            if fl_val < DEAD_END_LIMIT and fr_val < DEAD_END_LIMIT:
                print("!!! Dead end detected -> initiating 180-degree turn !!!")
                turn_180_counter = TURN_STEPS_DURATION
            
            else:
                # Normal operation: Braitenberg obstacle avoidance
                for i in range(2):
                    avoidance_speed[i] = 0.0
                    for j in range(4):
                        # Calculate distance factor (closer objects have stronger influence)
                        distance_factor = (2.0 - distance_sensors_value[j])
                        avoidance_speed[i] += (distance_factor **2) * coefficients[i][j]
                    motor_speed[i] = base_speed + avoidance_speed[i]
                
                # Fine-tuning: Additional turn if both front sensors detect obstacles
                if fl_val < COLLISION_THRESHOLD and fr_val < COLLISION_THRESHOLD:
                    # Turn towards the side with more distance
                    if abs(fl_val - fr_val) > 0.2:
                        if fl_val > fr_val:
                            motor_speed[0] = -MAX_VELOCITY * 0.5
                            motor_speed[1] = MAX_VELOCITY * 0.5
                        else:
                            motor_speed[0] = MAX_VELOCITY * 0.5
                            motor_speed[1] = -MAX_VELOCITY * 0.5
        
        # Apply speed limits
        motor_speed[0] = max(-MAX_VELOCITY, min(motor_speed[0], MAX_VELOCITY))
        motor_speed[1] = max(-MAX_VELOCITY, min(motor_speed[1], MAX_VELOCITY))
        
        # Set motor velocities
        front_left_motor.setVelocity(motor_speed[0])
        front_right_motor.setVelocity(motor_speed[1])
        rear_left_motor.setVelocity(motor_speed[0])
        rear_right_motor.setVelocity(motor_speed[1])

if __name__ == "__main__":
    main()