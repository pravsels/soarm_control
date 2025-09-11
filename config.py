# config.py
UIDS =  [1,2,3,4,5,6] 

DEFAULT_PORT = "/dev/ttyACM0"

MOTOR_RESOLUTION = 4096

# Gear ratios: joint_rotation / motor_rotation
# e.g., 1/345 means 1 joint rotation = 345 motor rotations
GEAR_RATIOS = {
    "so101_leader": {
        1: 1/191,   # Base/Shoulder Pan
        2: 1/345,   # Shoulder Lift  
        3: 1/191,   # Elbow Flex
        4: 1/147,   # Wrist Flex
        5: 1/147,   # Wrist Roll
        6: 1/147,   # Gripper
    },
    "so101_follower": {
        1: 1/345,   # Base/Shoulder Pan
        2: 1/345,   # Shoulder Lift  
        3: 1/345,   # Elbow Flex
        4: 1/345,   # Wrist Flex
        5: 1/345,   # Wrist Roll
        6: 1/345,   # Gripper
    }
}

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
