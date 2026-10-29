"""
Configuration parameters for the drone system.
Contains preset paths, timing parameters, and communication settings.
"""

# System parameters
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 50  # Hz
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY

# Communication parameters
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQUENCY = 400000  # 400 kHz
FCM_I2C_ADDRESS = 0x08
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
SERVER_IP = "192.168.4.1"
SERVER_PORT = 8080

# Task parameters
TAKEOFF_HEIGHT = 2.8  # meters (ceiling light height)
CRUISING_HEIGHT = 2.7  # meters
OBSTACLE_AVOIDANCE_HEIGHT = 2.6  # meters (below beams but above 2.5m)
MINIMUM_SAFE_HEIGHT = 2.5  # meters
LANDING_HEIGHT = 0.1  # meters
PICKUP_HEIGHT = 0.2  # meters (height for package pickup)
DROPOFF_HEIGHT = 0.2  # meters (height for package dropoff)
MISSION_COMPLETE_WAIT_TIME = 15  # seconds (wait time before transmission)

# Preset path coordinates (x, y, z) in meters from starting point
# Coordinates represent waypoints for the drone to follow
PATH_WAYPOINTS = [
    # Task 1: Takeoff
    (0, 0, TAKEOFF_HEIGHT),  # Starting point, ascend to ceiling light height
    
    # Task 2: Tunnelling through three lights
    (2, 0, TAKEOFF_HEIGHT),  # First light entry
    (4, 0, TAKEOFF_HEIGHT),  # Second light entry
    (6, 0, TAKEOFF_HEIGHT),  # Third light entry
    (8, 0, TAKEOFF_HEIGHT),  # Exit from third light
    
    # Task 3: Obstacle avoidance (turn right after tunnel)
    (8, 2, OBSTACLE_AVOIDANCE_HEIGHT),  # Turn right
    (8, 4, OBSTACLE_AVOIDANCE_HEIGHT),  # Passing first beam
    (8, 6, OBSTACLE_AVOIDANCE_HEIGHT),  # Passing second beam
    
    # Task 4: Search and pickup (red box area)
    (6, 6, CRUISING_HEIGHT),  # Approach red box area
    (6, 6, PICKUP_HEIGHT),    # Descend for pickup
    (6, 6, CRUISING_HEIGHT),  # Ascend after pickup
    
    # Task 5: Search and dropoff (green box area)
    (4, 8, CRUISING_HEIGHT),  # Approach green box area
    (4, 8, DROPOFF_HEIGHT),   # Descend for dropoff
    (4, 8, CRUISING_HEIGHT),  # Ascend after dropoff
    
    # Task 6: Landing
    (4, 8, LANDING_HEIGHT),   # Final landing
]

# Task transition points (indices into PATH_WAYPOINTS)
TASK_TRANSITION_INDICES = {
    "TAKEOFF_COMPLETE": 1,
    "TUNNELLING_COMPLETE": 4,
    "OBSTACLE_AVOIDANCE_COMPLETE": 8,
    "PICKUP_COMPLETE": 11,
    "DROPOFF_COMPLETE": 14,
    "LANDING_COMPLETE": 15,
}

# Sensor thresholds
HEIGHT_THRESHOLD = 0.05  # meters (tolerance for height measurements)
POSITION_THRESHOLD = 0.2  # meters (tolerance for position)

# LED patterns for status indication
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # Fast blink 3 times
    "TAKEOFF": [(1, 0.5), (0, 0.5)],      # Slow blink
    "TUNNELLING": [(1, 0.2), (0, 0.2)],   # Medium blink
    "OBSTACLE_AVOIDANCE": [(1, 0.1), (0, 0.1), (1, 0.1), (0, 0.5)],  # Double blink
    "PICKUP": [(1, 0.1), (0, 0.1), (1, 0.1), (0, 0.1), (1, 0.1), (0, 0.5)],  # Triple blink
    "DROPOFF": [(1, 1), (0, 0.5)],        # Long blink
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,  # Fast blink 5 times
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,   # Rapid blink 10 times
    "MISSION_COMPLETE": [(1, 2)],         # Solid on
}

# FCM command codes
FCM_COMMANDS = {
    "MOTORS_OFF": 0x00,
    "MOTORS_ON": 0x01,
    "SET_POSITION": 0x02,
    "SET_VELOCITY": 0x03,
    "SET_ATTITUDE": 0x04,
    "SET_ALTITUDE": 0x05,
    "EMERGENCY_STOP": 0xFF,
}
# Keep deployment credentials in an ignored, board-local file.
try:
    from config_local import WIFI_SSID, WIFI_PASSWORD
except ImportError:
    pass
