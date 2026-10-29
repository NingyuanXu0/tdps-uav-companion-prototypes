"""
Drone system configuration parameters.
Contains preset paths, timing parameters, and communication settings.
Optimized for skipping task 1 and focusing on preset path navigation.
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

# HC12 communication parameters
HC12_TX_PIN = 17  # UART TX pin connected to HC12 RX
HC12_RX_PIN = 16  # UART RX pin connected to HC12 TX 
HC12_SET_PIN = 23  # Pin connected to HC12 SET pin for AT command mode
HC12_BAUD_RATE = 9600
HC12_CHANNEL = 1  # Channel 1-127, team-specific channel will be provided

# Sensor parameters
VL53L0X_SCL_PIN = 5
VL53L0X_SDA_PIN = 4
HCSR04_TRIGGER_PIN = 12
HCSR04_ECHO_PIN = 13
SENSOR_READ_RETRIES = 3

# Battery monitoring
BATTERY_ADC_PIN = 34
LOW_BATTERY_THRESHOLD = 14.0  # Volts
CRITICAL_BATTERY_THRESHOLD = 13.5  # Volts

# Task parameters
TAKEOFF_HEIGHT = 2.8  # meters (ceiling light fixture height)
CRUISING_HEIGHT = 2.7  # meters
OBSTACLE_AVOIDANCE_HEIGHT = 2.6  # meters (below beams but above 2.5m)
MINIMUM_SAFE_HEIGHT = 2.5  # meters
LANDING_HEIGHT = 0.1  # meters
PICKUP_HEIGHT = 0.2  # meters (package pickup height)
DROPOFF_HEIGHT = 0.2  # meters (package drop-off height)
MISSION_COMPLETE_WAIT_TIME = 15  # seconds (wait time before transmission)

# Preset path waypoints (x, y, z), referenced from starting point
# Coordinates represent the waypoints the drone should follow
PATH_WAYPOINTS = [
    # Task 1: Take-off (preserved for reference)
    (0, 0, TAKEOFF_HEIGHT),  # Starting point, rise to ceiling light height
    
    # Task 2: Tunneling (skipped but waypoints preserved for reference)
    # These points will be used as transition waypoints
    (2, 0, TAKEOFF_HEIGHT),  
    (4, 0, TAKEOFF_HEIGHT),  
    (6, 0, TAKEOFF_HEIGHT),  
    (8, 0, TAKEOFF_HEIGHT),  
    
    # Task 3: Obstacle avoidance (after the right turn)
    (8, -2, OBSTACLE_AVOIDANCE_HEIGHT),  # 左转
    (8, -4, OBSTACLE_AVOIDANCE_HEIGHT),  # 通过第一个横梁
    (8, -6, OBSTACLE_AVOIDANCE_HEIGHT),  # 通过第二个横梁
    
    # Task 4: Package search and pickup (red box area)
    (6, -6, CRUISING_HEIGHT),  # Approach red box area
    (6, -6, PICKUP_HEIGHT),    # Descend for pickup
    (6, -6, CRUISING_HEIGHT),  # Ascend after pickup
    
    # Task 5: Marker search and package drop-off (green box area)
    (4, -8, CRUISING_HEIGHT),  # Approach green box area
    (4, -8, DROPOFF_HEIGHT),   # Descend for drop-off
    (4, -8, CRUISING_HEIGHT),  # Ascend after drop-off
    
    # Task 6: Landing
    (4, -8, LANDING_HEIGHT),   # Final landing
]

# Task transition indices (indices in PATH_WAYPOINTS)
TASK_TRANSITION_INDICES = {
    "TAKEOFF_COMPLETE": 1,    # Preserved for reference
    "TUNNELLING_COMPLETE": 4, # Preserved but used as entry to obstacle avoidance
    "OBSTACLE_AVOIDANCE_COMPLETE": 8,
    "PICKUP_COMPLETE": 11,
    "DROPOFF_COMPLETE": 14,
    "LANDING_COMPLETE": 15,
}

# Sensor thresholds
HEIGHT_THRESHOLD = 0.05  # meters (height measurement tolerance)
POSITION_THRESHOLD = 0.2  # meters (position tolerance)

# LED indicator blink patterns
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # Quick blink 3 times
    "WAITING_FOR_START": [(1, 0.1), (0, 1.9)],  # Very slow blink
    "MISSION_START": [(1, 0.2), (0, 0.2)] * 5,  # Medium blink 5 times
    "TAKEOFF": [(1, 0.5), (0, 0.5)],      # Slow blink
    "TUNNELLING": [(1, 0.2), (0, 0.2)],   # Medium blink
    "OBSTACLE_AVOIDANCE": [(1, 0.1), (0, 0.1), (1, 0.1), (0, 0.5)],  # Double blink
    "PICKUP": [(1, 0.1), (0, 0.1), (1, 0.1), (0, 0.1), (1, 0.1), (0, 0.5)],  # Triple blink
    "DROPOFF": [(1, 1), (0, 0.5)],        # Long blink
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,  # Quick blink 5 times
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,   # Quick blink 10 times
    "MANUAL_INTERVENTION": [(1, 0.5), (0, 0.1), (1, 0.1), (0, 0.1)],  # Long + two short
    "CONTINUE": [(1, 1), (0, 0.1), (1, 1)],  # Two long blinks
    "MISSION_COMPLETE": [(1, 2)],         # Sustained on
}

# FCM command codes
FCM_COMMANDS = {
    "MOTORS_OFF": 0x00,
    "MOTORS_ON": 0x01,
    "SET_POSITION": 0x02,
    "SET_VELOCITY": 0x03,
    "SET_ATTITUDE": 0x04,
    "SET_ALTITUDE": 0x05,
    "RC_OVERRIDE": 0x06,
    "SET_MODE": 0x07,
    "TAKEOFF": 0x08,
    "LAND": 0x09,
    "GET_STATUS": 0x0A,
    "HEARTBEAT": 0x0B,
    "EMERGENCY_STOP": 0xFF,
}
# Set the measured voltage-divider ratio in ignored config_local.py.
BATTERY_VOLTAGE_DIVIDER = None
try:
    from config_local import BATTERY_VOLTAGE_DIVIDER
except ImportError:
    pass
