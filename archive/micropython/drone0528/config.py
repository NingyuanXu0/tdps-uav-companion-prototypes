"""
Drone System Configuration - Enhanced Motor Test Version
Configuration optimized for safe low-speed propeller testing
"""

# System parameters
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 20  # Hz (increased for better responsiveness)
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY

# UART Communication parameters
UART_TX_PIN = 17  # ESP32 TX → PX4 RX (TELEM2)
UART_RX_PIN = 16  # ESP32 RX ← PX4 TX (TELEM2)
UART_BAUDRATE = 57600  # Standard MAVLink baudrate

# Enhanced Motor test parameters for low-speed operation
MOTOR_TEST_DURATION = 3.0  # seconds per individual motor test
MOTOR_TEST_ALL_DURATION = 5.0  # seconds for all motors test
MOTOR_TEST_THROTTLE_VERY_LOW = 5   # percentage - very gentle start
MOTOR_TEST_THROTTLE_LOW = 10       # percentage - low speed test
MOTOR_TEST_THROTTLE_MEDIUM = 15    # percentage - medium speed test
MOTOR_TEST_THROTTLE_MAX = 20       # percentage - maximum for testing (safety limit)
MOTOR_TEST_SEQUENCE_DELAY = 2.0    # delay between motor tests
MOTOR_RAMP_UP_TIME = 1.0           # gradual throttle increase time

# Safety parameters
ARM_SAFETY_DELAY = 3.0         # seconds before allowing motor operation
MAX_TEST_DURATION = 120.0      # maximum total test duration in seconds
AUTO_DISARM_TIMEOUT = 45.0     # auto-disarm if no commands sent
CONNECTION_TIMEOUT = 10.0      # PX4 connection timeout
HEARTBEAT_INTERVAL = 1.0       # heartbeat sending interval
MAX_RETRIES = 3               # maximum command retries

# Motor test sequence configuration
MOTOR_TEST_SEQUENCE = [
    # (throttle_percent, duration_seconds, description)
    (MOTOR_TEST_THROTTLE_VERY_LOW, 2.0, "Very Low Speed"),
    (MOTOR_TEST_THROTTLE_LOW, 3.0, "Low Speed"),
    (MOTOR_TEST_THROTTLE_MEDIUM, 2.0, "Medium Speed"),
]

# All motors test configuration
ALL_MOTORS_TEST_SEQUENCE = [
    (MOTOR_TEST_THROTTLE_VERY_LOW, 3.0, "All Motors - Very Low"),
    (MOTOR_TEST_THROTTLE_LOW, 4.0, "All Motors - Low Speed"),
]

# LED patterns for status indication
LED_PATTERNS = {
    "STARTUP": [(1, 0.2), (0, 0.2)] * 3,
    "CONNECTING": [(1, 0.1), (0, 0.1)],
    "CONNECTED": [(1, 0.5), (0, 2.0)],
    "ARMED": [(1, 0.3), (0, 0.3)],
    "MOTOR_TEST": [(1, 0.15), (0, 0.15)],
    "DISARMED": [(1, 0.1), (0, 1.9)],
    "ERROR": [(1, 0.05), (0, 0.05)] * 20,
    "TEST_COMPLETE": [(1, 3.0)],
    "EMERGENCY": [(1, 0.02), (0, 0.02)] * 50,
}

# Control pins
ABORT_BUTTON_PIN = 14      # GPIO14 for emergency stop (active low)
START_BUTTON_PIN = 15      # GPIO15 for test start (active low)
BUTTON_DEBOUNCE_MS = 200   # button debounce time

# PX4 specific settings
PX4_SYSTEM_ID = 1          # PX4 system ID
PX4_COMPONENT_ID = 1       # PX4 autopilot component ID
GROUND_STATION_ID = 255    # Our system ID

# Test modes
TEST_MODES = {
    "INDIVIDUAL": 1,   # Test each motor individually
    "ALL_TOGETHER": 2, # Test all motors together
    "SEQUENCE": 3,     # Test with predefined sequence
    "MANUAL": 4        # Manual control mode
}

# Default test mode
DEFAULT_TEST_MODE = TEST_MODES["SEQUENCE"]

# Motor mapping (for different frame types)
MOTOR_MAPPING = {
    "QUAD_X": {
        1: "Front Right",
        2: "Back Left", 
        3: "Front Left",
        4: "Back Right"
    },
    "QUAD_PLUS": {
        1: "Front",
        2: "Right",
        3: "Back", 
        4: "Left"
    }
}

# Default frame type
DEFAULT_FRAME_TYPE = "QUAD_X"

# Logging configuration
LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR
MAX_LOG_SIZE = 10000  # Maximum log file size in bytes
LOG_ROTATION = True   # Enable log rotation

# System health checks
HEALTH_CHECK_INTERVAL = 5.0  # seconds
MIN_FREE_MEMORY = 30000     # bytes
TEMPERATURE_WARNING = 70    # degrees C (if available)

# MAVLink specific settings
MAVLINK_RETRY_COUNT = 3
MAVLINK_RETRY_DELAY = 0.5   # seconds
MAVLINK_TIMEOUT = 2.0       # seconds

# Pre-flight safety checks
SAFETY_CHECKS = {
    "CONNECTION_REQUIRED": True,
    "HEARTBEAT_REQUIRED": True,
    "BATTERY_CHECK": False,     # Set to True if battery monitoring available
    "GPS_CHECK": False,         # Not required for motor testing
    "COMPASS_CHECK": False,     # Not required for motor testing
    "ACCELEROMETER_CHECK": False # Not required for motor testing
}