"""
Utility functions for the drone system.
Contains helper functions for timing, logging, and LED control.
"""
import time
import machine
import uos
from config import LED_PATTERNS, DEBUG_MODE

# Initialize built-in LED
led = machine.Pin(2, machine.Pin.OUT)

# Create log file
LOG_FILENAME = "drone_log.txt"
try:
    # Check if log file exists
    log_exists = False
    for file in uos.listdir():
        if file == LOG_FILENAME:
            log_exists = True
            break
    
    # If log file doesn't exist, create it
    if not log_exists:
        with open(LOG_FILENAME, "w") as f:
            f.write("=== 无人机系统日志 ===\n")
except:
    pass  # Continue even if log file creation fails

def current_time_ms():
    """Returns current time in milliseconds"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """Returns time difference in milliseconds"""
    return time.ticks_diff(time.ticks_ms(), start_time)

def log(message, level="INFO"):
    """
    Log message with level and timestamp.
    Also writes to log file if available.
    """
    if DEBUG_MODE or level in ["ERROR", "WARNING"]:
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        log_message = "[{}] {}: {}".format(time_str, level, message)
        print(log_message)
        
        # Write to log file if possible
        try:
            with open(LOG_FILENAME, "a") as f:
                f.write(log_message + "\n")
        except:
            pass  # Continue even if writing to log fails

def display_led_pattern(pattern_name, duration=None):
    """
    Display LED pattern for status indication.
    
    Args:
        pattern_name: Name of the LED pattern from LED_PATTERNS
        duration: Optional duration in seconds to display pattern
    """
    if pattern_name not in LED_PATTERNS:
        log(f"未知的LED模式: {pattern_name}", "WARNING")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    start_time = current_time_ms()
    
    while True:
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
            
        # Check if we should exit based on duration
        if duration is None or time_diff_ms(start_time) >= duration * 1000:
            break

def format_mission_complete_message():
    """Format mission complete message with timestamp"""
    timestamp = time.localtime()
    time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        timestamp[0], timestamp[1], timestamp[2],
        timestamp[3], timestamp[4], timestamp[5]
    )
    return f"任务完成于 {time_str}"

def vector_distance(pos1, pos2):
    """Calculate Euclidean distance between two 3D points"""
    return ((pos1[0] - pos2[0])**2 + 
            (pos1[1] - pos2[1])**2 + 
            (pos1[2] - pos2[2])**2) ** 0.5

def is_position_reached(current_pos, target_pos, threshold):
    """Check if current position is within threshold of target"""
    return vector_distance(current_pos, target_pos) < threshold

def calculate_velocity_vector(current_pos, target_pos, max_speed):
    """Calculate velocity vector towards target with max speed limit"""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    dz = target_pos[2] - current_pos[2]
    
    # Calculate distance
    distance = (dx**2 + dy**2 + dz**2) ** 0.5
    
    if distance < 0.001:  # Prevent division by near-zero
        return (0, 0, 0)
    
    # Normalize and scale by max_speed
    scale = min(max_speed, distance) / distance
    return (dx * scale, dy * scale, dz * scale)

def apply_moving_average(values, new_value, window_size=5):
    """
    Apply moving average filter to a list of values.
    
    Args:
        values: List of previous values
        new_value: New value to add
        window_size: Size of the moving average window
    
    Returns:
        Updated list of values and the filtered value
    """
    # Add new value to list
    values.append(new_value)
    
    # Keep only the last window_size values
    if len(values) > window_size:
        values = values[-window_size:]
    
    # Calculate average
    filtered_value = sum(values) / len(values)
    
    return values, filtered_value