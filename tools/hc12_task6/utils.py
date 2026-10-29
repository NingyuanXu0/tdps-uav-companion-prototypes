"""
Drone system utility functions.
Contains helper functions for timing, logging, and LED control.
Optimized for preset path following.
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
    
    # Create log file if it doesn't exist
    if not log_exists:
        with open(LOG_FILENAME, "w") as f:
            f.write("=== Drone System Log ===\n")
except:
    pass  # Continue even if log file creation fails

def current_time_ms():
    """Return current time (milliseconds)"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """Return time difference (milliseconds)"""
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
        
        # Write to log file (if possible)
        try:
            with open(LOG_FILENAME, "a") as f:
                f.write(log_message + "\n")
        except:
            pass  # Continue even if log write fails

def display_led_pattern(pattern_name, duration=None):
    """
    Display LED blink pattern to indicate status.
    
    Args:
        pattern_name: Pattern name from LED_PATTERNS
        duration: Optional display duration in seconds
    """
    if pattern_name not in LED_PATTERNS:
        log(f"Unknown LED pattern: {pattern_name}", "WARNING")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    start_time = current_time_ms()
    
    while True:
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
            
        # Check if should exit based on duration
        if duration is not None and time_diff_ms(start_time) >= duration * 1000:
            break

def format_mission_complete_message():
    """Format mission complete message with timestamp"""
    timestamp = time.localtime()
    time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        timestamp[0], timestamp[1], timestamp[2],
        timestamp[3], timestamp[4], timestamp[5]
    )
    return f"Mission completed at {time_str}"

def vector_distance(pos1, pos2):
    """Calculate Euclidean distance between two 3D points"""
    return ((pos1[0] - pos2[0])**2 + 
            (pos1[1] - pos2[1])**2 + 
            (pos1[2] - pos2[2])**2) ** 0.5

def is_position_reached(current_pos, target_pos, threshold):
    """
    Check if current position is within threshold of target - 
    optimized for preset path navigation with looser XY constraints.
    
    Args:
        current_pos: Current [x, y, z] position
        target_pos: Target [x, y, z] position
        threshold: Distance threshold in meters
    
    Returns:
        True if position is reached
    """
    # Use a more forgiving horizontal threshold but stricter vertical
    xy_threshold = threshold * 1.5  # More forgiving for x,y
    z_threshold = threshold * 0.7   # Stricter for height
    
    xy_distance = ((current_pos[0] - target_pos[0])**2 + 
                   (current_pos[1] - target_pos[1])**2) ** 0.5
    z_distance = abs(current_pos[2] - target_pos[2])
    
    return xy_distance < xy_threshold and z_distance < z_threshold

def calculate_velocity_vector(current_pos, target_pos, max_speed):
    """
    Calculate velocity vector toward target with max speed limit -
    optimized for smoother transitions along preset path.
    
    Args:
        current_pos: Current [x, y, z] position
        target_pos: Target [x, y, z] position
        max_speed: Maximum speed limit
    
    Returns:
        Velocity vector [vx, vy, vz]
    """
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    dz = target_pos[2] - current_pos[2]
    
    # Calculate distance
    distance = (dx**2 + dy**2 + dz**2) ** 0.5
    
    if distance < 0.001:  # Prevent division by near-zero
        return (0, 0, 0)
    
    # Use variable speed based on distance to target
    # Slow down as we approach target for smoother transitions
    if distance < 0.5:
        # Progressive slowdown as we approach target
        scale = min(max_speed * distance * 2, max_speed) / distance
    else:
        # Normal speed for longer distances
        scale = min(max_speed, distance) / distance
    
    # Height corrections should happen faster
    vz = dz * scale * 1.2  # 20% faster height adjustment
    
    return (dx * scale, dy * scale, vz)

def apply_moving_average(values, new_value, window_size=5):
    """
    Apply moving average filter to a list of values.
    
    Args:
        values: List of previous values
        new_value: New value to add
        window_size: Moving average window size
    
    Returns:
        Updated values list and filtered value
    """
    # Add new value to list
    values.append(new_value)
    
    # Keep only the last window_size values
    if len(values) > window_size:
        values = values[-window_size:]
    
    # Calculate average
    filtered_value = sum(values) / len(values)
    
    return values, filtered_value