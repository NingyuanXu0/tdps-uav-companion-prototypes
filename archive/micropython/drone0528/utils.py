"""
Enhanced Drone System Utilities - Motor Test Version
Improved utilities for motor testing with better LED patterns and logging
"""
import time
import machine
import gc
from config import DEBUG_MODE, LED_PATTERNS

# Initialize built-in LED
led = machine.Pin(2, machine.Pin.OUT)

# LED pattern state
_led_pattern_active = False
_led_pattern_start = 0
_led_pattern_index = 0
_current_pattern = None

# Create log file with timestamp
LOG_FILENAME = "motor_test_log.txt"
try:
    with open(LOG_FILENAME, "w") as f:
        timestamp = time.localtime()
        f.write(f"=== Motor Test Log Started {timestamp[0]}-{timestamp[1]:02d}-{timestamp[2]:02d} ===\n")
except:
    pass

def log(message, level="INFO"):
    """
    Enhanced log message with level and timestamp
    """
    if DEBUG_MODE or level in ["ERROR", "WARNING"]:
        try:
            timestamp = time.localtime()
            time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
                timestamp[0], timestamp[1], timestamp[2],
                timestamp[3], timestamp[4], timestamp[5]
            )
            
            # Add memory info for DEBUG level
            if level == "DEBUG":
                free_mem = gc.mem_free()
                message = f"{message} [Mem: {free_mem}]"
            
            log_message = "[{}] {}: {}".format(time_str, level, message)
            print(log_message)
            
            # Write to log file with rotation
            try:
                # Check file size and rotate if needed
                try:
                    with open(LOG_FILENAME, "r") as f:
                        size = len(f.read())
                    if size > 50000:  # 50KB limit
                        with open(LOG_FILENAME + ".old", "w") as old_f:
                            with open(LOG_FILENAME, "r") as f:
                                old_f.write(f.read())
                        with open(LOG_FILENAME, "w") as f:
                            f.write("=== Log Rotated ===\n")
                except:
                    pass
                
                with open(LOG_FILENAME, "a") as f:
                    f.write(log_message + "\n")
            except:
                pass
                
        except Exception as e:
            print(f"Logging error: {e}")

def set_led(state):
    """
    Set LED state and stop any active pattern
    
    Parameters:
        state: 0=off, 1=on
    """
    global _led_pattern_active
    _led_pattern_active = False
    led.value(state)

def blink_led_once(on_time=0.1, off_time=0.1):
    """
    Blink LED once - blocking
    """
    led.value(1)
    time.sleep(on_time)
    led.value(0)
    time.sleep(off_time)

def blink_led_pattern(pattern_name):
    """
    Start a non-blocking LED pattern
    """
    global _led_pattern_active, _led_pattern_start, _led_pattern_index, _current_pattern
    
    if pattern_name in LED_PATTERNS:
        _current_pattern = LED_PATTERNS[pattern_name]
        _led_pattern_active = True
        _led_pattern_start = time.ticks_ms()
        _led_pattern_index = 0
        log(f"Starting LED pattern: {pattern_name}", "DEBUG")

def update_led_pattern():
    """
    Update LED pattern - call this regularly from main loop
    """
    global _led_pattern_active, _led_pattern_start, _led_pattern_index, _current_pattern
    
    if not _led_pattern_active or not _current_pattern:
        return
    
    current_time = time.ticks_ms()
    
    # Calculate total pattern time elapsed
    pattern_elapsed = 0
    for i, (state, duration) in enumerate(_current_pattern):
        if i < _led_pattern_index:
            pattern_elapsed += duration * 1000  # Convert to ms
    
    # Calculate time in current step
    step_elapsed = time.ticks_diff(current_time, _led_pattern_start) - pattern_elapsed
    
    if _led_pattern_index < len(_current_pattern):
        state, duration = _current_pattern[_led_pattern_index]
        
        if step_elapsed < duration * 1000:  # Still in current step
            led.value(state)
        else:  # Move to next step
            _led_pattern_index += 1
            if _led_pattern_index >= len(_current_pattern):
                # Pattern complete, repeat
                _led_pattern_index = 0
                _led_pattern_start = current_time

def stop_led_pattern():
    """Stop any active LED pattern"""
    global _led_pattern_active
    _led_pattern_active = False
    led.value(0)

def display_startup_pattern():
    """Display startup pattern (3 quick blinks)"""
    log("Displaying startup pattern")
    for _ in range(3):
        blink_led_once(0.15, 0.15)
    time.sleep(0.5)

def display_error_pattern():
    """Display error pattern (rapid blinking)"""
    log("Displaying error pattern")
    for _ in range(20):
        blink_led_once(0.05, 0.05)

def display_success_pattern():
    """Display success pattern (3 long blinks)"""
    log("Displaying success pattern")
    for _ in range(3):
        blink_led_once(0.5, 0.3)

def check_system_resources():
    """Enhanced system resource check"""
    try:
        gc.collect()
        free_mem = gc.mem_free()
        allocated_mem = gc.mem_alloc()
        total_mem = free_mem + allocated_mem
        
        # Calculate percentages
        used_percent = (allocated_mem / total_mem) * 100
        
        log(f"Memory - Used: {allocated_mem} bytes ({used_percent:.1f}%), Free: {free_mem} bytes", "DEBUG")
        
        # Warning thresholds
        if free_mem < 30000:  # Less than 30KB free
            log("Critical memory warning", "WARNING")
            return False
        elif free_mem < 50000:  # Less than 50KB free
            log("Low memory warning", "WARNING")
        
        return True
    except Exception as e:
        log(f"Resource check error: {e}", "ERROR")
        return False

def format_time_duration(seconds):
    """Format duration in human readable format"""
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.1f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        return f"{hours}h {minutes}m"

def safe_file_write(filename, content, mode='w'):
    """Safely write to file with error handling"""
    try:
        with open(filename, mode) as f:
            f.write(content)
        return True
    except Exception as e:
        log(f"File write error ({filename}): {e}", "ERROR")
        return False

def safe_file_read(filename):
    """Safely read from file with error handling"""
    try:
        with open(filename, 'r') as f:
            return f.read()
    except Exception as e:
        log(f"File read error ({filename}): {e}", "ERROR")
        return None

def get_system_info():
    """Get system information"""
    try:
        import os
        import sys
        
        info = {
            'platform': sys.platform,
            'version': sys.version,
            'implementation': sys.implementation.name if hasattr(sys, 'implementation') else 'unknown',
            'free_memory': gc.mem_free(),
            'allocated_memory': gc.mem_alloc()
        }
        
        try:
            # Try to get additional ESP32 specific info
            import esp32
            info['hall_sensor'] = esp32.hall_sensor()
            info['raw_temperature'] = esp32.raw_temperature()
        except:
            pass
        
        return info
    except Exception as e:
        log(f"System info error: {e}", "ERROR")
        return {}

def watchdog_feed():
    """Feed watchdog if available"""
    try:
        # This will be called from main loop if watchdog is enabled
        pass
    except:
        pass

def emergency_cleanup():
    """Perform emergency cleanup operations"""
    log("Performing emergency cleanup", "WARNING")
    
    try:
        # Stop LED patterns
        stop_led_pattern()
        set_led(0)
        
        # Force garbage collection
        gc.collect()
        
        # Close any open files (not easily doable in MicroPython)
        
        # Save emergency log entry
        safe_file_write("emergency.log", f"Emergency cleanup at {time.ticks_ms()}\n", 'a')
        
        log("Emergency cleanup completed")
        return True
    except Exception as e:
        print(f"Emergency cleanup failed: {e}")
        return False

# Initialize system
def initialize_utils():
    """Initialize utilities subsystem"""
    log("Utilities initialized")
    log(f"System info: {get_system_info()}")
    return check_system_resources()