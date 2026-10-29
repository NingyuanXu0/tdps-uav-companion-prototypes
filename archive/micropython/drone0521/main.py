"""
ACM core's main program entry point.
Initializes all system components and runs the main control loop.
Modified to skip task 1 (takeoff) and start from task 3 (obstacle avoidance).
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, DEBUG_MODE
from utils import log, display_led_pattern
from perception import PerceptionSystem
from communication import FCMInterface
from task_manager import TaskManager, TaskState

def initialize_system():
    """Initialize all system components"""
    log("Initializing system components")
    
    # Initialize interfaces
    fcm = FCMInterface()
    perception = PerceptionSystem()
    
    # Initialize task manager
    task_manager = TaskManager(fcm, perception)
    
    # Perform system check
    if not fcm.initialized:
        log("FCM interface initialization failed", "ERROR")
        return None
    
    log("System components initialized successfully")
    return task_manager

def main():
    """
    Main program entry point - modified to skip tunneling.
    """
    log("ACM core starting up")
    
    # Configure watchdog timer for safety
    wdt = machine.WDT(timeout=15000)  # 15 second timeout
    
    # Initialize system components
    task_manager = initialize_system()
    if task_manager is None:
        log("System initialization failed, aborting", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # Initialize task manager
    if not task_manager.initialize():
        log("Task manager initialization failed, aborting", "ERROR")
        display_led_pattern("ERROR")
        return
    
    log("ACM core initialization complete, entering main loop")
    last_loop_time = time.ticks_ms()
    
    # Main control loop
    try:
        # Wait for operator confirmation to start
        display_led_pattern("WAITING_FOR_START")
        log("Waiting for start button press...")
        
        # Use GPIO pin as start button
        start_button = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
        while start_button.value() == 1:  # Button is active low
            wdt.feed()
            time.sleep(0.1)
        
        log("Start button pressed, mission beginning")
        display_led_pattern("MISSION_START", 2)
            
        while True:
            # Feed watchdog
            wdt.feed()
            
            # Periodically run garbage collection
            gc.collect()
            
            # Update FCM heartbeat
            task_manager.fcm.update()
            
            # Check manual abort button
            if hasattr(task_manager, 'abort_button') and task_manager.abort_button.value() == 0:
                log("Manual abort triggered")
                task_manager.abort_mission()
                break
            
            # Check battery voltage
            battery_voltage = task_manager.get_battery_voltage()
            if battery_voltage < task_manager.LOW_BATTERY_THRESHOLD:
                log(f"Low battery: {battery_voltage:.2f}V", "WARNING")
                if battery_voltage < task_manager.CRITICAL_BATTERY_THRESHOLD:
                    log("Critical battery level, emergency landing", "ERROR")
                    task_manager.emergency_landing()
                    break
            
            # Update task manager
            if not task_manager.update():
                log("Task manager signaled exit")
                break
            
            # Calculate loop time and sleep to maintain consistent frequency
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            if DEBUG_MODE:
                log(f"Loop time: {loop_time * 1000:.2f}ms")
            
            # Sleep to maintain loop frequency
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        log(f"Unhandled exception in main loop: {e}", "ERROR")
        display_led_pattern("ERROR")
        
        # Try to perform emergency landing (if possible)
        try:
            task_manager.emergency_landing()
        except:
            # Last resort - try to stop motors
            try:
                task_manager.fcm.emergency_stop()
            except:
                pass
    
    finally:
        log("ACM core shutting down")
        # Perform cleanup
        if task_manager.state != TaskState.ERROR:
            display_led_pattern("MISSION_COMPLETE", 5)

if __name__ == "__main__":
    main()