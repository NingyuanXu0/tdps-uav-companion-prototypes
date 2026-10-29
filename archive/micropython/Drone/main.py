"""
Main program entry point for the ACM core.
Initializes all system components and runs the main control loop.
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, DEBUG_MODE
from utils import log, display_led_pattern
from perception import PerceptionSystem
from communication import FCMInterface, WirelessCommunication
from task_manager import TaskManager, TaskState

def initialize_system():
    """Initialize all system components"""
    log("Initializing system components")
    
    # Initialize interfaces
    fcm = FCMInterface()
    perception = PerceptionSystem()
    wireless = WirelessCommunication()
    
    # Initialize task manager
    task_manager = TaskManager(fcm, perception, wireless)
    
    # Perform system checks
    if not fcm.initialized:
        log("FCM interface initialization failed", "ERROR")
        return None
    
    log("System components initialized successfully")
    return task_manager

def main():
    """Main program entry point"""
    log("ACM Core starting")
    
    # Configure watchdog timer for safety
    wdt = machine.WDT(timeout=10000)  # 10 second timeout
    
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
    
    log("ACM Core initialized, entering main loop")
    last_loop_time = time.ticks_ms()
    
    # Main control loop
    try:
        while True:
            # Feed watchdog
            wdt.feed()
            
            # Run garbage collection periodically
            gc.collect()
            
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
    
    finally:
        log("ACM Core shutting down")
        # Perform cleanup
        if task_manager.state != TaskState.ERROR:
            display_led_pattern("MISSION_COMPLETE", 5)

if __name__ == "__main__":
    main()