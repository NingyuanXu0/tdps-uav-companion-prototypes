"""
Enhanced ACM Core Main Program - Low-Speed Propeller Test Version
ESP32-S3 MicroPython program for safe motor testing with PX4 flight controller
"""
import time
import gc
import machine
from config import (
    MAIN_LOOP_DELAY, DEBUG_MODE, ARM_SAFETY_DELAY, MAX_TEST_DURATION, 
    ABORT_BUTTON_PIN, START_BUTTON_PIN, BUTTON_DEBOUNCE_MS, TEST_MODES,
    HEARTBEAT_INTERVAL, HEALTH_CHECK_INTERVAL
)
from utils import (
    log, set_led, display_startup_pattern, display_error_pattern, display_success_pattern,
    check_system_resources, update_led_pattern, initialize_utils, emergency_cleanup,
    format_time_duration, get_system_info
)
from communication import FCMInterface
from motor_test import MotorTestManager

class SystemState:
    """System state enumeration"""
    STARTUP = 0
    INITIALIZING = 1
    READY = 2
    TESTING = 3
    ERROR = 4
    SHUTDOWN = 5

class DroneTestSystem:
    """Main drone test system class"""
    
    def __init__(self):
        self.state = SystemState.STARTUP
        self.fcm = None
        self.test_manager = None
        self.wdt = None
        
        # Button handling
        self.start_button = None
        self.abort_button = None
        self.last_start_press = 0
        self.last_abort_press = 0
        
        # System monitoring
        self.last_heartbeat = 0
        self.last_health_check = 0
        self.startup_time = time.time()
        self.test_count = 0
        
        # Error tracking
        self.error_count = 0
        self.last_error_time = 0
        
    def initialize_hardware(self):
        """Initialize hardware components"""
        log("Initializing hardware components")
        
        try:
            # Initialize watchdog timer
            self.wdt = machine.WDT(timeout=15000)  # 15 second timeout
            log("Watchdog timer initialized (15s timeout)")
            
            # Initialize buttons with pull-up resistors
            self.start_button = machine.Pin(START_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
            self.abort_button = machine.Pin(ABORT_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
            log("Control buttons initialized")
            
            # Test LED
            set_led(1)
            time.sleep(0.2)
            set_led(0)
            
            return True
            
        except Exception as e:
            log(f"Hardware initialization failed: {e}", "ERROR")
            return False
    
    def initialize_communication(self):
        """Initialize communication with PX4"""
        log("Initializing PX4 communication interface")
        
        try:
            self.fcm = FCMInterface()
            
            if not self.fcm.initialized:
                log("FCM interface initialization failed", "ERROR")
                return False
            
            log("FCM interface initialized successfully")

            # Initialization leaves the vehicle disarmed; a button starts the test.

            return True
            
        except Exception as e:
            log(f"Communication initialization failed: {e}", "ERROR")
            return False
    
    def initialize_test_manager(self):
        """Initialize motor test manager"""
        log("Initializing motor test manager")
        
        try:
            self.test_manager = MotorTestManager(self.fcm)
            log("Motor test manager initialized")
            return True
            
        except Exception as e:
            log(f"Test manager initialization failed: {e}", "ERROR")
            return False
    
    def initialize_system(self):
        """Initialize all system components"""
        log("=== System Initialization Starting ===")
        
        self.state = SystemState.INITIALIZING
        
        # Initialize utilities
        if not initialize_utils():
            log("Utilities initialization failed", "ERROR")
            return False
        
        # Initialize hardware
        if not self.initialize_hardware():
            log("Hardware initialization failed", "ERROR")
            return False
        
        # Initialize communication
        if not self.initialize_communication():
            log("Communication initialization failed", "ERROR")
            return False
        
        # Initialize test manager
        if not self.initialize_test_manager():
            log("Test manager initialization failed", "ERROR")
            return False
        
        # Log system information
        system_info = get_system_info()
        log(f"System ready - Platform: {system_info.get('platform', 'unknown')}")
        log(f"Free memory: {system_info.get('free_memory', 0)} bytes")
        
        self.state = SystemState.READY
        log("=== System Initialization Complete ===")
        return True
    
    def check_button_press(self, button, last_press_time, button_name):
        """Check for button press with debouncing"""
        current_time = time.ticks_ms()
        
        if button.value() == 0:  # Button pressed (active low)
            if time.ticks_diff(current_time, last_press_time) > BUTTON_DEBOUNCE_MS:
                log(f"{button_name} button pressed")
                return True, current_time
        
        return False, last_press_time
    
    def handle_start_button(self):
        """Handle start button press"""
        pressed, self.last_start_press = self.check_button_press(
            self.start_button, self.last_start_press, "START"
        )
        
        if pressed:
            if self.state == SystemState.READY:
                log("Starting motor test sequence")
                self.test_count += 1
                
                # Choose test mode (could be made configurable)
                test_mode = TEST_MODES["SEQUENCE"]  # Default to sequence mode
                
                if self.test_manager.start_test(test_mode):
                    self.state = SystemState.TESTING
                    log(f"Test #{self.test_count} started - Mode: SEQUENCE")
                else:
                    log("Failed to start test", "ERROR")
                    self.handle_error("Test start failed")
            else:
                log(f"Cannot start test in current state: {self.state}")
    
    def handle_abort_button(self):
        """Handle abort button press"""
        pressed, self.last_abort_press = self.check_button_press(
            self.abort_button, self.last_abort_press, "ABORT"
        )
        
        if pressed:
            log("ABORT button pressed - Emergency stop initiated")
            
            if self.test_manager:
                self.test_manager.emergency_stop()
            
            if self.fcm:
                self.fcm.emergency_stop()
            
            self.state = SystemState.ERROR
            display_error_pattern()
            return True  # Signal to exit main loop
        
        return False
    
    def handle_error(self, error_msg):
        """Handle system errors"""
        self.error_count += 1
        self.last_error_time = time.time()
        
        log(f"System error #{self.error_count}: {error_msg}", "ERROR")
        
        # Emergency stop if testing
        if self.state == SystemState.TESTING and self.test_manager:
            self.test_manager.emergency_stop()
        
        self.state = SystemState.ERROR
        display_error_pattern()
    
    def update_heartbeat(self):
        """Send periodic heartbeat to PX4"""
        current_time = time.time()
        
        if current_time - self.last_heartbeat >= HEARTBEAT_INTERVAL:
            if self.fcm and self.fcm.initialized:
                self.fcm.send_heartbeat()
            self.last_heartbeat = current_time
    
    def update_health_check(self):
        """Perform periodic system health checks"""
        current_time = time.time()
        
        if current_time - self.last_health_check >= HEALTH_CHECK_INTERVAL:
            # Check system resources
            if not check_system_resources():
                self.handle_error("System resource check failed")
                return False
            
            # Check PX4 connection if initialized
            if self.fcm and self.fcm.initialized:
                if not self.fcm.is_connected():
                    log("PX4 connection lost", "WARNING")
                    # Don't treat as critical error in READY state
                    if self.state == SystemState.TESTING:
                        self.handle_error("PX4 connection lost during test")
                        return False
            
            self.last_health_check = current_time
        
        return True
    
    def update_system_status(self):
        """Update and log system status periodically"""
        if self.state == SystemState.TESTING and self.test_manager:
            status = self.test_manager.get_status()
            
            # Log status every 10 seconds during testing
            if int(status['elapsed_time']) % 10 == 0 and int(status['elapsed_time']) > 0:
                elapsed_str = format_time_duration(status['elapsed_time'])
                log(f"Test progress - State: {status['state']}, Time: {elapsed_str}, Tests: {status['tests_completed']}")
    
    def shutdown_system(self):
        """Safely shutdown system"""
        log("Initiating system shutdown")
        self.state = SystemState.SHUTDOWN
        
        try:
            # Stop any active tests
            if self.test_manager:
                self.test_manager.emergency_stop()
            
            # Disarm motors
            if self.fcm and self.fcm.initialized:
                self.fcm.disarm_motors()
                log("Motors disarmed")
            
            # Turn off LED
            set_led(0)
            
            # Final log entry
            uptime = time.time() - self.startup_time
            uptime_str = format_time_duration(uptime)
            log(f"System shutdown complete - Uptime: {uptime_str}, Tests: {self.test_count}, Errors: {self.error_count}")
            
        except Exception as e:
            log(f"Shutdown error: {e}", "ERROR")
    
    def run(self):
        """Main system run loop"""
        try:
            # Display startup pattern
            display_startup_pattern()
            
            # Initialize system
            if not self.initialize_system():
                log("System initialization failed", "ERROR")
                display_error_pattern()
                return
            
            # Show ready status
            log("System ready. Press START button to begin motor test")
            log("Press ABORT button for emergency stop")
            
            # Main control loop
            while True:
                # Feed watchdog
                if self.wdt:
                    self.wdt.feed()
                
                # Update LED patterns
                update_led_pattern()
                
                # Handle abort button (highest priority)
                if self.handle_abort_button():
                    break
                
                # Update communication
                if self.fcm:
                    self.fcm.update()
                
                # Send heartbeat
                self.update_heartbeat()
                

                
                # Perform health checks
                if not self.update_health_check():
                    break
                
                # Handle start button
                self.handle_start_button()
                
                # Update test manager
                if self.state == SystemState.TESTING and self.test_manager:
                    if not self.test_manager.update():
                        # Test completed or failed
                        log("Test sequence completed")
                        display_success_pattern()
                        self.state = SystemState.READY
                        log("System ready for next test")
                
                # Update system status
                self.update_system_status()
                
                # Break on error state
                if self.state == SystemState.ERROR:
                    log("System in error state, stopping", "ERROR")
                    break
                
                # Control loop delay
                time.sleep(MAIN_LOOP_DELAY)
                
                # Periodic garbage collection
                if time.ticks_ms() % 30000 < 100:  # Every 30 seconds
                    gc.collect()
        
        except KeyboardInterrupt:
            log("Keyboard interrupt received", "WARNING")
        except Exception as e:
            log(f"Unhandled exception in main loop: {e}", "ERROR")
            emergency_cleanup()
        finally:
            self.shutdown_system()

def main():
    """Main program entry point"""
    print("\n" + "="*60)
    print("Score99 Team - Enhanced Drone Motor Test Program")
    print("ESP32-S3 MicroPython - PX4 Flight Controller")
    print("Version: 2.0 (Low-Speed Propeller Test)")
    print("Safety Features: Emergency Stop, Connection Monitoring")
    print("="*60 + "\n")
    
    # Brief startup delay
    time.sleep(2)
    
    # Create and run system
    system = DroneTestSystem()
    system.run()

if __name__ == "__main__":
    main()