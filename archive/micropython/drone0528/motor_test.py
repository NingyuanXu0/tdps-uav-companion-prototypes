"""
Enhanced Motor Test Manager - Safe Low-Speed Propeller Testing
Handles comprehensive motor testing sequence with safety features
"""
import time
from config import (
    MOTOR_TEST_DURATION, MOTOR_TEST_ALL_DURATION, MOTOR_TEST_SEQUENCE,
    ALL_MOTORS_TEST_SEQUENCE, MOTOR_TEST_SEQUENCE_DELAY, ARM_SAFETY_DELAY,
    MAX_TEST_DURATION, CONNECTION_TIMEOUT, MOTOR_MAPPING, DEFAULT_FRAME_TYPE,
    DEFAULT_TEST_MODE, TEST_MODES
)
from utils import log, set_led, blink_led_pattern

class MotorTestState:
    """Enhanced motor test states"""
    IDLE = 0
    CONNECTING = 1
    CONNECTION_CHECK = 2
    ARMING = 3
    ARMED_WAIT = 4
    TEST_MOTOR_1 = 10
    TEST_MOTOR_2 = 11
    TEST_MOTOR_3 = 12
    TEST_MOTOR_4 = 13
    TEST_ALL_MOTORS = 20
    SEQUENCE_TEST = 30
    MANUAL_TEST = 40
    DISARMING = 90
    COMPLETE = 91
    ERROR = 99
    EMERGENCY_STOP = 100

class MotorTestManager:
    """Enhanced motor testing sequence manager"""
    
    def __init__(self, fcm_interface):
        self.fcm = fcm_interface
        self.state = MotorTestState.IDLE
        self.state_start_time = None
        self.test_start_time = None
        self.current_motor = 1
        self.test_mode = DEFAULT_TEST_MODE
        self.sequence_index = 0
        self.motor_mapping = MOTOR_MAPPING[DEFAULT_FRAME_TYPE]
        
        # Safety tracking
        self.connection_established = False
        self.last_safety_check = 0
        self.error_count = 0
        self.max_errors = 3
        
        # Test statistics
        self.tests_completed = 0
        self.total_test_time = 0
        
        log("Motor Test Manager initialized")
        
    def set_test_mode(self, mode):
        """Set test mode"""
        if mode in TEST_MODES.values():
            self.test_mode = mode
            log(f"Test mode set to: {self._get_mode_name(mode)}")
        else:
            log(f"Invalid test mode: {mode}", "ERROR")
    
    def _get_mode_name(self, mode):
        """Get human-readable mode name"""
        for name, value in TEST_MODES.items():
            if value == mode:
                return name
        return "UNKNOWN"
        
    def start_test(self, test_mode=None):
        """Start motor test sequence"""
        if self.state != MotorTestState.IDLE:
            log(f"Test already in progress (state: {self._get_state_name(self.state)})", "WARNING")
            return False
        
        if test_mode:
            self.set_test_mode(test_mode)
        
        log(f"Starting motor test sequence - Mode: {self._get_mode_name(self.test_mode)}")
        self.test_start_time = time.time()
        self.tests_completed = 0
        self.error_count = 0
        
        # Start with connection check
        self._change_state(MotorTestState.CONNECTING)
        return True
    
    def stop_test(self):
        """Stop motor test and disarm safely"""
        log("Stopping motor test sequence")
        self._change_state(MotorTestState.DISARMING)
    
    def emergency_stop(self):
        """Emergency stop all motors immediately"""
        log("EMERGENCY STOP ACTIVATED", "WARNING")
        self.fcm.emergency_stop()
        self._change_state(MotorTestState.EMERGENCY_STOP)
    
    def _change_state(self, new_state):
        """Change test state with logging"""
        old_state = self.state
        self.state = new_state
        self.state_start_time = time.time()
        
        log(f"State: {self._get_state_name(old_state)} → {self._get_state_name(new_state)}")
        
        # Update LED pattern based on state
        self._update_led_pattern()
    
    def _get_state_name(self, state):
        """Get human-readable state name"""
        state_names = {
            MotorTestState.IDLE: "IDLE",
            MotorTestState.CONNECTING: "CONNECTING",
            MotorTestState.CONNECTION_CHECK: "CONNECTION_CHECK",
            MotorTestState.ARMING: "ARMING",
            MotorTestState.ARMED_WAIT: "ARMED_WAIT",
            MotorTestState.TEST_MOTOR_1: "TEST_MOTOR_1",
            MotorTestState.TEST_MOTOR_2: "TEST_MOTOR_2",
            MotorTestState.TEST_MOTOR_3: "TEST_MOTOR_3",
            MotorTestState.TEST_MOTOR_4: "TEST_MOTOR_4",
            MotorTestState.TEST_ALL_MOTORS: "TEST_ALL_MOTORS",
            MotorTestState.SEQUENCE_TEST: "SEQUENCE_TEST",
            MotorTestState.MANUAL_TEST: "MANUAL_TEST",
            MotorTestState.DISARMING: "DISARMING",
            MotorTestState.COMPLETE: "COMPLETE",
            MotorTestState.ERROR: "ERROR",
            MotorTestState.EMERGENCY_STOP: "EMERGENCY_STOP"
        }
        return state_names.get(state, f"UNKNOWN_{state}")
    
    def _update_led_pattern(self):
        """Update LED pattern based on current state"""
        if self.state == MotorTestState.CONNECTING:
            blink_led_pattern("CONNECTING")
        elif self.state in [MotorTestState.TEST_MOTOR_1, MotorTestState.TEST_MOTOR_2,
                           MotorTestState.TEST_MOTOR_3, MotorTestState.TEST_MOTOR_4,
                           MotorTestState.TEST_ALL_MOTORS, MotorTestState.SEQUENCE_TEST]:
            blink_led_pattern("MOTOR_TEST")
        elif self.state == MotorTestState.ARMED_WAIT:
            blink_led_pattern("ARMED")
        elif self.state == MotorTestState.ERROR:
            blink_led_pattern("ERROR")
        elif self.state == MotorTestState.EMERGENCY_STOP:
            blink_led_pattern("EMERGENCY")
    
    def _safety_check(self):
        """Perform safety checks"""
        current_time = time.time()
        
        # Check connection status
        if self.connection_established and not self.fcm.is_connected():
            log("Connection to PX4 lost", "WARNING")
            return False
        
        # Check for maximum test duration
        if self.test_start_time and (current_time - self.test_start_time) > MAX_TEST_DURATION:
            log("Maximum test duration exceeded", "WARNING")
            return False
        
        # Check error count
        if self.error_count >= self.max_errors:
            log(f"Too many errors: {self.error_count}", "ERROR")
            return False
        
        return True
    
    def _handle_error(self, error_msg):
        """Handle errors with counting and logging"""
        self.error_count += 1
        log(f"Error {self.error_count}/{self.max_errors}: {error_msg}", "ERROR")
        
        if self.error_count >= self.max_errors:
            log("Maximum errors reached, stopping test", "ERROR")
            self._change_state(MotorTestState.ERROR)
    
    def update(self):
        """Update test state machine"""
        if self.state == MotorTestState.IDLE:
            return True
        
        elapsed = time.time() - self.state_start_time
        
        # Perform periodic safety checks
        if time.time() - self.last_safety_check > 1.0:
            if not self._safety_check():
                self._change_state(MotorTestState.ERROR)
                return False
            self.last_safety_check = time.time()
        
        # State machine logic
        if self.state == MotorTestState.CONNECTING:
            # Send heartbeats and wait for connection
            self.fcm.send_heartbeat()
            if elapsed > 2.0:  # Wait at least 2 seconds
                if self.fcm.is_connected():
                    log("Connection established with PX4")
                    self.connection_established = True
                    self._change_state(MotorTestState.CONNECTION_CHECK)
                elif elapsed > CONNECTION_TIMEOUT:
                    self._handle_error("Connection timeout")
                    return False
                    
        elif self.state == MotorTestState.CONNECTION_CHECK:
            # Verify stable connection
            if elapsed > 2.0:
                if self.fcm.is_connected():
                    log("Connection verified, proceeding to ARM")
                    self._change_state(MotorTestState.ARMING)
                else:
                    self._handle_error("Connection verification failed")
                    
        elif self.state == MotorTestState.ARMING:
            if self.fcm.disarm_motors():
                log("Disarm requested before the motor test")
                self._change_state(MotorTestState.ARMED_WAIT)
            else:
                self._handle_error("Failed to prepare disarmed motor test")
                
        elif self.state == MotorTestState.ARMED_WAIT:
            if elapsed >= ARM_SAFETY_DELAY and not self.fcm.armed:
                log("Safety delay complete, starting motor tests")
                self._start_test_sequence()
                
        elif self.state in [MotorTestState.TEST_MOTOR_1, MotorTestState.TEST_MOTOR_2,
                           MotorTestState.TEST_MOTOR_3, MotorTestState.TEST_MOTOR_4]:
            self._handle_individual_motor_test(elapsed)
            
        elif self.state == MotorTestState.TEST_ALL_MOTORS:
            self._handle_all_motors_test(elapsed)
            
        elif self.state == MotorTestState.SEQUENCE_TEST:
            self._handle_sequence_test(elapsed)
            
        elif self.state == MotorTestState.DISARMING:
            if self.fcm.disarm_motors():
                log("Motors disarmed successfully")
                self._change_state(MotorTestState.COMPLETE)
            else:
                self._handle_error("Failed to disarm motors")
                
        elif self.state == MotorTestState.COMPLETE:
            total_time = time.time() - self.test_start_time
            log(f"Motor test completed successfully in {total_time:.1f} seconds")
            log(f"Tests completed: {self.tests_completed}")
            set_led(1)  # Solid LED indicates completion
            return False
            
        elif self.state in [MotorTestState.ERROR, MotorTestState.EMERGENCY_STOP]:
            set_led(0)
            return False
        
        return True
    
    def _start_test_sequence(self):
        """Start the appropriate test sequence based on mode"""
        if self.test_mode == TEST_MODES["INDIVIDUAL"]:
            self._change_state(MotorTestState.TEST_MOTOR_1)
        elif self.test_mode == TEST_MODES["ALL_TOGETHER"]:
            self._change_state(MotorTestState.TEST_ALL_MOTORS)
        elif self.test_mode == TEST_MODES["SEQUENCE"]:
            self.sequence_index = 0
            self._change_state(MotorTestState.SEQUENCE_TEST)
        else:
            log(f"Unsupported test mode: {self.test_mode}", "ERROR")
            self._change_state(MotorTestState.ERROR)
    
    def _handle_individual_motor_test(self, elapsed):
        """Handle individual motor testing"""
        motor_num = self.state - MotorTestState.TEST_MOTOR_1 + 1
        motor_name = self.motor_mapping.get(motor_num, f"Motor {motor_num}")
        
        if elapsed < MOTOR_TEST_DURATION:
            # Test motor at configured throttle
            if elapsed < 0.5:  # First half second - very low
                throttle = 5
            else:  # Remaining time - normal test throttle
                throttle = 12
                
            if int(elapsed * 2) % 2 == 0:  # Update every 0.5 seconds
                log(f"Testing {motor_name} at {throttle}%")
                self.fcm.test_motor_individual(motor_num, throttle, 2)
        else:
            # Stop current motor and move to next
            self.fcm.stop_motor_test()
            self.tests_completed += 1
            
            if motor_num < 4:
                # Next motor
                time.sleep(MOTOR_TEST_SEQUENCE_DELAY)
                self._change_state(self.state + 1)
            else:
                # All individual motors tested, proceed to all motors test
                if self.test_mode == TEST_MODES["INDIVIDUAL"]:
                    self._change_state(MotorTestState.TEST_ALL_MOTORS)
                else:
                    self._change_state(MotorTestState.DISARMING)
    
    def _handle_all_motors_test(self, elapsed):
        """Handle all motors testing"""
        if elapsed < MOTOR_TEST_ALL_DURATION:
            # Test all motors at low throttle
            if elapsed < 1.0:
                throttle = 5  # Start very low
            elif elapsed < 3.0:
                throttle = 10  # Medium low
            else:
                throttle = 8   # Reduce slightly at end
                
            if int(elapsed * 2) % 2 == 0:  # Update every 0.5 seconds
                log(f"Testing all motors at {throttle}%")
                self.fcm.test_all_motors(throttle, 2)
        else:
            # Stop all motors and complete
            self.fcm.stop_motor_test()
            self.tests_completed += 1
            self._change_state(MotorTestState.DISARMING)
    
    def _handle_sequence_test(self, elapsed):
        """Handle predefined sequence testing"""
        if self.sequence_index >= len(MOTOR_TEST_SEQUENCE):
            # Sequence complete
            self._change_state(MotorTestState.DISARMING)
            return
        
        throttle, duration, description = MOTOR_TEST_SEQUENCE[self.sequence_index]
        
        if elapsed < duration:
            if int(elapsed * 4) % 4 == 0:  # Update every 0.25 seconds
                log(f"Sequence {self.sequence_index + 1}: {description} - {throttle}%")
                self.fcm.test_all_motors(throttle, 1)
        else:
            # Move to next in sequence
            self.fcm.stop_motor_test()
            self.tests_completed += 1
            self.sequence_index += 1
            
            if self.sequence_index < len(MOTOR_TEST_SEQUENCE):
                time.sleep(MOTOR_TEST_SEQUENCE_DELAY)
                self._change_state(MotorTestState.SEQUENCE_TEST)  # Reset timer
            else:
                self._change_state(MotorTestState.DISARMING)
    
    def get_status(self):
        """Get current test status"""
        return {
            'state': self._get_state_name(self.state),
            'elapsed_time': time.time() - self.test_start_time if self.test_start_time else 0,
            'tests_completed': self.tests_completed,
            'error_count': self.error_count,
            'connected': self.connection_established
        }