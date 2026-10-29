"""
Task manager module for handling drone tasks.
Implements task state machine.
Modified to skip task 2 (tunneling) and start from task 3 (obstacle avoidance).
"""
import time
import config as _board_config
import machine
from enum import Enum
from config import (
    PATH_WAYPOINTS, TASK_TRANSITION_INDICES, POSITION_THRESHOLD,
    TAKEOFF_HEIGHT, CRUISING_HEIGHT, OBSTACLE_AVOIDANCE_HEIGHT,
    PICKUP_HEIGHT, DROPOFF_HEIGHT, LANDING_HEIGHT,
    MISSION_COMPLETE_WAIT_TIME, BATTERY_ADC_PIN,
    LOW_BATTERY_THRESHOLD, CRITICAL_BATTERY_THRESHOLD
)
from utils import log, display_led_pattern, format_mission_complete_message, is_position_reached, vector_distance
import math

class TaskState(Enum):
    """Task state enumeration - modified to skip tunneling"""
    INITIALIZING = 0
    WAITING_FOR_START = 1
    # TUNNELLING = 2  # Commented out as we're skipping this task
    OBSTACLE_AVOIDANCE = 3  # This will now be the second task after takeoff
    PACKAGE_SEARCH = 4
    PACKAGE_PICKUP = 5
    MARKER_SEARCH = 6
    PACKAGE_DROPOFF = 7
    LANDING = 8
    MISSION_COMPLETE = 9
    ERROR = 10
    ABORT = 11

class TaskManager:
    """
    Manages drone task execution.
    Implements state machine for task transitions.
    Modified to skip task 2 (tunneling) and transition directly to task 3.
    """
    
    def __init__(self, fcm_interface, perception_system):
        """
        Initialize task manager.
        
        Args:
            fcm_interface: FCMInterface instance for communicating with flight controller
            perception_system: PerceptionSystem instance for sensor data
        """
        self.fcm = fcm_interface
        self.perception = perception_system
        
        self.state = TaskState.INITIALIZING
        # Start from tunneling complete waypoint index (will transition to obstacle avoidance)
        self.current_waypoint_index = TASK_TRANSITION_INDICES["TAKEOFF_COMPLETE"]
        self.mission_start_time = None
        self.mission_complete_time = None
        
        # Task-specific variables
        self.package_picked_up = False
        self.package_dropped_off = False
        self.wall_distance = None
        self.ground_distance = None
        
        # Safety features
        self.LOW_BATTERY_THRESHOLD = LOW_BATTERY_THRESHOLD
        self.CRITICAL_BATTERY_THRESHOLD = CRITICAL_BATTERY_THRESHOLD
        self.battery_adc = machine.ADC(machine.Pin(BATTERY_ADC_PIN))
        self.battery_adc.atten(machine.ADC.ATTN_11DB)  # Full voltage range
        
        # Add abort button
        self.abort_button = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
        
        # Position control variables
        self.path_adjustment = [0, 0, 0]  # Adjustment to preset path based on sensor readings
        self.max_adjustment = 0.5  # Maximum allowed adjustment (meters)
        
        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Manual intervention flags
        self.manual_intervention_required = False
        self.manual_intervention_acknowledged = False
        
        # Initialize HC12 communication interface for Task 6
        from communication import HC12Interface
        self.hc12 = HC12Interface()
    
    def initialize(self):
        """Initialize task manager and prepare for tasks"""
        divider = _board_config.BATTERY_VOLTAGE_DIVIDER
        if divider is None or divider <= 0 or 3.3 * divider <= LOW_BATTERY_THRESHOLD:
            log("Configure BATTERY_VOLTAGE_DIVIDER for the actual ADC circuit", "ERROR")
            return False
        log("Initializing task manager")
        display_led_pattern("STARTUP", 3)
        
        # Ensure FCM interface is initialized
        if not self.fcm.initialized:
            log("FCM interface not initialized", "ERROR")
            self.state = TaskState.ERROR
            return False
            
        # Initialize sensors
        if not self.perception.initialize_sensors():
            log("Sensor initialization failed", "ERROR")
            self.state = TaskState.ERROR
            return False
        
        # Calibrate position sensors
        log("Calibrating sensors...")
        try:
            self.perception.calibrate_sensors()
            log("Sensor calibration complete")
        except Exception as e:
            log(f"Sensor calibration failed: {e}", "ERROR")
            self.state = TaskState.ERROR
            return False
        
        self.state = TaskState.WAITING_FOR_START
        log("Task manager initialization complete, ready to start mission")
        return True
    
    def update(self):
        """
        Update task manager state - modified to skip tunneling.
        Now transitions directly from WAITING_FOR_START to OBSTACLE_AVOIDANCE.
        
        Returns:
            True if task is in progress, False if completed or error
        """
        # Check if manual intervention is required but not yet acknowledged
        if self.manual_intervention_required and not self.manual_intervention_acknowledged:
            display_led_pattern("MANUAL_INTERVENTION")
            
            # Check if intervention button is pressed
            if self.abort_button.value() == 0:  # Button is active low
                time.sleep(0.2)  # Debounce
                if self.abort_button.value() == 0:
                    self.manual_intervention_acknowledged = True
                    log("Manual intervention acknowledged")
                    display_led_pattern("CONTINUE", 2)
            
            return True  # Continue loop until acknowledged
        
        # Get current position from perception system
        try:
            current_position = self.perception.get_current_position()
            self.wall_distance = self.perception.get_wall_distance()
            self.ground_distance = self.perception.get_height()
            
            # Check safe operating height
            if self.state not in [TaskState.INITIALIZING, TaskState.WAITING_FOR_START, 
                                  TaskState.LANDING, TaskState.MISSION_COMPLETE, 
                                  TaskState.ERROR, TaskState.ABORT]:
                if self.ground_distance < 0.5:  # Too close to ground
                    log(f"Warning: Too close to ground: {self.ground_distance:.2f}m", "WARNING")
                    self._apply_safety_correction([0, 0, 0.2])  # Move up 20cm
            
            # Reset consecutive error counter on success
            self.consecutive_errors = 0
            
        except Exception as e:
            log(f"Error getting position data: {e}", "ERROR")
            self.consecutive_errors += 1
            
            if self.consecutive_errors >= self.max_consecutive_errors:
                log("Too many consecutive errors, switching to ERROR state", "ERROR")
                self.state = TaskState.ERROR
                return False
            
            # Try to use last known position
            if not hasattr(self, 'last_known_position'):
                log("No position data available, cannot continue", "ERROR")
                self.state = TaskState.ERROR
                return False
                
            current_position = self.last_known_position
            log("Using last known position as fallback", "WARNING")
        
        # Store current position for future fallback
        self.last_known_position = current_position
        
        # Check if current waypoint is reached
        current_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
        waypoint_reached = is_position_reached(
            current_position, current_waypoint, POSITION_THRESHOLD
        )
        
        # Update task state based on current waypoint and state
        if self.state == TaskState.WAITING_FOR_START:
            # Direct transition to obstacle avoidance (skipping tunneling)
            self.state = TaskState.OBSTACLE_AVOIDANCE
            self.mission_start_time = time.time()
            
            # Initialize motors and set flight mode
            self.fcm.motors_on()
            self.fcm.set_flight_mode("POSITION")
            
            # Move to the first obstacle avoidance waypoint
            # Using the waypoint that was previously the tunneling complete point
            self.current_waypoint_index = TASK_TRANSITION_INDICES["TUNNELLING_COMPLETE"]
            first_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
            self.fcm.set_position(*first_waypoint)
            
            # Display obstacle avoidance pattern since we're skipping tunneling
            display_led_pattern("OBSTACLE_AVOIDANCE", 0.5)
            
            log(f"Starting mission, directly entering obstacle avoidance, setting position {first_waypoint}")
            return True
            
        elif self.state == TaskState.OBSTACLE_AVOIDANCE:
            return self._handle_obstacle_avoidance(current_position, waypoint_reached)
            
        elif self.state == TaskState.PACKAGE_SEARCH:
            return self._handle_package_search(current_position, waypoint_reached)
            
        elif self.state == TaskState.PACKAGE_PICKUP:
            return self._handle_package_pickup(current_position, waypoint_reached)
            
        elif self.state == TaskState.MARKER_SEARCH:
            return self._handle_marker_search(current_position, waypoint_reached)
            
        elif self.state == TaskState.PACKAGE_DROPOFF:
            return self._handle_package_dropoff(current_position, waypoint_reached)
            
        elif self.state == TaskState.LANDING:
            return self._handle_landing(current_position, waypoint_reached)
            
        elif self.state == TaskState.MISSION_COMPLETE:
            return self._handle_mission_complete()
            
        elif self.state == TaskState.ERROR:
            log("System in error state", "ERROR")
            self.fcm.emergency_stop()
            display_led_pattern("ERROR")
            return False
            
        elif self.state == TaskState.ABORT:
            log("System in abort state, performing emergency landing", "WARNING")
            return self._handle_abort()
        
        return True
    
    def get_battery_voltage(self):
        """Get current battery voltage"""
        # Read ADC value and convert to voltage
        raw_value = self.battery_adc.read()
        voltage = raw_value * 3.3 / 4095 * _board_config.BATTERY_VOLTAGE_DIVIDER  # Assuming 3:1 voltage divider ratio
        return voltage
    
    def abort_mission(self):
        """Abort mission and perform emergency landing"""
        log("Mission abort requested", "WARNING")
        self.state = TaskState.ABORT
    
    def emergency_landing(self):
        """Immediately perform emergency landing"""
        log("Emergency landing initiated", "WARNING")
        
        # Get current position
        try:
            current_position = self.perception.get_current_position()
            # Set target to current x,y but landing height
            landing_target = (current_position[0], current_position[1], LANDING_HEIGHT)
            
            # Switch to landing mode
            self.fcm.set_flight_mode("LAND")
            
            # Send emergency landing command
            self.fcm.set_position(*landing_target)
            log(f"Emergency landing to position {landing_target}")
            
            # Wait for landing
            start_time = time.time()
            while time.time() - start_time < 10:  # Wait max 10 seconds
                try:
                    height = self.perception.get_height()
                    if height < 0.3:  # Close enough to ground
                        break
                    time.sleep(0.5)
                except:
                    pass
                
            # Stop motors
            self.fcm.motors_off()
            log("Emergency landing complete")
            
        except Exception as e:
            log(f"Error during emergency landing: {e}", "ERROR")
            # Last resort - stop motors
            self.fcm.emergency_stop()
    
    def _get_adjusted_waypoint(self, index):
        """
        Get dynamically adjusted waypoint based on sensor data.
        
        Args:
            index: Waypoint index in PATH_WAYPOINTS
        
        Returns:
            Adjusted waypoint coordinates (x, y, z)
        """
        if index >= len(PATH_WAYPOINTS):
            return PATH_WAYPOINTS[-1]  # Return last waypoint if index out of range
            
        waypoint = PATH_WAYPOINTS[index]
        
        # Apply position adjustment within limits
        adjusted_waypoint = [
            waypoint[0] + self.path_adjustment[0],
            waypoint[1] + self.path_adjustment[1],
            waypoint[2] + self.path_adjustment[2]
        ]
        
        return adjusted_waypoint
    
    def _apply_safety_correction(self, correction):
        """Apply safety correction to path adjustment"""
        for i in range(3):
            self.path_adjustment[i] += correction[i]
            # Limit adjustment to max_adjustment
            self.path_adjustment[i] = max(min(self.path_adjustment[i], 
                                            self.max_adjustment), 
                                        -self.max_adjustment)
    
    def _reset_path_adjustment(self):
        """Reset path adjustment to zero"""
        self.path_adjustment = [0, 0, 0]
    
    def _move_to_next_waypoint(self):
        """Advance to next waypoint and update FCM"""
        if self.current_waypoint_index < len(PATH_WAYPOINTS) - 1:
            self.current_waypoint_index += 1
            next_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
            self.fcm.set_position(*next_waypoint)
            log(f"Moving to waypoint {self.current_waypoint_index}: {next_waypoint}")
            return True
        else:
            log("Reached final waypoint", "WARNING")
            return False
    
    def _handle_obstacle_avoidance(self, current_position, waypoint_reached):
        """Handle obstacle avoidance task state"""
        display_led_pattern("OBSTACLE_AVOIDANCE", 0.5)
        
        # Use position mode for precise path following
        if self.fcm.flight_mode != "POSITION":
            self.fcm.set_flight_mode("POSITION")
        
        # Check height to ensure we're below beams but above minimum safe height
        if current_position[2] > OBSTACLE_AVOIDANCE_HEIGHT + 0.1:
            log(f"Obstacle avoidance height too high, adjusting downward", "WARNING")
            self._apply_safety_correction([0, 0, -0.1])  # Adjust slightly downward
        
        if waypoint_reached:
            log(f"Reached obstacle avoidance waypoint {self.current_waypoint_index}")
            
            # Check if all obstacle avoidance waypoints are completed
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["OBSTACLE_AVOIDANCE_COMPLETE"]:
                log("Obstacle avoidance task complete, transitioning to package search task")
                self.state = TaskState.PACKAGE_SEARCH
                self._reset_path_adjustment()  # Reset adjustments for next task
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_search(self, current_position, waypoint_reached):
        """Handle package search task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package search area, transitioning to package pickup task")
            
            # In a real implementation, we would use camera to locate package
            # For simplified version, just move to next waypoint
            
            self.state = TaskState.PACKAGE_PICKUP
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_pickup(self, current_position, waypoint_reached):
        """Handle package pickup task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package pickup position")
            
            if not self.package_picked_up:
                # Switch to altitude hold mode for stable pickup
                self.fcm.set_flight_mode("ALTITUDE")
                
                # Use RC control for fine control
                channels = [1500, 1500, 1450, 1500, 1500, 1100, 1100, 1100]  # Slightly lower throttle for stable hover
                self.fcm.rc_override(channels)
                time.sleep(1)  # Wait for stability
                
                # Simulate package pickup - would activate gripper mechanism in real implementation
                log("Activating gripper mechanism")
                
                # Add gripper control code here
                # self.gripper.activate()
                
                log("Package successfully picked up")
                self.package_picked_up = True
                time.sleep(1)  # Wait for stable pickup
                
                # Switch back to position mode for continued flight
                self.fcm.set_flight_mode("POSITION")
                
                # Check if operator confirmation needed
                if current_position[2] < 0.5:  # If we're close to ground
                    log("Manual confirmation of successful pickup required", "WARNING")
                    self.manual_intervention_required = True
                    self.manual_intervention_acknowledged = False
                    return True
                
            # Move to next waypoint (ascend after pickup)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["PICKUP_COMPLETE"]:
                    log("Package pickup task complete, transitioning to marker search task")
                    self.state = TaskState.MARKER_SEARCH
                    self._reset_path_adjustment()  # Reset adjustments for next task
            
        return True
    
    def _handle_marker_search(self, current_position, waypoint_reached):
        """Handle marker search task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached marker search area, transitioning to package drop-off task")
            
            # In a real implementation, we would use camera to locate marker
            # For simplified version, just move to next waypoint
            
            self.state = TaskState.PACKAGE_DROPOFF
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_dropoff(self, current_position, waypoint_reached):
        """Handle package drop-off task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached package drop-off position")
            
            if not self.package_dropped_off and self.package_picked_up:
                # Switch to altitude hold mode for stable drop-off
                self.fcm.set_flight_mode("ALTITUDE")
                
                # Use RC control for fine control
                channels = [1500, 1500, 1450, 1500, 1500, 1100, 1100, 1100]  # Slightly lower throttle for stable hover
                self.fcm.rc_override(channels)
                time.sleep(1)  # Wait for stability
                
                # Simulate package drop-off - would release gripper mechanism in real implementation
                log("Releasing gripper mechanism")
                
                # Add gripper release code here
                # self.gripper.release()
                
                log("Package successfully dropped off")
                self.package_dropped_off = True
                time.sleep(1)  # Wait for stable drop-off
                
                # Switch back to position mode for continued flight
                self.fcm.set_flight_mode("POSITION")
                
                # Check if operator confirmation needed
                if current_position[2] < 0.5:  # If we're close to ground
                    log("Manual confirmation of successful drop-off required", "WARNING")
                    self.manual_intervention_required = True
                    self.manual_intervention_acknowledged = False
                    return True
                
            # Move to next waypoint (ascend after drop-off)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["DROPOFF_COMPLETE"]:
                    log("Package drop-off task complete, transitioning to landing task")
                    self.state = TaskState.LANDING
                    self._reset_path_adjustment()  # Reset adjustments for landing
            
        return True
    
    def _handle_landing(self, current_position, waypoint_reached):
        """Handle landing task state"""
        display_led_pattern("LANDING", 0.5)
        
        # Switch to landing mode
        if self.fcm.flight_mode != "LAND":
            self.fcm.set_flight_mode("LAND")
        
        # Monitor landing progress
        height = self.perception.get_height()
        if height is not None and height < 0.1:
            log(f"Landing complete, current height: {height:.2f}m")
            self.fcm.motors_off()
            self.mission_complete_time = time.time()
            self.state = TaskState.MISSION_COMPLETE
        
        # If we've reached landing waypoint but not low enough,
        # make small downward adjustments
        if waypoint_reached and height > 0.1:
            log(f"At landing waypoint but still at height {height:.2f}m, continuing descent")
            # Use RC control for fine landing
            channels = self.fcm.rc_channels.copy()
            channels[2] = 1350  # Reduce throttle for slow descent
            self.fcm.rc_override(channels)
        
        return True
    
    def _handle_mission_complete(self):
        """Handle mission complete state"""
        display_led_pattern("MISSION_COMPLETE")
        
        # Check if we need to wait before sending mission complete
        if not self.mission_complete_time:
            self.mission_complete_time = time.time()
            log("Mission complete, waiting before transmission")
            return True
        
        elapsed_time = time.time() - self.mission_complete_time
        if elapsed_time < MISSION_COMPLETE_WAIT_TIME:
            # Still waiting
            return True
        
        # Time's up, send mission complete message
        message = format_mission_complete_message()
        log(f"Mission complete message: {message}")
        
        # Send message via HC12
        if self.hc12.initialized:
            if self.hc12.send_message(message):
                log("Mission complete message transmitted successfully via HC12")
            else:
                log("Failed to transmit mission complete message via HC12", "ERROR")
        else:
            log("HC12 interface not initialized, cannot transmit message", "ERROR")
        
        log("Mission complete")
        return False  # Mission complete, exit main loop
    
    def _handle_abort(self):
        """Handle abort task state"""
        display_led_pattern("ERROR", 0.2)
        
        # Perform emergency landing
        self.emergency_landing()
        
        log("Mission aborted")
        return False  # Exit main loop