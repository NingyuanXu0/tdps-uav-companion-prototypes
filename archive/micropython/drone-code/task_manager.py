"""
Task manager module for handling drone mission tasks.
Implements a state machine for the six main tasks.
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
    """Enumeration of task states"""
    INITIALIZING = 0
    WAITING_FOR_START = 1
    TAKEOFF = 2
    TUNNELLING = 3
    OBSTACLE_AVOIDANCE = 4
    PACKAGE_SEARCH = 5
    PACKAGE_PICKUP = 6
    MARKER_SEARCH = 7
    PACKAGE_DROPOFF = 8
    LANDING = 9
    MISSION_COMPLETE = 10
    ERROR = 11
    ABORT = 12

class TaskManager:
    """
    Manages the execution of drone mission tasks.
    Implements a state machine for task transitions.
    """
    
    def __init__(self, fcm_interface, perception_system, wireless_comm):
        """
        Initialize task manager.
        
        Args:
            fcm_interface: FCMInterface instance for communicating with flight controller
            perception_system: PerceptionSystem instance for sensor data
            wireless_comm: WirelessCommunication instance for mission status transmission
        """
        self.fcm = fcm_interface
        self.perception = perception_system
        self.wireless = wireless_comm
        
        self.state = TaskState.INITIALIZING
        self.current_waypoint_index = 0
        self.mission_start_time = None
        self._takeoff_started = False
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
        self.max_adjustment = 0.5  # Maximum adjustment allowed in meters
        
        # Error tracking
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # Manual intervention flag
        self.manual_intervention_required = False
        self.manual_intervention_acknowledged = False
    
    def initialize(self):
        """Initialize task manager and prepare for mission"""
        divider = _board_config.BATTERY_VOLTAGE_DIVIDER
        if divider is None or divider <= 0 or 3.3 * divider <= LOW_BATTERY_THRESHOLD:
            log("Configure BATTERY_VOLTAGE_DIVIDER for the actual ADC circuit", "ERROR")
            return False
        log("Initializing task manager")
        display_led_pattern("STARTUP", 3)
        
        # Make sure wireless module is initialized
        self.wireless.initialize()
        
        # Initialize sensors
        if not self.perception.initialize_sensors():
            log("Failed to initialize sensors", "ERROR")
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
        log("Task manager initialized, ready for mission start")
        return True
    
    def update(self):
        """
        Update task manager state.
        This is the main state machine function called in the main loop.
        
        Returns:
            True if mission is ongoing, False if mission is complete or error occurred
        """
        # Check if manual intervention is required but not acknowledged
        if self.manual_intervention_required and not self.manual_intervention_acknowledged:
            display_led_pattern("MANUAL_INTERVENTION")
            
            # Check if intervention button is pressed
            if self.abort_button.value() == 0:  # Button is pressed (active low)
                time.sleep(0.2)  # Debounce
                if self.abort_button.value() == 0:
                    self.manual_intervention_acknowledged = True
                    log("Manual intervention acknowledged")
                    display_led_pattern("CONTINUE", 2)
            
            return True  # Keep looping until acknowledged
        
        # Get current position from perception system
        try:
            current_position = self.perception.get_current_position()
            self.wall_distance = self.perception.get_wall_distance()
            self.ground_distance = self.perception.get_height()
            
            # Check for safe operating height
            if self.state not in [TaskState.INITIALIZING, TaskState.WAITING_FOR_START, 
                                  TaskState.TAKEOFF, TaskState.LANDING, 
                                  TaskState.MISSION_COMPLETE, TaskState.ERROR, TaskState.ABORT]:
                if self.ground_distance < 0.5:  # Too close to ground
                    log(f"Warning: Too close to ground: {self.ground_distance:.2f}m", "WARNING")
                    self._apply_safety_correction([0, 0, 0.2])  # Move up 20cm
            
            # Reset consecutive errors counter when successful
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
        
        # Check if we've reached the current waypoint
        current_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
        waypoint_reached = is_position_reached(
            current_position, current_waypoint, POSITION_THRESHOLD
        )
        
        # Update task state based on current waypoint and state
        if self.state == TaskState.WAITING_FOR_START:
            # This state is now handled in main.py
            self.state = TaskState.TAKEOFF
            self.mission_start_time = time.time()
            return True
            
        elif self.state == TaskState.TAKEOFF:
            return self._handle_takeoff(current_position, waypoint_reached)
            
        elif self.state == TaskState.TUNNELLING:
            return self._handle_tunnelling(current_position, waypoint_reached)
            
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
            log("System in ERROR state", "ERROR")
            self.fcm.emergency_stop()
            display_led_pattern("ERROR")
            return False
            
        elif self.state == TaskState.ABORT:
            log("System in ABORT state, executing emergency landing", "WARNING")
            return self._handle_abort()
        
        return True
    
    def get_battery_voltage(self):
        """Get current battery voltage"""
        # Read ADC value and convert to voltage
        raw_value = self.battery_adc.read()
        voltage = raw_value * 3.3 / 4095 * _board_config.BATTERY_VOLTAGE_DIVIDER  # Assuming voltage divider with 3:1 ratio
        return voltage
    
    def abort_mission(self):
        """Abort the mission and perform emergency landing"""
        log("Mission abort requested", "WARNING")
        self.state = TaskState.ABORT
    
    def emergency_landing(self):
        """Perform emergency landing immediately"""
        log("Emergency landing initiated", "WARNING")
        
        # Get current position
        try:
            current_position = self.perception.get_current_position()
            # Set target to current x,y but at landing height
            landing_target = (current_position[0], current_position[1], LANDING_HEIGHT)
            
            # Send emergency landing command
            self.fcm.set_position(*landing_target)
            log(f"Emergency landing at position {landing_target}")
            
            # Wait for landing
            start_time = time.time()
            while time.time() - start_time < 10:  # Maximum 10s wait
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
        Get waypoint with dynamic adjustment based on sensor data.
        
        Args:
            index: Index of waypoint in PATH_WAYPOINTS
        
        Returns:
            Adjusted waypoint coordinates (x, y, z)
        """
        if index >= len(PATH_WAYPOINTS):
            return PATH_WAYPOINTS[-1]  # Return last waypoint if index is out of range
            
        waypoint = PATH_WAYPOINTS[index]
        
        # Apply position adjustment within limits
        adjusted_waypoint = [
            waypoint[0] + self.path_adjustment[0],
            waypoint[1] + self.path_adjustment[1],
            waypoint[2] + self.path_adjustment[2]
        ]
        
        return adjusted_waypoint
    
    def _apply_safety_correction(self, correction):
        """Apply a safety correction to the path adjustment"""
        for i in range(3):
            self.path_adjustment[i] += correction[i]
            # Limit adjustment to max_adjustment
            self.path_adjustment[i] = max(min(self.path_adjustment[i], 
                                            self.max_adjustment), 
                                        -self.max_adjustment)
    
    def _reset_path_adjustment(self):
        """Reset the path adjustment to zero"""
        self.path_adjustment = [0, 0, 0]
    
    def _move_to_next_waypoint(self):
        """Advance to the next waypoint and update FCM"""
        if self.current_waypoint_index < len(PATH_WAYPOINTS) - 1:
            self.current_waypoint_index += 1
            next_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
            self.fcm.set_position(*next_waypoint)
            log(f"Moving to waypoint {self.current_waypoint_index}: {next_waypoint}")
            return True
        else:
            log("Reached final waypoint", "WARNING")
            return False
    
    def _handle_takeoff(self, current_position, waypoint_reached):
        """Handle takeoff task state"""
        display_led_pattern("TAKEOFF", 0.5)
        
        if not self._takeoff_started:
            self._takeoff_started = True
            if self.mission_start_time is None:
                self.mission_start_time = time.time()
            log("Starting takeoff")
            
            # Initialize motors and start takeoff
            self.fcm.motors_on()
            first_waypoint = self._get_adjusted_waypoint(0)
            self.fcm.set_position(*first_waypoint)
        
        # Monitor takeoff progress
        height = self.perception.get_height()
        if height < 0.5 and time.time() - self.mission_start_time > 5:
            log("Takeoff seems stuck, may need intervention", "WARNING")
            self.manual_intervention_required = True
            return True
        
        if waypoint_reached:
            log(f"Takeoff complete, reached height {current_position[2]:.2f}m")
            self.state = TaskState.TUNNELLING
            self._move_to_next_waypoint()
            self._reset_path_adjustment()  # Reset any adjustments after successful takeoff
        
        return True
    
    def _handle_tunnelling(self, current_position, waypoint_reached):
        """Handle tunnelling task state"""
        display_led_pattern("TUNNELLING", 0.5)
        
        # Check for proximity to tunnels using front-facing sensor
        if self.wall_distance is not None and self.wall_distance < 0.5:
            log(f"Wall detected at {self.wall_distance:.2f}m, adjusting position", "WARNING")
            # Adjust path to avoid collision
            if self.wall_distance < 0.3:  # Too close
                self._apply_safety_correction([0, -0.2, 0])  # Move back from wall
        
        if waypoint_reached:
            log(f"Reached tunnelling waypoint {self.current_waypoint_index}")
            
            # Check if we've completed all tunnelling waypoints
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["TUNNELLING_COMPLETE"]:
                log("Tunnelling complete, transitioning to obstacle avoidance")
                self.state = TaskState.OBSTACLE_AVOIDANCE
                self._reset_path_adjustment()  # Reset adjustments for next task
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_obstacle_avoidance(self, current_position, waypoint_reached):
        """Handle obstacle avoidance task state"""
        display_led_pattern("OBSTACLE_AVOIDANCE", 0.5)
        
        # Check height to make sure we're under the beams but above safety height
        if current_position[2] > OBSTACLE_AVOIDANCE_HEIGHT + 0.1:
            log(f"Too high for obstacle avoidance, adjusting down", "WARNING")
            self._apply_safety_correction([0, 0, -0.1])  # Adjust down slightly
        
        if waypoint_reached:
            log(f"Reached obstacle avoidance waypoint {self.current_waypoint_index}")
            
            # Check if we've completed all obstacle avoidance waypoints
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["OBSTACLE_AVOIDANCE_COMPLETE"]:
                log("Obstacle avoidance complete, transitioning to package search")
                self.state = TaskState.PACKAGE_SEARCH
                self._reset_path_adjustment()  # Reset adjustments for next task
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_search(self, current_position, waypoint_reached):
        """Handle package search task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package search area, transitioning to pickup")
            
            # In a real implementation, we would use the camera to locate the package
            # For the simplified version, we'll just move to the next waypoint
            
            self.state = TaskState.PACKAGE_PICKUP
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_pickup(self, current_position, waypoint_reached):
        """Handle package pickup task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package pickup position")
            
            if not self.package_picked_up:
                # Simulate package pickup - in real implementation, would activate gripper
                log("Activating gripper mechanism")
                
                # This would be a call to the gripper mechanism
                # self.gripper.activate()
                
                log("Package picked up successfully")
                self.package_picked_up = True
                time.sleep(1)  # Wait for stable pickup
                
                # Wait for operator confirmation if needed
                if current_position[2] < 0.5:  # If we're close to the ground
                    log("Manual check required for successful pickup", "WARNING")
                    self.manual_intervention_required = True
                    self.manual_intervention_acknowledged = False
                    return True
                
            # Move to next waypoint (ascend after pickup)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["PICKUP_COMPLETE"]:
                    log("Package pickup complete, transitioning to marker search")
                    self.state = TaskState.MARKER_SEARCH
                    self._reset_path_adjustment()  # Reset adjustments for next task
            
        return True
    
    def _handle_marker_search(self, current_position, waypoint_reached):
        """Handle marker search task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached marker search area, transitioning to dropoff")
            
            # In a real implementation, we would use the camera to locate the marker
            # For the simplified version, we'll just move to the next waypoint
            
            self.state = TaskState.PACKAGE_DROPOFF
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_dropoff(self, current_position, waypoint_reached):
        """Handle package dropoff task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached package dropoff position")
            
            if not self.package_dropped_off and self.package_picked_up:
                # Simulate package dropoff - in real implementation, would release gripper
                log("Releasing gripper mechanism")
                
                # This would be a call to the gripper mechanism
                # self.gripper.release()
                
                log("Package dropped off successfully")
                self.package_dropped_off = True
                time.sleep(1)  # Wait for stable dropoff
                
                # Wait for operator confirmation if needed
                if current_position[2] < 0.5:  # If we're close to the ground
                    log("Manual check required for successful dropoff", "WARNING")
                    self.manual_intervention_required = True
                    self.manual_intervention_acknowledged = False
                    return True
                
            # Move to next waypoint (ascend after dropoff)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["DROPOFF_COMPLETE"]:
                    log("Package dropoff complete, transitioning to landing")
                    self.state = TaskState.LANDING
                    self._reset_path_adjustment()  # Reset adjustments for landing
            
        return True
    
    def _handle_landing(self, current_position, waypoint_reached):
        """Handle landing task state"""
        display_led_pattern("LANDING", 0.5)
        
        # Monitor landing progress
        height = self.perception.get_height()
        if height < 0.1:
            log(f"Landing complete, current height: {height:.2f}m")
            self.fcm.motors_off()
            self.mission_complete_time = time.time()
            self.state = TaskState.MISSION_COMPLETE
        
        # If we've reached the landing waypoint but still not low enough,
        # adjust downward in small increments
        if waypoint_reached and height > 0.1:
            log(f"At landing waypoint but still at height {height:.2f}m, continuing descent")
            landing_target = (current_position[0], current_position[1], 0.05)  # Target very close to ground
            self.fcm.set_position(*landing_target)
        
        return True
    
    def _handle_mission_complete(self):
        """Handle mission complete task state"""
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
        
        # Time to send mission complete message
        if self.wireless.connect():
            message = format_mission_complete_message()
            log(f"Sending mission complete message: {message}")
            
            if self.wireless.send_mission_complete(message):
                log("Mission complete message sent successfully")
                self.wireless.disconnect()
                return False  # Mission is complete, exit main loop
            else:
                log("Failed to send mission complete message", "ERROR")
                # We'll try again next iteration
                return True
        else:
            log("Failed to connect to Wi-Fi", "ERROR")
            # We'll try again next iteration
            return True
    
    def _handle_abort(self):
        """Handle abort task state"""
        display_led_pattern("ERROR", 0.2)
        
        # Perform emergency landing
        self.emergency_landing()
        
        log("Mission aborted")
        return False  # Exit main loop