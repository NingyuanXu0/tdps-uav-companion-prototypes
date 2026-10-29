"""
Task manager module for handling drone mission tasks.
Implements a state machine for the six main tasks.
"""
import time
from enum import Enum
from config import (
    PATH_WAYPOINTS, TASK_TRANSITION_INDICES, POSITION_THRESHOLD,
    TAKEOFF_HEIGHT, CRUISING_HEIGHT, OBSTACLE_AVOIDANCE_HEIGHT,
    PICKUP_HEIGHT, DROPOFF_HEIGHT, LANDING_HEIGHT,
    MISSION_COMPLETE_WAIT_TIME
)
from utils import log, display_led_pattern, format_mission_complete_message, is_position_reached

class TaskState(Enum):
    """Enumeration of task states"""
    INITIALIZING = 0
    TAKEOFF = 1
    TUNNELLING = 2
    OBSTACLE_AVOIDANCE = 3
    PACKAGE_SEARCH = 4
    PACKAGE_PICKUP = 5
    MARKER_SEARCH = 6
    PACKAGE_DROPOFF = 7
    LANDING = 8
    MISSION_COMPLETE = 9
    ERROR = 10

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
    
    def initialize(self):
        """Initialize task manager and prepare for mission"""
        log("Initializing task manager")
        display_led_pattern("STARTUP", 3)
        
        # Make sure wireless module is initialized
        self.wireless.initialize()
        
        self.state = TaskState.TAKEOFF
        self.mission_start_time = time.time()
        log("Task manager initialized, starting mission")
        return True
    
    def update(self):
        """
        Update task manager state.
        This is the main state machine function called in the main loop.
        
        Returns:
            True if mission is ongoing, False if mission is complete or error occurred
        """
        # Get current position from perception system
        current_position = self.perception.get_current_position()
        
        # Check if we've reached the current waypoint
        current_waypoint = PATH_WAYPOINTS[self.current_waypoint_index]
        waypoint_reached = is_position_reached(
            current_position, current_waypoint, POSITION_THRESHOLD
        )
        
        # Update task state based on current waypoint and state
        if self.state == TaskState.TAKEOFF:
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
        
        return True
    
    def _move_to_next_waypoint(self):
        """Advance to the next waypoint and update FCM"""
        if self.current_waypoint_index < len(PATH_WAYPOINTS) - 1:
            self.current_waypoint_index += 1
            next_waypoint = PATH_WAYPOINTS[self.current_waypoint_index]
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
            self.fcm.set_position(*PATH_WAYPOINTS[0])
        
        if waypoint_reached:
            log(f"Takeoff complete, reached height {current_position[2]:.2f}m")
            self.state = TaskState.TUNNELLING
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_tunnelling(self, current_position, waypoint_reached):
        """Handle tunnelling task state"""
        display_led_pattern("TUNNELLING", 0.5)
        
        if waypoint_reached:
            log(f"Reached tunnelling waypoint {self.current_waypoint_index}")
            
            # Check if we've completed all tunnelling waypoints
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["TUNNELLING_COMPLETE"]:
                log("Tunnelling complete, transitioning to obstacle avoidance")
                self.state = TaskState.OBSTACLE_AVOIDANCE
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_obstacle_avoidance(self, current_position, waypoint_reached):
        """Handle obstacle avoidance task state"""
        display_led_pattern("OBSTACLE_AVOIDANCE", 0.5)
        
        if waypoint_reached:
            log(f"Reached obstacle avoidance waypoint {self.current_waypoint_index}")
            
            # Check if we've completed all obstacle avoidance waypoints
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["OBSTACLE_AVOIDANCE_COMPLETE"]:
                log("Obstacle avoidance complete, transitioning to package search")
                self.state = TaskState.PACKAGE_SEARCH
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_search(self, current_position, waypoint_reached):
        """Handle package search task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package search area, transitioning to pickup")
            self.state = TaskState.PACKAGE_PICKUP
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_pickup(self, current_position, waypoint_reached):
        """Handle package pickup task state"""
        display_led_pattern("PICKUP", 0.5)
        
        if waypoint_reached:
            log("Reached package pickup position")
            
            if not self.package_picked_up:
                # Simulate package pickup
                log("Package picked up successfully")
                self.package_picked_up = True
                time.sleep(1)  # Wait for stable pickup
                
            # Move to next waypoint (ascend after pickup)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["PICKUP_COMPLETE"]:
                    log("Package pickup complete, transitioning to marker search")
                    self.state = TaskState.MARKER_SEARCH
            
        return True
    
    def _handle_marker_search(self, current_position, waypoint_reached):
        """Handle marker search task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached marker search area, transitioning to dropoff")
            self.state = TaskState.PACKAGE_DROPOFF
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_package_dropoff(self, current_position, waypoint_reached):
        """Handle package dropoff task state"""
        display_led_pattern("DROPOFF", 0.5)
        
        if waypoint_reached:
            log("Reached package dropoff position")
            
            if not self.package_dropped_off and self.package_picked_up:
                # Simulate package dropoff
                log("Package dropped off successfully")
                self.package_dropped_off = True
                time.sleep(1)  # Wait for stable dropoff
                
            # Move to next waypoint (ascend after dropoff)
            if self._move_to_next_waypoint():
                if self.current_waypoint_index >= TASK_TRANSITION_INDICES["DROPOFF_COMPLETE"]:
                    log("Package dropoff complete, transitioning to landing")
                    self.state = TaskState.LANDING
            
        return True
    
    def _handle_landing(self, current_position, waypoint_reached):
        """Handle landing task state"""
        display_led_pattern("LANDING", 0.5)
        
        if waypoint_reached:
            log("Landing complete")
            self.fcm.motors_off()
            self.mission_complete_time = time.time()
            self.state = TaskState.MISSION_COMPLETE
        
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