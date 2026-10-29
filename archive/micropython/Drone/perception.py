"""
Simplified perception system module.
Integrates sensor data with focus on height measurement.
"""
import machine
import time
from config import HEIGHT_THRESHOLD
from utils import log

class HeightSensor:
    """
    Class for handling height sensor (VL53L0X) data.
    Simplified implementation that provides height estimation.
    """
    def __init__(self, scl_pin=5, sda_pin=4):
        """Initialize I2C connection for height sensor"""
        try:
            self.i2c = machine.I2C(0, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
            self.initialized = True
            log("Height sensor initialized successfully")
        except Exception as e:
            self.initialized = False
            log(f"Height sensor initialization failed: {e}", "ERROR")
    
    def get_height(self):
        """
        Get height measurement from sensor.
        In real implementation, this would read from VL53L0X.
        For simplicity, we're simulating height measurements.
        """
        if not self.initialized:
            return None
        
        # This is a placeholder for actual sensor reading
        # In a real implementation, this would read from the VL53L0X sensor
        # For test purposes, we're returning a simulated value
        # For actual implementation, replace with sensor-specific code
        simulated_height = 2.7  # Simulated height in meters
        return simulated_height
    
    def is_at_height(self, target_height):
        """Check if current height is within threshold of target height"""
        current_height = self.get_height()
        if current_height is None:
            return False
        
        return abs(current_height - target_height) < HEIGHT_THRESHOLD

class PerceptionSystem:
    """
    Main perception system class that integrates all sensors.
    Simplified version focusing on height data only.
    """
    def __init__(self):
        """Initialize all sensors"""
        self.height_sensor = HeightSensor()
        self._current_position = [0, 0, 0]  # x, y, z in meters
        self._last_update_time = time.ticks_ms()
    
    def update(self, fcm_position_data=None):
        """
        Update perception data.
        In a real implementation, this would fuse data from multiple sensors.
        For simplicity, we're using simulated position updates.
        """
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0  # seconds
        self._last_update_time = current_time
        
        # Update position based on FCM data if available
        if fcm_position_data:
            self._current_position = fcm_position_data
        
        # In a real implementation, we would integrate height sensor data
        # with other position information for more accurate positioning
        height = self.height_sensor.get_height()
        if height is not None:
            self._current_position[2] = height
        
        return self._current_position
    
    def get_current_position(self):
        """Return current position estimate"""
        return self._current_position
    
    def get_height(self):
        """Return current height estimate"""
        return self._current_position[2]
    
    def is_at_target(self, target_position, threshold):
        """Check if current position is within threshold of target"""
        return all(abs(c - t) < threshold for c, t in zip(self._current_position, target_position))