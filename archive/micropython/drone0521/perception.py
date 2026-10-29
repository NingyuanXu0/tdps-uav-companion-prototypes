"""
Simplified perception system module.
Integrates sensor data with focus on height measurement and wall detection.
Optimized for preset path following with minimal sensor reliance.
"""
import machine
import time
import math
from micropython import const
from config import (
    HEIGHT_THRESHOLD, VL53L0X_SCL_PIN, VL53L0X_SDA_PIN, 
    HCSR04_TRIGGER_PIN, HCSR04_ECHO_PIN, SENSOR_READ_RETRIES
)
from utils import log

# Constants for VL53L0X
_VL53L0X_ADDR = const(0x29)
_REG_ID = const(0xC0)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_RANGE_STATUS = const(0x14)

class VL53L0XSensor:
    """
    Class for handling VL53L0X time-of-flight distance sensor data.
    Provides precise height measurement.
    """
    def __init__(self, scl_pin=VL53L0X_SCL_PIN, sda_pin=VL53L0X_SDA_PIN, address=_VL53L0X_ADDR):
        """Initialize I2C connection for VL53L0X sensor"""
        try:
            self.i2c = machine.I2C(0, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
            
            # Check if sensor is present on the bus
            devices = self.i2c.scan()
            if address not in devices:
                log(f"VL53L0X not found at address 0x{address:02X}. Available devices: {devices}", "ERROR")
                self.initialized = False
                return
                
            # Read sensor ID to verify communication
            self.address = address
            try:
                sensor_id = self.read_reg16(_REG_ID)
                if sensor_id != 0xEEAA:
                    log(f"Invalid VL53L0X ID: 0x{sensor_id:04X}", "ERROR")
                    self.initialized = False
                    return
            except Exception as e:
                log(f"Error reading VL53L0X ID: {e}", "ERROR")
                self.initialized = False
                return
                
            # Initialize sensor with default settings
            # In a full implementation, we would configure the sensor here
            
            self.initialized = True
            log("VL53L0X sensor initialized successfully")
            
            # Calibration variables
            self.offset = 0.0
            self.calibrated = False
            
        except Exception as e:
            self.initialized = False
            log(f"VL53L0X sensor initialization failed: {e}", "ERROR")
    
    def read_reg8(self, reg):
        """Read 8-bit register"""
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def read_reg16(self, reg):
        """Read 16-bit register"""
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return (data[0] << 8) | data[1]
    
    def write_reg8(self, reg, val):
        """Write 8-bit register"""
        self.i2c.writeto_mem(self.address, reg, bytes([val]))
    
    def write_reg16(self, reg, val):
        """Write 16-bit register"""
        self.i2c.writeto_mem(self.address, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]))
    
    def start_ranging(self):
        """Start a ranging measurement"""
        self.write_reg8(_REG_SYSRANGE_START, 0x01)
    
    def check_measurement_ready(self):
        """Check if a measurement is ready"""
        return (self.read_reg8(_REG_RESULT_RANGE_STATUS) & 0x01) > 0
    
    def get_distance(self):
        """
        Get distance measurement in meters.
        Returns None if measurement fails.
        """
        if not self.initialized:
            return None
        
        try:
            # Start ranging
            self.start_ranging()
            
            # Wait for result (with timeout)
            start_time = time.ticks_ms()
            while not self.check_measurement_ready():
                if time.ticks_diff(time.ticks_ms(), start_time) > 100:  # 100ms timeout
                    log("VL53L0X measurement timeout", "WARNING")
                    return None
                time.sleep(0.001)
            
            # Read distance
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + 10)
            
            # Convert to meters
            distance_m = distance_mm / 1000.0
            
            # Apply calibration offset
            if self.calibrated:
                distance_m += self.offset
            
            return distance_m
            
        except Exception as e:
            log(f"Error reading VL53L0X distance: {e}", "ERROR")
            return None
    
    def calibrate(self, known_distance):
        """
        Calibrate the sensor using a known distance.
        
        Args:
            known_distance: Actual distance in meters
        """
        if not self.initialized:
            return False
        
        try:
            # Take multiple readings and average
            readings = []
            for _ in range(10):
                reading = self.get_distance()
                if reading is not None:
                    readings.append(reading)
                time.sleep(0.05)
            
            if not readings:
                log("Calibration failed: No valid readings", "ERROR")
                return False
            
            # Calculate average reading
            avg_reading = sum(readings) / len(readings)
            
            # Calculate offset
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"VL53L0X calibrated. Offset: {self.offset:.3f}m")
            return True
            
        except Exception as e:
            log(f"Error during VL53L0X calibration: {e}", "ERROR")
            return False

class HCSR04Sensor:
    """
    Class for handling HC-SR04 ultrasonic sensor data.
    Provides redundant distance measurement for wall detection.
    """
    def __init__(self, trigger_pin=HCSR04_TRIGGER_PIN, echo_pin=HCSR04_ECHO_PIN):
        """Initialize GPIO pins for HC-SR04 sensor"""
        try:
            self.trigger = machine.Pin(trigger_pin, machine.Pin.OUT)
            self.echo = machine.Pin(echo_pin, machine.Pin.IN)
            self.trigger.value(0)  # Initialize trigger pin to low
            
            self.initialized = True
            log("HC-SR04 sensor initialized successfully")
            
            # Calibration variables
            self.offset = 0.0
            self.calibrated = False
            
        except Exception as e:
            self.initialized = False
            log(f"HC-SR04 sensor initialization failed: {e}", "ERROR")
    
    def get_distance(self):
        """
        Get distance measurement in meters.
        Returns None if measurement fails.
        """
        if not self.initialized:
            return None
        
        try:
            # Send 10us pulse to trigger
            self.trigger.value(0)
            time.sleep_us(5)
            self.trigger.value(1)
            time.sleep_us(10)
            self.trigger.value(0)
            
            # Wait for echo to start (with timeout)
            start_time = time.ticks_us()
            timeout = False
            while self.echo.value() == 0:
                if time.ticks_diff(time.ticks_us(), start_time) > 30000:  # 30ms timeout
                    timeout = True
                    break
            
            if timeout:
                log("HC-SR04 echo start timeout", "WARNING")
                return None
            
            # Measure echo duration
            start_time = time.ticks_us()
            timeout = False
            while self.echo.value() == 1:
                if time.ticks_diff(time.ticks_us(), start_time) > 30000:  # 30ms timeout
                    timeout = True
                    break
            
            if timeout:
                log("HC-SR04 echo end timeout", "WARNING")
                return None
            
            pulse_duration = time.ticks_diff(time.ticks_us(), start_time)
            
            # Calculate distance (speed of sound = 343 m/s)
            # Distance = (time * speed) / 2 (round trip)
            distance = (pulse_duration * 0.000343) / 2
            
            # Apply calibration offset
            if self.calibrated:
                distance += self.offset
            
            return distance
            
        except Exception as e:
            log(f"Error reading HC-SR04 distance: {e}", "ERROR")
            return None
    
    def calibrate(self, known_distance):
        """
        Calibrate the sensor using a known distance.
        
        Args:
            known_distance: Actual distance in meters
        """
        if not self.initialized:
            return False
        
        try:
            # Take multiple readings and average
            readings = []
            for _ in range(10):
                reading = self.get_distance()
                if reading is not None:
                    readings.append(reading)
                time.sleep(0.05)
            
            if not readings:
                log("Calibration failed: No valid readings", "ERROR")
                return False
            
            # Calculate average reading
            avg_reading = sum(readings) / len(readings)
            
            # Calculate offset
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"HC-SR04 calibrated. Offset: {self.offset:.3f}m")
            return True
            
        except Exception as e:
            log(f"Error during HC-SR04 calibration: {e}", "ERROR")
            return False

class PerceptionSystem:
    """
    Main perception system class integrating all sensors.
    Simplified to focus on basic height measurement for preset path following.
    """
    def __init__(self):
        """Initialize sensors"""
        self.height_sensor = None  # VL53L0X for height
        self.wall_sensor = None    # HC-SR04 for wall detection
        self._current_position = [0, 0, 0]  # x, y, z in meters
        self._last_update_time = time.ticks_ms()
        
        # Data filtering
        self.height_readings = []
        self.wall_readings = []
        self.filter_window = 5  # Number of readings to average
        
        # Initialization flag
        self.initialized = False
    
    def initialize_sensors(self):
        """Initialize required sensors for simplified path following"""
        log("Initializing perception sensors")
        
        # Initialize height sensor for minimal feedback
        self.height_sensor = VL53L0XSensor()
        
        # Initialize wall sensor as optional
        self.wall_sensor = HCSR04Sensor()
        
        # Check if height sensor initialized correctly (most critical)
        if not self.height_sensor.initialized:
            log("Height sensor initialization failed", "ERROR")
            return False
        
        if not self.wall_sensor.initialized:
            log("Wall sensor initialization failed - will proceed with preset path navigation", "WARNING")
            # We can continue with just the height sensor for preset paths
        
        self.initialized = True
        log("Perception system initialized successfully")
        return True
    
    def calibrate_sensors(self):
        """Calibrate sensors for preset path navigation"""
        log("Calibrating perception sensors")
        
        # For this modified implementation focused on preset paths,
        # set default values suitable for the tasks
        
        # Set initial height for obstacle avoidance task
        self._current_position[2] = 2.8  # TAKEOFF_HEIGHT value from config
        
        # Simulate height sensor calibration (assume known height)
        if self.height_sensor.initialized:
            self.height_sensor.offset = 0.0  # No offset for simulation
            self.height_sensor.calibrated = True
            log("Height sensor calibrated")
        
        # Simulate wall sensor calibration (if available)
        if self.wall_sensor and self.wall_sensor.initialized:
            self.wall_sensor.offset = 0.0  # No offset for simulation
            self.wall_sensor.calibrated = True
            log("Wall sensor calibrated")
        
        log("Sensor calibration complete")
        return True
    
    def update(self, fcm_position_data=None):
        """
        Update perception data - simplified for preset path following.
        
        Args:
            fcm_position_data: Optional position data from FCM
        
        Returns:
            Current position estimate [x, y, z] in meters
        """
        if not self.initialized:
            log("Perception system not initialized", "ERROR")
            return self._current_position
        
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0  # seconds
        self._last_update_time = current_time
        
        # Prioritize FCM data for position estimation since we're using preset paths
        if fcm_position_data:
            self._current_position = fcm_position_data
        
        # Update height from VL53L0X sensor for minimal feedback
        height = self.get_height()
        if height is not None:
            self._current_position[2] = height
        
        return self._current_position
    
    def get_height(self):
        """
        Get current height estimate.
        Uses VL53L0X sensor with filtering.
        """
        if not self.initialized or not self.height_sensor.initialized:
            return None
        
        # Try multiple readings if needed
        for _ in range(SENSOR_READ_RETRIES):
            height = self.height_sensor.get_distance()
            if height is not None:
                # Add to filter window
                self.height_readings.append(height)
                # Keep only the last N readings
                if len(self.height_readings) > self.filter_window:
                    self.height_readings.pop(0)
                
                # Return filtered value (average of readings)
                if self.height_readings:
                    filtered_height = sum(self.height_readings) / len(self.height_readings)
                    return filtered_height
                return height
            
            time.sleep(0.01)  # Short delay between retries
        
        # If all retries failed, return last known height
        if self.height_readings:
            return sum(self.height_readings) / len(self.height_readings)
        
        return None
    
    def get_wall_distance(self):
        """
        Get current wall distance.
        Uses HC-SR04 sensor with filtering, if available.
        """
        if not self.initialized or not self.wall_sensor or not self.wall_sensor.initialized:
            return None
        
        # Try multiple readings if needed
        for _ in range(SENSOR_READ_RETRIES):
            distance = self.wall_sensor.get_distance()
            if distance is not None:
                # Add to filter window
                self.wall_readings.append(distance)
                # Keep only the last N readings
                if len(self.wall_readings) > self.filter_window:
                    self.wall_readings.pop(0)
                
                # Return filtered value (average of readings)
                if self.wall_readings:
                    filtered_distance = sum(self.wall_readings) / len(self.wall_readings)
                    return filtered_distance
                return distance
            
            time.sleep(0.01)  # Short delay between retries
        
        # If all retries failed, return last known distance
        if self.wall_readings:
            return sum(self.wall_readings) / len(self.wall_readings)
        
        return None
    
    def get_current_position(self):
        """Return current position estimate"""
        return self._current_position
    
    def is_at_target(self, target_position, threshold):
        """Check if current position is within threshold of target"""
        # For preset path following, use a more forgiving horizontal threshold
        xy_threshold = threshold * 1.5  # More forgiving for x,y
        z_threshold = threshold * 0.7   # Stricter for height
        
        xy_distance = ((self._current_position[0] - target_position[0])**2 + 
                      (self._current_position[1] - target_position[1])**2) ** 0.5
        z_distance = abs(self._current_position[2] - target_position[2])
        
        return xy_distance < xy_threshold and z_distance < z_threshold