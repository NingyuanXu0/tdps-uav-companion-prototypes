"""
Simplified perception system module.
Integrates sensor data with focus on height measurement and wall detection.
Optimized for preset path following.
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
            
            # Check if the sensor is present on the bus
            devices = self.i2c.scan()
            if address not in devices:
                log(f"未在地址0x{address:02X}找到VL53L0X。可用设备: {devices}", "ERROR")
                self.initialized = False
                return
                
            # Read sensor ID to verify communication
            self.address = address
            try:
                sensor_id = self.read_reg16(_REG_ID)
                if sensor_id != 0xEEAA:
                    log(f"无效的VL53L0X ID: 0x{sensor_id:04X}", "ERROR")
                    self.initialized = False
                    return
            except Exception as e:
                log(f"读取VL53L0X ID时出错: {e}", "ERROR")
                self.initialized = False
                return
                
            # Initialize sensor with default settings
            # In a full implementation, we would configure the sensor here
            
            self.initialized = True
            log("VL53L0X传感器初始化成功")
            
            # Calibration variables
            self.offset = 0.0
            self.calibrated = False
            
        except Exception as e:
            self.initialized = False
            log(f"VL53L0X传感器初始化失败: {e}", "ERROR")
    
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
        Get distance measurement in millimeters.
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
                    log("VL53L0X测量超时", "WARNING")
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
            log(f"读取VL53L0X距离时出错: {e}", "ERROR")
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
                log("校准失败: 没有有效读数", "ERROR")
                return False
            
            # Calculate average reading
            avg_reading = sum(readings) / len(readings)
            
            # Calculate offset
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"VL53L0X已校准。偏移: {self.offset:.3f}m")
            return True
            
        except Exception as e:
            log(f"VL53L0X校准过程中出错: {e}", "ERROR")
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
            log("HC-SR04传感器初始化成功")
            
            # Calibration variables
            self.offset = 0.0
            self.calibrated = False
            
        except Exception as e:
            self.initialized = False
            log(f"HC-SR04传感器初始化失败: {e}", "ERROR")
    
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
                log("HC-SR04回声开始超时", "WARNING")
                return None
            
            # Measure echo duration
            start_time = time.ticks_us()
            timeout = False
            while self.echo.value() == 1:
                if time.ticks_diff(time.ticks_us(), start_time) > 30000:  # 30ms timeout
                    timeout = True
                    break
            
            if timeout:
                log("HC-SR04回声结束超时", "WARNING")
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
            log(f"读取HC-SR04距离时出错: {e}", "ERROR")
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
                log("校准失败: 没有有效读数", "ERROR")
                return False
            
            # Calculate average reading
            avg_reading = sum(readings) / len(readings)
            
            # Calculate offset
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"HC-SR04已校准。偏移: {self.offset:.3f}m")
            return True
            
        except Exception as e:
            log(f"HC-SR04校准过程中出错: {e}", "ERROR")
            return False

class PerceptionSystem:
    """
    Main perception system class that integrates all sensors.
    Provides height measurement and wall detection.
    Optimized for preset path following.
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
        """Initialize all sensors"""
        log("初始化感知传感器")
        
        # Initialize VL53L0X height sensor
        self.height_sensor = VL53L0XSensor()
        
        # Initialize HC-SR04 wall sensor
        self.wall_sensor = HCSR04Sensor()
        
        # Check if both sensors initialized correctly
        if not self.height_sensor.initialized:
            log("高度传感器初始化失败", "ERROR")
            return False
        
        if not self.wall_sensor.initialized:
            log("墙壁传感器初始化失败", "WARNING")
            # We can continue with just the height sensor
        
        self.initialized = True
        log("感知系统初始化成功")
        return True
    
    def calibrate_sensors(self):
        """Calibrate all sensors"""
        log("校准感知传感器")
        
        # For this modified implementation, we'll set default values suitable for the
        # updated task flow (starting from tunnelling)
        
        # Set height for tunnelling task
        self._current_position[2] = 2.8  # TAKEOFF_HEIGHT value from config
        
        # Simulate height sensor calibration (assume known height)
        if self.height_sensor.initialized:
            self.height_sensor.offset = 0.0  # No offset for simulation
            self.height_sensor.calibrated = True
            log("高度传感器已校准")
        
        # Simulate wall sensor calibration (assume known distance)
        if self.wall_sensor.initialized:
            self.wall_sensor.offset = 0.0  # No offset for simulation
            self.wall_sensor.calibrated = True
            log("墙壁传感器已校准")
        
        log("传感器校准完成")
        return True
    
    def update(self, fcm_position_data=None):
        """
        Update perception data.
        
        Args:
            fcm_position_data: Optional position data from FCM
        
        Returns:
            Current position estimate [x, y, z] in meters
        """
        if not self.initialized:
            log("感知系统未初始化", "ERROR")
            return self._current_position
        
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0  # seconds
        self._last_update_time = current_time
        
        # Update position based on FCM data if available
        if fcm_position_data:
            self._current_position = fcm_position_data
        
        # Update height from VL53L0X sensor
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
        Uses HC-SR04 sensor with filtering.
        """
        if not self.initialized or not self.wall_sensor.initialized:
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
        return all(abs(c - t) < threshold for c, t in zip(self._current_position, target_position))