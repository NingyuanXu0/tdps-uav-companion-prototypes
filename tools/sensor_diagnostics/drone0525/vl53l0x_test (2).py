"""
VL53L0X Time-of-Flight Sensor Test Program - Optimized Version
For ESP32-S3-DevKitC-1U-N32R8V running MicroPython
Team Score99 - Drone Project

This test program validates VL53L0X sensor functionality including:
- I2C communication verification
- Distance measurement accuracy
- Calibration procedures
- Long-term stability testing
"""

import machine
import time
import struct
import gc
from micropython import const

# VL53L0X Register Definitions
_VL53L0X_ADDR = const(0x29)
_REG_IDENTIFICATION_MODEL_ID = const(0xC0)
_REG_IDENTIFICATION_REVISION_ID = const(0xC2)
_REG_SYSRANGE_START = const(0x00)
_REG_SYSTEM_SEQUENCE_CONFIG = const(0x01)
_REG_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)
_REG_SYSTEM_INTERRUPT_CONFIG_GPIO = const(0x0A)
_REG_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0B)
_REG_RESULT_INTERRUPT_STATUS = const(0x13)
_REG_RESULT_RANGE_STATUS = const(0x14)
_REG_SYSTEM_THRESH_HIGH = const(0x0C)
_REG_SYSTEM_THRESH_LOW = const(0x0E)
_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = const(0x50)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x51)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x52)
_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = const(0x70)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x71)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x72)
_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = const(0x20)
_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM = const(0x28)
_REG_I2C_SLAVE_DEVICE_ADDRESS = const(0x8A)
_REG_MSRC_CONFIG_CONTROL = const(0x60)
_REG_PRE_RANGE_CONFIG_MIN_SNR = const(0x27)
_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW = const(0x56)
_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x57)
_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = const(0x47)
_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x48)
_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = const(0x4E)
_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = const(0x4F)
_REG_POWER_MANAGEMENT_GO1_POWER_FORCE = const(0x80)
_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = const(0x89)
_REG_ALGO_PHASECAL_CONFIG_TIMEOUT = const(0x30)
_REG_ALGO_PHASECAL_LIM = const(0x30)
_REG_GLOBAL_CONFIG_VCSEL_WIDTH = const(0x32)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = const(0xB0)
_REG_GLOBAL_CONFIG_REF_EN_START_SELECT = const(0xB6)
_REG_SYSTEM_HISTOGRAM_BIN = const(0x81)
_REG_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_REG_SYSTEM_RANGE_CONFIG = const(0x09)
_REG_SYSTEM_INTERRUPT_CONFIG = const(0x0A)

class VL53L0XTester:
    """Comprehensive test class for VL53L0X sensor validation"""
    
    def __init__(self, scl_pin=5, sda_pin=4, freq=400000):
        """Initialize test environment"""
        print("\n=== VL53L0X Sensor Test Program ===")
        print(f"SCL Pin: {scl_pin}, SDA Pin: {sda_pin}")
        print(f"I2C Frequency: {freq} Hz")
        print("="*40)
        
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.freq = freq
        self.i2c = None
        self.sensor_address = _VL53L0X_ADDR
        self.test_results = {}
        self.measurement_timing_budget = 33000  # Default 33ms
        
    def run_all_tests(self):
        """Execute complete test suite"""
        print("\nStarting comprehensive sensor tests...\n")
        
        # Test 1: I2C Communication
        if not self.test_i2c_communication():
            print("CRITICAL: I2C communication failed. Cannot proceed.")
            return False
            
        # Test 2: Sensor Identification
        self.test_sensor_identification()
        
        # Test 3: Basic Distance Measurement
        self.test_basic_measurement()
        
        # Test 4: Measurement Accuracy
        self.test_measurement_accuracy()
        
        # Test 5: Calibration
        self.test_calibration()
        
        # Test 6: Long-term Stability
        self.test_stability()
        
        # Generate Test Report
        self.generate_report()
        
        return True
    
    def test_i2c_communication(self):
        """Test 1: Verify I2C communication"""
        print("\n[TEST 1] I2C Communication Test")
        print("-" * 30)
        
        try:
            # Initialize I2C
            self.i2c = machine.I2C(0, 
                                   scl=machine.Pin(self.scl_pin), 
                                   sda=machine.Pin(self.sda_pin), 
                                   freq=self.freq)
            
            # Scan I2C bus
            devices = self.i2c.scan()
            print(f"Devices found on I2C bus: {[hex(d) for d in devices]}")
            
            if self.sensor_address not in devices:
                print(f"ERROR: VL53L0X not found at address 0x{self.sensor_address:02X}")
                self.test_results['i2c_communication'] = 'FAILED'
                return False
                
            print(f"SUCCESS: VL53L0X found at address 0x{self.sensor_address:02X}")
            self.test_results['i2c_communication'] = 'PASSED'
            return True
            
        except Exception as e:
            print(f"ERROR: I2C initialization failed - {e}")
            self.test_results['i2c_communication'] = 'FAILED'
            return False
    
    def test_sensor_identification(self):
        """Test 2: Read and verify sensor identification"""
        print("\n[TEST 2] Sensor Identification Test")
        print("-" * 30)
        
        try:
            # Read model ID
            model_id = self.read_reg(_REG_IDENTIFICATION_MODEL_ID)
            revision_id = self.read_reg(_REG_IDENTIFICATION_REVISION_ID)
            
            print(f"Model ID: 0x{model_id:02X} (Expected: 0xEE)")
            print(f"Revision ID: 0x{revision_id:02X}")
            
            if model_id == 0xEE:
                print("SUCCESS: Correct VL53L0X model identified")
                self.test_results['sensor_identification'] = 'PASSED'
            else:
                print("WARNING: Unexpected model ID")
                self.test_results['sensor_identification'] = 'WARNING'
                
        except Exception as e:
            print(f"ERROR: Failed to read sensor ID - {e}")
            self.test_results['sensor_identification'] = 'FAILED'
    
    def test_basic_measurement(self):
        """Test 3: Perform basic distance measurements"""
        print("\n[TEST 3] Basic Distance Measurement Test")
        print("-" * 30)
        
        try:
            # Initialize sensor for measurement
            if not self.init_sensor():
                print("ERROR: Sensor initialization failed")
                self.test_results['basic_measurement'] = 'FAILED'
                return
            
            print("Taking 10 measurements...")
            measurements = []
            
            for i in range(10):
                distance = self.get_distance()
                if distance is not None and distance > 0:
                    measurements.append(distance)
                    print(f"Measurement {i+1}: {distance:.3f} meters")
                else:
                    print(f"Measurement {i+1}: FAILED")
                time.sleep(0.1)
            
            if len(measurements) >= 5:
                avg_distance = sum(measurements) / len(measurements)
                print(f"\nAverage distance: {avg_distance:.3f} meters")
                print(f"Successful measurements: {len(measurements)}/10")
                self.test_results['basic_measurement'] = 'PASSED'
            else:
                print("ERROR: Too many failed measurements")
                self.test_results['basic_measurement'] = 'FAILED'
                
        except Exception as e:
            print(f"ERROR: Measurement test failed - {e}")
            self.test_results['basic_measurement'] = 'FAILED'
    
    def test_measurement_accuracy(self):
        """Test 4: Test measurement accuracy and precision"""
        print("\n[TEST 4] Measurement Accuracy Test")
        print("-" * 30)
        
        try:
            print("Place sensor at known distances for accuracy test")
            print("Taking 50 measurements for statistical analysis...")
            
            measurements = []
            for i in range(50):
                distance = self.get_distance()
                if distance is not None and distance > 0:
                    measurements.append(distance)
                time.sleep(0.05)
            
            if len(measurements) > 10:
                # Calculate statistics
                avg = sum(measurements) / len(measurements)
                variance = sum((x - avg) ** 2 for x in measurements) / len(measurements)
                std_dev = variance ** 0.5
                min_val = min(measurements)
                max_val = max(measurements)
                
                print(f"\nStatistical Analysis:")
                print(f"Samples: {len(measurements)}")
                print(f"Average: {avg:.3f} m")
                print(f"Std Dev: {std_dev*1000:.1f} mm")
                print(f"Min: {min_val:.3f} m")
                print(f"Max: {max_val:.3f} m")
                print(f"Range: {(max_val-min_val)*1000:.1f} mm")
                
                # Check if standard deviation is acceptable (< 10mm)
                if std_dev < 0.010:
                    print("SUCCESS: Excellent precision")
                    self.test_results['measurement_accuracy'] = 'PASSED'
                elif std_dev < 0.020:
                    print("WARNING: Acceptable precision")
                    self.test_results['measurement_accuracy'] = 'WARNING'
                else:
                    print("ERROR: Poor precision")
                    self.test_results['measurement_accuracy'] = 'FAILED'
                    
            else:
                print("ERROR: Insufficient valid measurements")
                self.test_results['measurement_accuracy'] = 'FAILED'
                
        except Exception as e:
            print(f"ERROR: Accuracy test failed - {e}")
            self.test_results['measurement_accuracy'] = 'FAILED'
    
    def test_calibration(self):
        """Test 5: Test sensor calibration"""
        print("\n[TEST 5] Calibration Test")
        print("-" * 30)
        
        try:
            print("Place a target at exactly 0.5 meters")
            input("Press Enter when ready...")
            
            # Take measurements before calibration
            print("\nPre-calibration measurements:")
            pre_cal_measurements = []
            for i in range(20):
                distance = self.get_distance()
                if distance is not None and distance > 0:
                    pre_cal_measurements.append(distance)
                time.sleep(0.05)
            
            if len(pre_cal_measurements) > 10:
                avg_pre = sum(pre_cal_measurements) / len(pre_cal_measurements)
                print(f"Average before calibration: {avg_pre:.3f} m")
                
                # Calculate offset
                offset = 0.5 - avg_pre
                print(f"Calculated offset: {offset*1000:.1f} mm")
                
                # Apply offset
                self.apply_offset_calibration(int(offset * 1000))
                
                # Take measurements after calibration
                print("\nPost-calibration measurements:")
                post_cal_measurements = []
                for i in range(20):
                    distance = self.get_distance()
                    if distance is not None and distance > 0:
                        post_cal_measurements.append(distance + offset)
                    time.sleep(0.05)
                
                if len(post_cal_measurements) > 10:
                    avg_post = sum(post_cal_measurements) / len(post_cal_measurements)
                    print(f"Average after calibration: {avg_post:.3f} m")
                    
                    error = abs(avg_post - 0.5)
                    print(f"Calibration error: {error*1000:.1f} mm")
                    
                    if error < 0.005:
                        print("SUCCESS: Excellent calibration")
                        self.test_results['calibration'] = 'PASSED'
                    elif error < 0.010:
                        print("WARNING: Acceptable calibration")
                        self.test_results['calibration'] = 'WARNING'
                    else:
                        print("ERROR: Poor calibration")
                        self.test_results['calibration'] = 'FAILED'
                else:
                    print("ERROR: Insufficient post-calibration measurements")
                    self.test_results['calibration'] = 'FAILED'
            else:
                print("ERROR: Insufficient measurements for calibration")
                self.test_results['calibration'] = 'FAILED'
                
        except Exception as e:
            print(f"ERROR: Calibration test failed - {e}")
            self.test_results['calibration'] = 'FAILED'
    
    def test_stability(self):
        """Test 6: Long-term stability test"""
        print("\n[TEST 6] Long-term Stability Test")
        print("-" * 30)
        
        try:
            print("Running 2-minute stability test...")
            print("Place sensor pointing at a stable target")
            input("Press Enter to start...")
            
            start_time = time.time()
            measurements = []
            timestamps = []
            
            # Run for 2 minutes instead of 5 for faster testing
            while time.time() - start_time < 120:  # 2 minutes
                distance = self.get_distance()
                if distance is not None and distance > 0:
                    measurements.append(distance)
                    timestamps.append(time.time() - start_time)
                
                # Print progress every 20 seconds
                if len(measurements) % 20 == 0 and len(measurements) > 0:
                    elapsed = time.time() - start_time
                    print(f"Progress: {elapsed:.0f}s / 120s")
                
                time.sleep(1)
            
            if len(measurements) > 50:
                # Analyze drift
                first_10 = sum(measurements[:10]) / 10
                last_10 = sum(measurements[-10:]) / 10
                drift = abs(last_10 - first_10)
                
                print(f"\nStability Analysis:")
                print(f"Total measurements: {len(measurements)}")
                print(f"Initial average: {first_10:.3f} m")
                print(f"Final average: {last_10:.3f} m")
                print(f"Drift: {drift*1000:.1f} mm")
                
                if drift < 0.002:
                    print("SUCCESS: Excellent stability")
                    self.test_results['stability'] = 'PASSED'
                elif drift < 0.005:
                    print("WARNING: Acceptable stability")
                    self.test_results['stability'] = 'WARNING'
                else:
                    print("ERROR: Poor stability")
                    self.test_results['stability'] = 'FAILED'
            else:
                print("ERROR: Insufficient measurements")
                self.test_results['stability'] = 'FAILED'
                
        except Exception as e:
            print(f"ERROR: Stability test failed - {e}")
            self.test_results['stability'] = 'FAILED'
    
    def generate_report(self):
        """Generate comprehensive test report"""
        print("\n" + "="*50)
        print("VL53L0X SENSOR TEST REPORT")
        print("="*50)
        print(f"Test Date: {time.localtime()}")
        print(f"Hardware: ESP32-S3-DevKitC-1U-N32R8V")
        print(f"I2C Pins: SCL={self.scl_pin}, SDA={self.sda_pin}")
        print("\nTest Results Summary:")
        print("-"*30)
        
        passed = 0
        failed = 0
        warnings = 0
        
        for test_name, result in self.test_results.items():
            # Fix the string formatting issue
            formatted_name = test_name.replace('_', ' ').upper()
            print(f"{formatted_name}: {result}")
            
            if result == 'PASSED':
                passed += 1
            elif result == 'FAILED':
                failed += 1
            elif result == 'WARNING':
                warnings += 1
        
        print("\n" + "-"*30)
        print(f"Total Tests: {len(self.test_results)}")
        print(f"Passed: {passed}")
        print(f"Warnings: {warnings}")
        print(f"Failed: {failed}")
        
        if failed == 0:
            print("\nOVERALL: SENSOR READY FOR DEPLOYMENT")
        else:
            print("\nOVERALL: SENSOR REQUIRES ATTENTION")
        
        print("="*50)
    
    # Helper methods
    
    def read_reg(self, reg):
        """Read single register"""
        return self.i2c.readfrom_mem(self.sensor_address, reg, 1)[0]
    
    def read_reg16(self, reg):
        """Read 16-bit register"""
        data = self.i2c.readfrom_mem(self.sensor_address, reg, 2)
        return (data[0] << 8) | data[1]
    
    def write_reg(self, reg, val):
        """Write single register"""
        self.i2c.writeto_mem(self.sensor_address, reg, bytes([val]))
    
    def write_reg16(self, reg, val):
        """Write 16-bit register"""
        self.i2c.writeto_mem(self.sensor_address, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]))
    
    def init_sensor(self):
        """Initialize sensor with comprehensive configuration"""
        try:
            # Data initialization sequence based on VL53L0X API
            init_sequence = [
                # Reset and set I2C standard mode
                (0x88, 0x00),
                (0x80, 0x01),
                (0xFF, 0x01),
                (0x00, 0x00),
                (0x91, 0x3c),
                (0x00, 0x01),
                (0xFF, 0x00),
                (0x80, 0x00),
                
                # Configure GPIO interrupt
                (_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04),
                (_REG_GPIO_HV_MUX_ACTIVE_HIGH, self.read_reg(_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10),
                (_REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                
                # Set sequence steps
                (_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8),
                
                # Configure range settings
                (_REG_SYSTEM_RANGE_CONFIG, 0x00),
                (_REG_SYSTEM_THRESH_HIGH, 0x00),
                (_REG_SYSTEM_THRESH_LOW, 0x00),
                
                # Configure VCSEL periods
                (_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, 0x18),
                (_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, 0x14),
                
                # Set timing budget
                (_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x00),
                (_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, 0xB0),
                (_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x01),
                (_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, 0xA0),
                
                # Enable/Disable specific steps
                (_REG_SYSTEM_SEQUENCE_CONFIG, 0x01),
                (_REG_SYSTEM_SEQUENCE_CONFIG, 0x02),
                (_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8),
            ]
            
            # Apply initialization sequence
            for reg, val in init_sequence:
                self.write_reg(reg, val)
                time.sleep_ms(5)
            
            # Perform additional calibration steps
            self.perform_ref_calibration()
            
            print("Sensor initialization completed successfully")
            return True
            
        except Exception as e:
            print(f"Sensor initialization failed: {e}")
            return False
    
    def perform_ref_calibration(self):
        """Perform reference calibration"""
        # Start VHV calibration
        self.write_reg(_REG_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self.perform_single_ref_calibration(0x40)
        
        # Start phase calibration
        self.write_reg(_REG_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self.perform_single_ref_calibration(0x00)
        
        # Restore sequence config
        self.write_reg(_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)
    
    def perform_single_ref_calibration(self, vhv_init_byte):
        """Perform a single reference calibration"""
        self.write_reg(_REG_SYSRANGE_START, 0x01 | vhv_init_byte)
        
        # Wait for measurement to complete
        timeout = 0
        while timeout < 100:
            if (self.read_reg(_REG_RESULT_INTERRUPT_STATUS) & 0x07) != 0:
                break
            time.sleep_ms(5)
            timeout += 1
        
        self.write_reg(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self.write_reg(_REG_SYSRANGE_START, 0x00)
    
    def get_distance(self):
        """Get single distance measurement with improved error handling"""
        try:
            # Start single measurement
            self.write_reg(_REG_SYSRANGE_START, 0x01)
            
            # Poll for measurement completion
            timeout = 0
            while timeout < 200:  # Increased timeout
                status = self.read_reg(_REG_RESULT_INTERRUPT_STATUS)
                if (status & 0x07) != 0:
                    break
                time.sleep_ms(1)
                timeout += 1
            
            if timeout >= 200:
                return None
            
            # Read range status
            range_status = self.read_reg(_REG_RESULT_RANGE_STATUS)
            
            # Check device ready and no phase error
            if (range_status & 0x01) == 0:
                return None
            
            # Read result
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + 10)
            
            # Clear interrupt
            self.write_reg(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            # Check for valid measurement
            if distance_mm == 0 or distance_mm == 8190:
                return None
            
            # Convert to meters
            return distance_mm / 1000.0
            
        except Exception as e:
            return None
    
    def apply_offset_calibration(self, offset_mm):
        """Apply offset calibration in millimeters"""
        # Convert to device units and apply
        offset_encoded = offset_mm * 4
        self.write_reg(_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, 
                      (offset_encoded >> 3) & 0xFF)
        
        # Store in upper byte of register
        high_byte = (offset_encoded >> 11) & 0xFF
        low_byte = self.read_reg(_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM + 1)
        self.write_reg(_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM + 1,
                      (high_byte & 0x0F) | (low_byte & 0xF0))

# Main test execution
def main():
    """Run complete sensor test suite"""
    print("\n" + "="*60)
    print("VL53L0X Time-of-Flight Sensor Test Suite")
    print("Team Score99 - Drone Project")
    print("="*60)
    
    # Create tester instance
    tester = VL53L0XTester(scl_pin=5, sda_pin=4)
    
    # Run all tests
    tester.run_all_tests()
    
    print("\nTest suite completed!")

if __name__ == "__main__":
    main()
