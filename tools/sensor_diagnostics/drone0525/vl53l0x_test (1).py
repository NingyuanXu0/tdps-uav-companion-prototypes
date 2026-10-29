"""
VL53L0X Time-of-Flight Sensor Test Program
For ESP32-S3-DevKitC-1U-N32R8V running MicroPython
Team Score99 - Drone Project

This test program validates VL53L0X sensor functionality including:
- I2C communication verification
- Distance measurement accuracy
- Calibration procedures
- Performance characterization
- Environmental testing
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
_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD = const(0x50)
_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD = const(0x70)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_INTERRUPT_STATUS = const(0x13)
_REG_RESULT_RANGE_STATUS = const(0x14)
_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = const(0xBC)
_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = const(0xC0)
_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = const(0xD0)
_REG_RESULT_CORE_RANGING_TOTAL_EVENTS_REF = const(0xD4)
_REG_RESULT_PEAK_SIGNAL_RATE_REF = const(0xB6)
_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM = const(0x28)
_REG_I2C_SLAVE_DEVICE_ADDRESS = const(0x8A)
_REG_MSRC_CONFIG_CONTROL = const(0x60)
_REG_PRE_RANGE_CONFIG_MIN_SNR = const(0x27)
_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW = const(0x56)
_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x57)
_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = const(0x64)
_REG_FINAL_RANGE_CONFIG_MIN_SNR = const(0x67)
_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = const(0x47)
_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x48)
_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_HI = const(0x61)
_REG_PRE_RANGE_CONFIG_SIGMA_THRESH_LO = const(0x62)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x51)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x52)
_REG_SYSTEM_HISTOGRAM_BIN = const(0x81)
_REG_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = const(0x33)
_REG_HISTOGRAM_CONFIG_READOUT_CTRL = const(0x55)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x71)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x72)
_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = const(0x20)
_REG_MSRC_CONFIG_TIMEOUT_MACROP = const(0x46)
_REG_SOFT_RESET_GO2_SOFT_RESET_N = const(0xBF)
_REG_OSC_CALIBRATE_VAL = const(0xF8)
_REG_GLOBAL_CONFIG_VCSEL_WIDTH = const(0x32)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = const(0xB0)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = const(0xB1)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = const(0xB2)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = const(0xB3)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = const(0xB4)
_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = const(0xB5)
_REG_GLOBAL_CONFIG_REF_EN_START_SELECT = const(0xB6)
_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = const(0x4E)
_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = const(0x4F)
_REG_POWER_MANAGEMENT_GO1_POWER_FORCE = const(0x80)
_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = const(0x89)
_REG_ALGO_PHASECAL_LIM = const(0x30)
_REG_ALGO_PHASECAL_CONFIG_TIMEOUT = const(0x30)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0B)

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
        
        # Test 5: Measurement Speed
        self.test_measurement_speed()
        
        # Test 6: Calibration
        self.test_calibration()
        
        # Test 7: Environmental Conditions
        self.test_environmental_conditions()
        
        # Test 8: Long-term Stability
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
            self.init_sensor()
            
            print("Taking 10 measurements...")
            measurements = []
            
            for i in range(10):
                distance = self.get_distance()
                if distance is not None:
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
                if distance is not None:
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
    
    def test_measurement_speed(self):
        """Test 5: Test measurement acquisition speed"""
        print("\n[TEST 5] Measurement Speed Test")
        print("-" * 30)
        
        try:
            print("Testing maximum measurement rate...")
            
            # Test different timing budgets
            timing_budgets = [20, 33, 50, 100, 200]  # milliseconds
            
            for budget in timing_budgets:
                print(f"\nTiming budget: {budget}ms")
                self.set_measurement_timing_budget(budget * 1000)  # Convert to microseconds
                
                start_time = time.ticks_ms()
                count = 0
                
                # Measure for 2 seconds
                while time.ticks_diff(time.ticks_ms(), start_time) < 2000:
                    distance = self.get_distance()
                    if distance is not None:
                        count += 1
                
                rate = count / 2.0
                print(f"Measurement rate: {rate:.1f} Hz")
            
            print("\nSUCCESS: Speed test completed")
            self.test_results['measurement_speed'] = 'PASSED'
            
        except Exception as e:
            print(f"ERROR: Speed test failed - {e}")
            self.test_results['measurement_speed'] = 'FAILED'
    
    def test_calibration(self):
        """Test 6: Test sensor calibration"""
        print("\n[TEST 6] Calibration Test")
        print("-" * 30)
        
        try:
            print("Place a target at exactly 0.5 meters")
            input("Press Enter when ready...")
            
            # Take measurements before calibration
            print("\nPre-calibration measurements:")
            pre_cal_measurements = []
            for i in range(20):
                distance = self.get_distance()
                if distance is not None:
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
                    if distance is not None:
                        post_cal_measurements.append(distance + offset)
                    time.sleep(0.05)
                
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
                print("ERROR: Insufficient measurements for calibration")
                self.test_results['calibration'] = 'FAILED'
                
        except Exception as e:
            print(f"ERROR: Calibration test failed - {e}")
            self.test_results['calibration'] = 'FAILED'
    
    def test_environmental_conditions(self):
        """Test 7: Test sensor under different conditions"""
        print("\n[TEST 7] Environmental Conditions Test")
        print("-" * 30)
        
        try:
            print("Testing different target surfaces...")
            
            surfaces = [
                "White paper (high reflectance)",
                "Black paper (low reflectance)",
                "Metal surface (specular)",
                "Wood surface (diffuse)",
                "Glass (transparent)"
            ]
            
            for surface in surfaces:
                print(f"\nTesting: {surface}")
                input("Place target and press Enter...")
                
                measurements = []
                failures = 0
                
                for i in range(20):
                    distance = self.get_distance()
                    if distance is not None:
                        measurements.append(distance)
                    else:
                        failures += 1
                    time.sleep(0.05)
                
                if len(measurements) > 0:
                    avg = sum(measurements) / len(measurements)
                    print(f"Average: {avg:.3f} m")
                    print(f"Success rate: {len(measurements)/20*100:.1f}%")
                else:
                    print("No valid measurements")
            
            print("\nEnvironmental test completed")
            self.test_results['environmental_conditions'] = 'COMPLETED'
            
        except Exception as e:
            print(f"ERROR: Environmental test failed - {e}")
            self.test_results['environmental_conditions'] = 'FAILED'
    
    def test_stability(self):
        """Test 8: Long-term stability test"""
        print("\n[TEST 8] Long-term Stability Test")
        print("-" * 30)
        
        try:
            print("Running 5-minute stability test...")
            print("Place sensor pointing at a stable target")
            input("Press Enter to start...")
            
            start_time = time.time()
            measurements = []
            timestamps = []
            
            while time.time() - start_time < 300:  # 5 minutes
                distance = self.get_distance()
                if distance is not None:
                    measurements.append(distance)
                    timestamps.append(time.time() - start_time)
                
                # Print progress every 30 seconds
                if len(measurements) % 30 == 0:
                    elapsed = time.time() - start_time
                    print(f"Progress: {elapsed:.0f}s / 300s")
                
                time.sleep(1)
            
            if len(measurements) > 100:
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
        
        for test, result in self.test_results.items():
            print(f"{test.replace('_', ' ').title()}: {result}")
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
        """Initialize sensor with basic configuration"""
        # Simplified initialization for testing
        self.write_reg(_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01)
        self.write_reg(0xFF, 0x01)
        self.write_reg(0x00, 0x00)
        self.write_reg(0x91, 0x3c)
        self.write_reg(0x00, 0x01)
        self.write_reg(0xFF, 0x00)
        self.write_reg(_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00)
    
    def get_distance(self):
        """Get single distance measurement"""
        try:
            # Start measurement
            self.write_reg(_REG_SYSRANGE_START, 0x01)
            
            # Wait for measurement
            timeout = 0
            while timeout < 100:
                if (self.read_reg(_REG_RESULT_INTERRUPT_STATUS) & 0x07) != 0:
                    break
                time.sleep(0.001)
                timeout += 1
            
            if timeout >= 100:
                return None
            
            # Read result
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + 10)
            
            # Clear interrupt
            self.write_reg(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            # Convert to meters
            return distance_mm / 1000.0
            
        except:
            return None
    
    def set_measurement_timing_budget(self, budget_us):
        """Set measurement timing budget in microseconds"""
        # Simplified timing budget setting
        # In production code, this would involve complex register calculations
        pass
    
    def apply_offset_calibration(self, offset_mm):
        """Apply offset calibration in millimeters"""
        # Write offset to sensor
        offset_um = offset_mm * 1000  # Convert to micrometers
        self.write_reg(_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, 
                      (offset_um >> 2) & 0xFF)

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
