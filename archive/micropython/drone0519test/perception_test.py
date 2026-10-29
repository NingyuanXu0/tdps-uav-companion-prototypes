"""
perception.py 的单元测试
测试传感器数据处理和环境感知功能
"""
import unittest
from unittest.mock import patch, MagicMock
import time
import sys

# 模拟依赖
sys.modules['machine'] = MagicMock()
sys.modules['micropython'] = MagicMock()
sys.modules['utils'] = MagicMock()
sys.modules['utils'].log = MagicMock()

# 导入被测试模块
import perception

class TestVL53L0XSensor(unittest.TestCase):
    """测试VL53L0X传感器类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟I2C总线和设备扫描结果
        self.mock_i2c = MagicMock()
        self.mock_i2c.scan.return_value = [0x29]  # VL53L0X地址
        
        # 模拟读取寄存器返回值
        self.mock_i2c.readfrom_mem.return_value = bytes([0xEE, 0xAA])  # 有效的传感器ID
        
        # 创建测试对象前先打补丁
        patch('machine.I2C', return_value=self.mock_i2c).start()
        
        # 创建传感器对象
        self.sensor = perception.VL53L0XSensor()
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialization(self):
        """测试传感器初始化"""
        self.assertTrue(self.sensor.initialized)
        self.mock_i2c.scan.assert_called_once()
    
    def test_read_reg16(self):
        """测试16位寄存器读取"""
        self.assertEqual(self.sensor.read_reg16(0x00), 0xEEAA)
    
    def test_get_distance(self):
        """测试距离测量"""
        # 模拟测量准备就绪
        self.sensor.check_measurement_ready = MagicMock(return_value=True)
        
        # 模拟距离值 (1000mm = 1m)
        self.mock_i2c.readfrom_mem.return_value = bytes([0x03, 0xE8])
        
        distance = self.sensor.get_distance()
        self.assertEqual(distance, 1.0)  # 1000mm = 1m
    
    def test_calibrate(self):
        """测试传感器校准"""
        # 模拟多次测量结果
        self.sensor.get_distance = MagicMock(side_effect=[0.9, 0.95, 0.92, 0.93, 0.94, 
                                                         0.91, 0.92, 0.93, 0.94, 0.96])
        
        # 执行校准 (已知距离1.0m)
        result = self.sensor.calibrate(1.0)
        
        # 验证校准结果
        self.assertTrue(result)
        self.assertTrue(self.sensor.calibrated)
        # 平均测量值为0.93，偏移量应为0.07
        self.assertAlmostEqual(self.sensor.offset, 0.07, places=2)


class TestHCSR04Sensor(unittest.TestCase):
    """测试HC-SR04超声波传感器类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟GPIO引脚
        self.mock_trigger = MagicMock()
        self.mock_echo = MagicMock()
        
        # 打补丁
        patch('machine.Pin', side_effect=[self.mock_trigger, self.mock_echo]).start()
        patch('time.ticks_us', side_effect=[0, 1000, 5830]).start()
        patch('time.ticks_diff', return_value=4830).start()
        
        # 创建传感器对象
        self.sensor = perception.HCSR04Sensor()
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialization(self):
        """测试传感器初始化"""
        self.assertTrue(self.sensor.initialized)
        self.mock_trigger.value.assert_called_with(0)
    
    def test_get_distance(self):
        """测试距离测量"""
        # 模拟echo引脚状态
        self.mock_echo.value.side_effect = [0, 1, 1, 1, 0]
        
        # 测量距离
        distance = self.sensor.get_distance()
        
        # 验证触发过程
        self.mock_trigger.value.assert_any_call(1)  # 发送触发脉冲
        self.mock_trigger.value.assert_any_call(0)  # 结束触发脉冲
        
        # 验证距离计算 (4830μs对应约0.83m)
        self.assertAlmostEqual(distance, 0.83, places=2)


class TestPerceptionSystem(unittest.TestCase):
    """测试感知系统类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟传感器
        self.mock_height_sensor = MagicMock()
        self.mock_height_sensor.initialized = True
        self.mock_height_sensor.get_distance.return_value = 2.5
        
        self.mock_wall_sensor = MagicMock()
        self.mock_wall_sensor.initialized = True
        self.mock_wall_sensor.get_distance.return_value = 1.0
        
        # 打补丁
        patch('perception.VL53L0XSensor', return_value=self.mock_height_sensor).start()
        patch('perception.HCSR04Sensor', return_value=self.mock_wall_sensor).start()
        
        # 创建感知系统
        self.perception_system = perception.PerceptionSystem()
        self.perception_system.initialized = True
        self.perception_system.height_sensor = self.mock_height_sensor
        self.perception_system.wall_sensor = self.mock_wall_sensor
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_get_height(self):
        """测试高度获取"""
        height = self.perception_system.get_height()
        self.assertEqual(height, 2.5)
        self.mock_height_sensor.get_distance.assert_called_once()
    
    def test_get_wall_distance(self):
        """测试墙壁距离获取"""
        distance = self.perception_system.get_wall_distance()
        self.assertEqual(distance, 1.0)
        self.mock_wall_sensor.get_distance.assert_called_once()
    
    def test_update(self):
        """测试状态更新"""
        # 初始位置
        self.perception_system._current_position = [0, 0, 0]
        
        # 更新状态 (不提供FCM数据)
        updated_position = self.perception_system.update()
        
        # 验证更新结果 (高度应该被更新)
        self.assertEqual(updated_position, [0, 0, 2.5])
        
        # 测试提供FCM数据的情况
        fcm_position = [1, 2, 3]
        updated_position = self.perception_system.update(fcm_position)
        
        # 验证更新结果 (应该使用FCM数据，但高度仍然使用传感器数据)
        self.assertEqual(updated_position, [1, 2, 2.5])
    
    def test_is_at_target(self):
        """测试位置到达检测"""
        self.perception_system._current_position = [1, 1, 1]
        
        # 目标位置在阈值内
        target_position = [1.1, 1.1, 1.1]
        threshold = 0.2
        result = self.perception_system.is_at_target(target_position, threshold)
        self.assertTrue(result)
        
        # 目标位置超出阈值
        target_position = [1.5, 1.5, 1.5]
        result = self.perception_system.is_at_target(target_position, threshold)
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()