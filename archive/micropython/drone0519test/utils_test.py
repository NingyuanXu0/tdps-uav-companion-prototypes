"""
utils.py 的单元测试
主要测试日志记录、LED控制和其他辅助功能
"""
import unittest
from unittest.mock import patch, MagicMock
import time
import sys
import io

# 将需要测试的模块导入前，先模拟必要的依赖
sys.modules['machine'] = MagicMock()
sys.modules['uos'] = MagicMock()

# 导入被测试模块
import utils

class TestUtils(unittest.TestCase):
    """测试utils.py中的工具函数"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟machine.Pin
        self.mock_pin = MagicMock()
        utils.led = self.mock_pin
        
        # 捕获标准输出
        self.held_output = io.StringIO()
        sys.stdout = self.held_output
    
    def tearDown(self):
        """测试后的清理工作"""
        # 恢复标准输出
        sys.stdout = sys.__stdout__
    
    def test_log(self):
        """测试日志记录功能"""
        # 测试INFO级别的日志
        utils.log("测试信息")
        output = self.held_output.getvalue()
        self.assertIn("INFO: 测试信息", output)
        
        # 测试WARNING级别的日志
        self.held_output = io.StringIO()  # 重置捕获
        sys.stdout = self.held_output
        utils.log("测试警告", "WARNING")
        output = self.held_output.getvalue()
        self.assertIn("WARNING: 测试警告", output)
    
    @patch('time.sleep')
    def test_display_led_pattern(self, mock_sleep):
        """测试LED模式显示功能"""
        # 模拟LED_PATTERNS
        utils.LED_PATTERNS = {
            "TEST": [(1, 0.1), (0, 0.1)]
        }
        
        # 测试无限循环模式
        utils.display_led_pattern("TEST", 0.5)
        
        # 验证LED控制和sleep调用
        self.assertEqual(self.mock_pin.value.call_count, 2)
        self.assertEqual(mock_sleep.call_count, 2)
    
    def test_vector_distance(self):
        """测试向量距离计算"""
        pos1 = [0, 0, 0]
        pos2 = [3, 4, 0]
        distance = utils.vector_distance(pos1, pos2)
        self.assertEqual(distance, 5.0)
    
    def test_is_position_reached(self):
        """测试位置到达检测"""
        current_pos = [1, 1, 1]
        target_pos = [1.1, 1.1, 1.1]
        threshold = 0.2
        result = utils.is_position_reached(current_pos, target_pos, threshold)
        self.assertTrue(result)
        
        # 超出阈值测试
        target_pos = [1.5, 1.5, 1.5]
        result = utils.is_position_reached(current_pos, target_pos, threshold)
        self.assertFalse(result)
    
    def test_calculate_velocity_vector(self):
        """测试速度向量计算"""
        current_pos = [0, 0, 0]
        target_pos = [10, 0, 0]
        max_speed = 2.0
        velocity = utils.calculate_velocity_vector(current_pos, target_pos, max_speed)
        self.assertEqual(velocity, (2.0, 0, 0))
    
    def test_apply_moving_average(self):
        """测试移动平均滤波"""
        values = [1, 2, 3]
        new_value = 4
        window_size = 3
        
        updated_values, filtered_value = utils.apply_moving_average(values, new_value, window_size)
        
        self.assertEqual(updated_values, [1, 2, 3, 4])
        self.assertEqual(filtered_value, 2.5)
        
        # 测试超过窗口大小
        values = [1, 2, 3, 4]
        new_value = 5
        updated_values, filtered_value = utils.apply_moving_average(values, new_value, window_size)
        
        self.assertEqual(updated_values, [3, 4, 5])
        self.assertEqual(filtered_value, 4.0)

if __name__ == '__main__':
    unittest.main()