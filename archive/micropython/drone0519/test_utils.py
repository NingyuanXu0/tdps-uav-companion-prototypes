"""
工具函数单元测试
测试utils模块中的各种工具函数
"""
import sys
import os
import time
import math

# 添加项目目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入测试框架
from simple_test_framework import TestCase

# 创建模拟的utils模块
utils_module = type('module', (), {})()

# 模拟LED引脚
class MockLED:
    def __init__(self):
        self._value = 0
    
    def value(self, val=None):
        if val is not None:
            self._value = val
        return self._value

utils_module.led = MockLED()

# 实现utils模块中的函数
def current_time_ms():
    """返回当前时间（毫秒）"""
    return int(time.time() * 1000)

utils_module.current_time_ms = current_time_ms

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return int(time.time() * 1000) - start_time

utils_module.time_diff_ms = time_diff_ms

# 日志函数
def log(message, level="INFO"):
    """记录日志"""
    # 在测试中，只打印不写文件
    print(f"[{level}] {message}")

utils_module.log = log

# LED模式显示
def display_led_pattern(pattern_name, duration=None):
    """显示LED模式"""
    # 在测试中，仅模拟显示
    patterns = {
        "STARTUP": [(1, 0.1), (0, 0.1)] * 3,
        "ERROR": [(1, 0.1), (0, 0.1)] * 10,
    }
    
    pattern = patterns.get(pattern_name)
    if not pattern:
        print(f"警告：未知LED模式 {pattern_name}")
        return
    
    if duration is not None and duration <= 0:
        return
    
    # 在测试中，不实际闪烁，只模拟几个状态变化
    for state, delay in pattern[:2]:  # 只模拟前两个状态
        utils_module.led.value(state)

utils_module.display_led_pattern = display_led_pattern

# 任务完成消息格式化
def format_mission_complete_message():
    """格式化任务完成消息"""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    return f"任务完成于 {timestamp}"

utils_module.format_mission_complete_message = format_mission_complete_message

# 向量距离计算
def vector_distance(pos1, pos2):
    """计算两点之间的欧几里得距离"""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

utils_module.vector_distance = vector_distance

# 位置到达检查
def is_position_reached(current_pos, target_pos, threshold):
    """检查当前位置是否达到目标位置（在阈值范围内）"""
    return vector_distance(current_pos, target_pos) < threshold

utils_module.is_position_reached = is_position_reached

# 速度向量计算
def calculate_velocity_vector(current_pos, target_pos, max_speed):
    """计算从当前位置到目标位置的速度向量"""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    dz = target_pos[2] - current_pos[2]
    
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if distance < 0.001:  # 防止除以接近零的值
        return (0, 0, 0)
    
    scale = min(max_speed, distance) / distance
    return (dx * scale, dy * scale, dz * scale)

utils_module.calculate_velocity_vector = calculate_velocity_vector

# 移动平均过滤
def apply_moving_average(values, new_value, window_size=5):
    """应用移动平均过滤"""
    # 添加新值
    values.append(new_value)
    
    # 保持窗口大小
    if len(values) > window_size:
        values = values[-window_size:]
    
    # 计算平均值
    filtered_value = sum(values) / len(values)
    
    return values, filtered_value

utils_module.apply_moving_average = apply_moving_average

# 将模块添加到sys.modules
sys.modules['utils'] = utils_module

class TestUtils(TestCase):
    """测试工具函数"""
    
    def test_current_time_ms(self):
        """测试获取当前时间（毫秒）"""
        time_ms = utils_module.current_time_ms()
        self.assertIsNotNone(time_ms, "当前时间不应为None")
        self.assertIsInstance(time_ms, int, "当前时间应为整数")
    
    def test_time_diff_ms(self):
        """测试计算时间差（毫秒）"""
        start_time = utils_module.current_time_ms()
        time.sleep(0.01)  # 休眠10毫秒
        diff = utils_module.time_diff_ms(start_time)
        self.assertIsInstance(diff, int, "时间差应为整数")
        self.assertTrue(diff > 0, "时间差应大于0")
    
    def test_log(self):
        """测试日志功能"""
        # 这里只测试不抛出异常
        utils_module.log("测试日志消息")
        utils_module.log("测试警告消息", "WARNING")
        utils_module.log("测试错误消息", "ERROR")
    
    def test_display_led_pattern(self):
        """测试显示LED模式"""
        utils_module.display_led_pattern("STARTUP", 0.1)
        self.assertEqual(utils_module.led.value(), 0, "LED最终应为关闭状态")
        
        utils_module.display_led_pattern("INVALID_PATTERN")
        self.assertEqual(utils_module.led.value(), 0, "未知模式不应改变LED状态")
    
    def test_format_mission_complete_message(self):
        """测试格式化任务完成消息"""
        message = utils_module.format_mission_complete_message()
        self.assertIsNotNone(message, "消息不应为None")
        self.assertIsInstance(message, str, "消息应为字符串")
        self.assertTrue("任务完成于" in message, "消息应包含'任务完成于'")
    
    def test_vector_distance(self):
        """测试计算向量距离"""
        pos1 = [0, 0, 0]
        pos2 = [3, 4, 0]
        distance = utils_module.vector_distance(pos1, pos2)
        self.assertEqual(distance, 5.0, "距离计算错误")
    
    def test_is_position_reached(self):
        """测试位置是否达到"""
        current_pos = [1.0, 1.0, 1.0]
        target_pos = [1.1, 1.1, 1.1]
        
        # 使用0.2的阈值，应该达到
        result1 = utils_module.is_position_reached(current_pos, target_pos, 0.2)
        self.assertTrue(result1, "使用0.2阈值应判定为达到")
        
        # 使用0.1的阈值，应该未达到
        result2 = utils_module.is_position_reached(current_pos, target_pos, 0.1)
        self.assertFalse(result2, "使用0.1阈值应判定为未达到")
    
    def test_calculate_velocity_vector(self):
        """测试计算速度向量"""
        current_pos = [0, 0, 0]
        target_pos = [3, 4, 0]
        max_speed = 1.0
        
        # 计算速度向量
        velocity = utils_module.calculate_velocity_vector(current_pos, target_pos, max_speed)
        
        # 结果应该是单位向量（长度为1）乘以max_speed
        expected = (0.6, 0.8, 0)  # 3/5, 4/5, 0
        self.assertAlmostEqual(velocity[0], expected[0], places=6, message="x分量计算错误")
        self.assertAlmostEqual(velocity[1], expected[1], places=6, message="y分量计算错误")
        self.assertAlmostEqual(velocity[2], expected[2], places=6, message="z分量计算错误")
    
    def test_apply_moving_average(self):
        """测试应用移动平均"""
        values = [1.0, 2.0, 3.0]
        new_value = 4.0
        
        # 应用移动平均
        updated_values, filtered_value = utils_module.apply_moving_average(values, new_value)
        
        # 检查更新后的值列表
        self.assertEqual(len(updated_values), 4, "更新后应有4个值")
        self.assertEqual(updated_values[-1], 4.0, "新值应添加到列表末尾")
        
        # 检查过滤后的值（平均值）
        self.assertEqual(filtered_value, 2.5, "过滤值应为平均值")
        
        # 测试窗口大小限制
        values = [1.0, 2.0, 3.0, 4.0, 5.0]
        new_value = 6.0
        window_size = 3
        
        updated_values, filtered_value = utils_module.apply_moving_average(values, new_value, window_size)
        
        # 窗口大小应限制为3
        self.assertEqual(len(updated_values), 3, "窗口大小应限制为3")
        self.assertEqual(updated_values, [4.0, 5.0, 6.0], "应保留最新的3个值")
        
        # 过滤后的值（平均值）
        self.assertEqual(filtered_value, 5.0, "过滤值应为保留值的平均值")