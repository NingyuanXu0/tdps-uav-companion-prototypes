"""
无人机系统工具函数 - 起飞降落测试版本
包含日志记录和LED控制的辅助函数
"""
import time
import machine
import uos
from config import LED_PATTERNS, DEBUG_MODE

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

# 创建日志文件
LOG_FILENAME = "takeoff_landing_test_log.txt"
try:
    # 检查日志文件是否存在
    log_exists = False
    for file in uos.listdir():
        if file == LOG_FILENAME:
            log_exists = True
            break
    
    # 如果不存在则创建日志文件
    if not log_exists:
        with open(LOG_FILENAME, "w") as f:
            f.write("=== 起飞降落测试日志 ===\n")
except:
    pass  # 即使日志文件创建失败也继续

def current_time_ms():
    """返回当前时间（毫秒）"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return time.ticks_diff(time.ticks_ms(), start_time)

def log(message, level="INFO"):
    """
    记录带级别和时间戳的消息
    同时写入日志文件（如果可用）
    """
    if DEBUG_MODE or level in ["ERROR", "WARNING"]:
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        log_message = "[{}] {}: {}".format(time_str, level, message)
        print(log_message)
        
        # 写入日志文件（如果可能）
        try:
            with open(LOG_FILENAME, "a") as f:
                f.write(log_message + "\n")
        except:
            pass  # 即使日志写入失败也继续

def display_led_pattern(pattern_name, duration=None):
    """
    显示LED闪烁模式以指示状态
    
    参数:
        pattern_name: LED_PATTERNS中的模式名称
        duration: 可选的显示持续时间（秒）
    """
    if pattern_name not in LED_PATTERNS:
        log(f"未知的LED模式: {pattern_name}", "WARNING")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    start_time = current_time_ms()
    
    while True:
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
            
        # 根据持续时间检查是否应退出
        if duration is None or time_diff_ms(start_time) >= duration * 1000:
            break

def format_test_result(test_start_time, test_complete_time):
    """格式化测试结果消息"""
    if test_start_time and test_complete_time:
        duration = test_complete_time - test_start_time
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        return f"起飞降落测试完成于 {time_str}，总时长 {duration:.1f} 秒"
    else:
        return "起飞降落测试完成"

def apply_moving_average(values, new_value, window_size=5):
    """
    应用移动平均滤波器
    
    参数:
        values: 先前值的列表
        new_value: 要添加的新值
        window_size: 移动平均窗口大小
    
    返回:
        更新后的值列表和滤波值
    """
    # 添加新值到列表
    values.append(new_value)
    
    # 只保留最后window_size个值
    if len(values) > window_size:
        values = values[-window_size:]
    
    # 计算平均值
    filtered_value = sum(values) / len(values)
    
    return values, filtered_value

"""
# 注释掉的完整任务功能
# def format_mission_complete_message():
#     格式化任务完成消息
#     pass

# def vector_distance(pos1, pos2):
#     计算两个3D点之间的欧几里得距离
#     pass

# def is_position_reached(current_pos, target_pos, threshold):
#     检查是否到达目标位置
#     pass

# def calculate_velocity_vector(current_pos, target_pos, max_speed):
#     计算朝向目标的速度向量
#     pass
"""