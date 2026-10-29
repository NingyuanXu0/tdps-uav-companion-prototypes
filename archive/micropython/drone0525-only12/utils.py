"""
工具函数模块 - 起飞降落测试版本
仅包含基本的日志和LED控制功能

作者: Score99团队
日期: 2025年4月
"""
import time
import machine
from config import LED_PATTERNS, DEBUG_MODE

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

def log(message, level="信息"):
    """
    记录日志消息
    
    参数:
        message: 日志消息
        level: 日志级别（信息/警告/错误）
    """
    timestamp = time.localtime()
    time_str = "{:02d}:{:02d}:{:02d}".format(
        timestamp[3], timestamp[4], timestamp[5]
    )
    log_message = "[{}] {}: {}".format(time_str, level, message)
    print(log_message)

def display_led_pattern(pattern_name, duration=None):
    """
    显示LED闪烁模式以指示状态
    
    参数:
        pattern_name: LED_PATTERNS中的模式名称
        duration: 可选的显示持续时间（秒）
    """
    if pattern_name not in LED_PATTERNS:
        log("未知的LED模式: " + pattern_name, "警告")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    
    if duration is None:
        # 只执行一次模式
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
        # 最后关闭LED
        led.value(0)
    else:
        # 在指定时间内重复模式
        start_time = time.time()
        while time.time() - start_time < duration:
            for state, delay in pattern:
                led.value(state)
                time.sleep(delay)
                if time.time() - start_time >= duration:
                    break
        # 最后关闭LED
        led.value(0)

def format_mission_complete_message():
    """格式化任务完成消息（测试版本不需要）"""
    return "测试任务完成"

def is_position_reached(current_pos, target_pos, threshold):
    """检查是否到达目标位置（测试版本简化实现）"""
    # 测试版本总是返回True
    return True

def current_time_ms():
    """返回当前时间（毫秒）"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return time.ticks_diff(time.ticks_ms(), start_time)