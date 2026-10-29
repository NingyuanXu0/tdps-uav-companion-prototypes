"""
无人机系统的工具函数
包含计时、日志记录和LED控制的辅助函数
"""
import time
import machine
from config import LED_PATTERNS, DEBUG_MODE

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

def current_time_ms():
    """返回当前时间（毫秒）"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return time.ticks_diff(time.ticks_ms(), start_time)

def log(message, level="INFO"):
    """记录带有级别和时间戳的消息"""
    if DEBUG_MODE or level in ["ERROR", "WARNING"]:
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        print("[{}] {}: {}".format(time_str, level, message))

def display_led_pattern(pattern_name, duration=None):
    """显示LED指示状态"""
    if pattern_name not in LED_PATTERNS:
        log(f"未知LED模式: {pattern_name}", "WARNING")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    start_time = current_time_ms()
    
    count = 0
    max_count = 5 if duration is None else 1000  # 防止无限循环
    
    while count < max_count:
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
        
        count += 1
        
        # 根据持续时间检查是否应该退出
        if duration is None or time_diff_ms(start_time) >= duration * 1000:
            break

def format_mission_complete_message():
    """格式化任务完成消息，包含时间戳"""
    timestamp = time.localtime()
    time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        timestamp[0], timestamp[1], timestamp[2],
        timestamp[3], timestamp[4], timestamp[5]
    )
    return f"任务完成于 {time_str}"