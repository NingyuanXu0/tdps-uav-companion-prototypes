"""
无人机系统工具函数
包含日志记录、LED控制等辅助功能
简化版本，移除了Wi-Fi相关功能

作者: Score99团队
日期: 2025年4月
"""
import time
import machine
import uos
from config import LED_PATTERNS, DEBUG_MODE

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

# 创建日志文件
LOG_FILENAME = "drone_log.txt"
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
            f.write("=== 无人机系统日志 ===\n")
except:
    pass  # 即使日志文件创建失败也继续

def log(message, level="信息"):
    """
    记录日志消息
    
    参数:
        message: 日志消息
        level: 日志级别（信息/警告/错误）
    """
    if DEBUG_MODE or level in ["错误", "警告"]:
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        log_message = "[{}] {}: {}".format(time_str, level, message)
        print(log_message)
        
        # 写入日志文件
        try:
            with open(LOG_FILENAME, "a") as f:
                f.write(log_message + "\n")
        except:
            pass

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
    else:
        # 在指定时间内重复模式
        start_time = time.time()
        while time.time() - start_time < duration:
            for state, delay in pattern:
                led.value(state)
                time.sleep(delay)
                if time.time() - start_time >= duration:
                    break

def format_mission_complete_message():
    """格式化任务完成消息"""
    timestamp = time.localtime()
    time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
        timestamp[0], timestamp[1], timestamp[2],
        timestamp[3], timestamp[4], timestamp[5]
    )
    return "任务完成于 " + time_str

def is_position_reached(current_pos, target_pos, threshold):
    """
    检查当前位置是否在目标位置的阈值范围内
    
    参数:
        current_pos: 当前[x, y, z]位置
        target_pos: 目标[x, y, z]位置
        threshold: 距离阈值（米）
    
    返回:
        如果到达位置则返回True
    """
    # 对于预设路径导航，使用更宽松的水平阈值
    xy_threshold = threshold * 2.0  # x,y更宽松
    z_threshold = threshold        # 高度保持正常精度
    
    xy_distance = ((current_pos[0] - target_pos[0])**2 + 
                   (current_pos[1] - target_pos[1])**2) ** 0.5
    z_distance = abs(current_pos[2] - target_pos[2])
    
    return xy_distance < xy_threshold and z_distance < z_threshold

def current_time_ms():
    """返回当前时间（毫秒）"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return time.ticks_diff(time.ticks_ms(), start_time)