"""
工具函数模块 - MAVLink版本
日志记录和LED控制功能

作者: Score99团队
日期: 2025年4月
"""
import time
import sys
import machine
from config import LED_PATTERNS, DEBUG_MODE

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

# 日志级别颜色（串口终端支持）
LOG_COLORS = {
    "信息": "",
    "警告": "\033[93m",  # 黄色
    "错误": "\033[91m",  # 红色
    "成功": "\033[92m",  # 绿色
}
COLOR_RESET = "\033[0m"

def log(message, level="信息"):
    """
    记录日志消息
    
    参数:
        message: 日志消息
        level: 日志级别（信息/警告/错误/成功）
    """
    timestamp = time.localtime()
    time_str = "{:02d}:{:02d}:{:02d}".format(
        timestamp[3], timestamp[4], timestamp[5]
    )
    
    # 添加颜色（如果支持）
    color = LOG_COLORS.get(level, "")
    if color:
        log_message = "{}[{}] {}: {}{}".format(
            color, time_str, level, message, COLOR_RESET
        )
    else:
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
    
    try:
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
    except Exception as e:
        # LED控制失败不应影响主程序
        pass

def format_hex(data):
    """格式化字节数据为十六进制字符串（调试用）"""
    return ' '.join(['{:02X}'.format(b) for b in data])

def check_connection_status():
    """检查连接状态（可扩展）"""
    # 这里可以添加更多的连接状态检查
    return True

def system_info():
    """打印系统信息"""
    import gc
    log("="*50)
    log("系统信息:")
    log("- 平台: ESP32-S3")
    log("- MicroPython版本: " + str(sys.implementation.version))
    log("- 可用内存: {} KB".format(gc.mem_free() // 1024))
    log("- 主频: {} MHz".format(machine.freq() // 1000000))
    log("="*50)