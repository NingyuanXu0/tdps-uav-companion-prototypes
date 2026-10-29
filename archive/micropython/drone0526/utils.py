"""
无人机系统工具函数 - 起飞降落测试版本
包含日志记录和LED控制的辅助函数
修复了display_led_pattern函数的无限循环问题
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
                 如果为None，只执行一次完整的模式
    """
    if pattern_name not in LED_PATTERNS:
        log(f"未知的LED模式: {pattern_name}", "WARNING")
        return
    
    pattern = LED_PATTERNS[pattern_name]
    
    if duration is None:
        # 如果没有指定持续时间，只执行一次模式
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
        # 最后关闭LED
        led.value(0)
    else:
        # 如果指定了持续时间，在该时间内重复模式
        start_time = current_time_ms()
        while time_diff_ms(start_time) < duration * 1000:
            for state, delay in pattern:
                led.value(state)
                time.sleep(delay)
                # 检查是否超时
                if time_diff_ms(start_time) >= duration * 1000:
                    break
        # 最后关闭LED
        led.value(0)

def display_led_pattern_once(pattern_name):
    """
    显示LED闪烁模式一次
    这是一个便捷函数，用于只需要执行一次模式的场景
    """
    display_led_pattern(pattern_name, None)

def set_led(state):
    """
    直接设置LED状态
    
    参数:
        state: 0=关闭, 1=开启
    """
    led.value(state)

def blink_led(times=3, on_time=0.1, off_time=0.1):
    """
    简单的LED闪烁函数
    
    参数:
        times: 闪烁次数
        on_time: LED亮起时间（秒）
        off_time: LED熄灭时间（秒）
    """
    for _ in range(times):
        led.value(1)
        time.sleep(on_time)
        led.value(0)
        time.sleep(off_time)

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