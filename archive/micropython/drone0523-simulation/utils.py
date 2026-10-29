"""
无人机系统工具函数 - 起飞降落测试版本
包含日志记录和LED控制的辅助函数
支持仿真模式运行
"""
import time
import gc
import machine
import uos
from config import LED_PATTERNS, DEBUG_MODE, SIMULATION_MODE

# 初始化内置LED（仿真模式下使用try-except处理）
led = None
try:
    led = machine.Pin(2, machine.Pin.OUT)
    if SIMULATION_MODE:
        log("仿真模式：LED控制已初始化")
except Exception as e:
    if SIMULATION_MODE:
        log("仿真模式：跳过LED硬件初始化")
        led = None
    else:
        print(f"LED初始化失败: {e}")
        led = None

# 创建日志文件
LOG_FILENAME = "takeoff_landing_test_log.txt"
log_file_available = False

try:
    # 检查日志文件是否存在
    log_exists = False
    try:
        for file in uos.listdir():
            if file == LOG_FILENAME:
                log_exists = True
                break
    except:
        # 如果文件系统不可用（如仿真模式），跳过文件操作
        if not SIMULATION_MODE:
            raise
    
    # 如果不存在则创建日志文件
    if not log_exists and not SIMULATION_MODE:
        with open(LOG_FILENAME, "w") as f:
            f.write("=== 起飞降落测试日志 ===\n")
        log_file_available = True
    elif not SIMULATION_MODE:
        log_file_available = True
    
except:
    log_file_available = False
    if SIMULATION_MODE:
        print("仿真模式：跳过日志文件初始化")
    else:
        print("日志文件初始化失败，将仅使用控制台输出")

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
        
        # 在仿真模式下添加标识
        mode_prefix = "[SIM] " if SIMULATION_MODE else ""
        log_message = "[{}] {}{}: {}".format(time_str, mode_prefix, level, message)
        print(log_message)
        
        # 写入日志文件（如果可能且非仿真模式）
        if log_file_available and not SIMULATION_MODE:
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
    
    if SIMULATION_MODE:
        # 仿真模式下仅记录LED状态，不实际控制硬件
        log(f"仿真LED模式: {pattern_name}", "DEBUG")
        if duration:
            time.sleep(duration)
        else:
            # 执行一个周期的模式
            for state, delay in pattern:
                log(f"仿真LED状态: {state}", "DEBUG")
                time.sleep(delay)
        return
    
    if led is None:
        log(f"LED硬件不可用，跳过模式: {pattern_name}", "WARNING")
        return
    
    try:
        while True:
            for state, delay in pattern:
                led.value(state)
                time.sleep(delay)
                
            # 根据持续时间检查是否应退出
            if duration is None or time_diff_ms(start_time) >= duration * 1000:
                break
    except Exception as e:
        log(f"LED控制出错: {e}", "WARNING")

def safe_led_control(state):
    """
    安全的LED控制函数
    
    参数:
        state: LED状态 (1=亮, 0=灭)
    """
    if SIMULATION_MODE:
        log(f"仿真LED状态: {'亮' if state else '灭'}", "DEBUG")
        return
    
    if led is not None:
        try:
            led.value(state)
        except Exception as e:
            log(f"LED控制失败: {e}", "WARNING")

def format_test_result(test_start_time, test_complete_time):
    """格式化测试结果消息"""
    if test_start_time and test_complete_time:
        duration = test_complete_time - test_start_time
        timestamp = time.localtime()
        time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
        mode_str = "仿真" if SIMULATION_MODE else "实际"
        return f"{mode_str}起飞降落测试完成于 {time_str}，总时长 {duration:.1f} 秒"
    else:
        mode_str = "仿真" if SIMULATION_MODE else "实际"
        return f"{mode_str}起飞降落测试完成"

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

def safe_pin_init(pin_number, mode, pull=None):
    """
    安全的GPIO引脚初始化
    
    参数:
        pin_number: 引脚编号
        mode: 引脚模式 (machine.Pin.IN, machine.Pin.OUT等)
        pull: 上拉/下拉设置
    
    返回:
        Pin对象或None（如果初始化失败）
    """
    if SIMULATION_MODE:
        log(f"仿真模式：跳过GPIO{pin_number}初始化", "DEBUG")
        return None
    
    try:
        if pull is not None:
            pin = machine.Pin(pin_number, mode, pull)
        else:
            pin = machine.Pin(pin_number, mode)
        log(f"GPIO{pin_number}初始化成功", "DEBUG")
        return pin
    except Exception as e:
        log(f"GPIO{pin_number}初始化失败: {e}", "WARNING")
        return None

def safe_i2c_init(scl_pin, sda_pin, freq=400000):
    """
    安全的I2C接口初始化
    
    参数:
        scl_pin: SCL引脚编号
        sda_pin: SDA引脚编号
        freq: I2C频率
    
    返回:
        I2C对象或None（如果初始化失败）
    """
    if SIMULATION_MODE:
        log(f"仿真模式：跳过I2C初始化 (SCL:{scl_pin}, SDA:{sda_pin})", "DEBUG")
        return None
    
    try:
        i2c = machine.I2C(0, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin), freq=freq)
        log(f"I2C接口初始化成功 (SCL:{scl_pin}, SDA:{sda_pin})", "DEBUG")
        return i2c
    except Exception as e:
        log(f"I2C接口初始化失败: {e}", "WARNING")
        return None

def validate_range(value, min_val, max_val, name="值"):
    """
    验证值是否在指定范围内
    
    参数:
        value: 要验证的值
        min_val: 最小值
        max_val: 最大值
        name: 值的名称（用于日志）
    
    返回:
        限制在范围内的值
    """
    if value < min_val:
        log(f"{name} {value} 小于最小值 {min_val}，已调整", "WARNING")
        return min_val
    elif value > max_val:
        log(f"{name} {value} 大于最大值 {max_val}，已调整", "WARNING")
        return max_val
    return value

def system_info():
    """获取系统信息"""
    info = {
        "仿真模式": SIMULATION_MODE,
        "调试模式": DEBUG_MODE,
        "可用内存": gc.mem_free() if hasattr(__builtins__, 'gc') else "未知",
        "时间戳": time.localtime()
    }
    
    if SIMULATION_MODE:
        info["运行环境"] = "仿真环境"
    else:
        info["运行环境"] = "ESP32-S3硬件"
    
    return info

def print_system_info():
    """打印系统信息"""
    info = system_info()
    log("=== 系统信息 ===")
    for key, value in info.items():
        log(f"{key}: {value}")
    log("===============")

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