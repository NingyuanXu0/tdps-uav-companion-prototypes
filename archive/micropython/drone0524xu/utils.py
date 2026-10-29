"""
无人机系统工具函数 - 起飞降落测试版本
包含日志记录、LED控制和各种辅助函数
"""
import time
import machine
import uos
import gc
from config import (
    LED_PATTERNS, DEBUG_MODE, LED_PIN, 
    LOG_FILENAME, LOG_MAX_SIZE, LOG_LEVELS
)

# 初始化内置LED
try:
    led = machine.Pin(LED_PIN, machine.Pin.OUT)
except:
    led = None
    print("警告：无法初始化LED引脚")

# 全局变量
_log_file_size = 0
_led_pattern_active = False
_current_led_pattern = None

def initialize_logging():
    """初始化日志系统"""
    global _log_file_size
    
    try:
        # 检查日志文件是否存在
        try:
            stat = uos.stat(LOG_FILENAME)
            _log_file_size = stat[6]  # 文件大小
            
            # 如果文件过大，创建备份
            if _log_file_size > LOG_MAX_SIZE:
                backup_name = LOG_FILENAME.replace(".txt", "_backup.txt")
                try:
                    uos.remove(backup_name)
                except:
                    pass
                uos.rename(LOG_FILENAME, backup_name)
                _log_file_size = 0
                log("日志文件已备份并重新创建", "INFO")
                
        except OSError:
            # 文件不存在，创建新文件
            with open(LOG_FILENAME, "w") as f:
                f.write(f"=== 无人机飞行日志 - {format_time()} ===\n")
                f.write("="*50 + "\n")
            _log_file_size = 100
            
    except Exception as e:
        print(f"日志系统初始化失败: {e}")

# 初始化日志系统
initialize_logging()

def current_time_ms():
    """返回当前时间（毫秒）"""
    return time.ticks_ms()

def time_diff_ms(start_time):
    """返回时间差（毫秒）"""
    return time.ticks_diff(time.ticks_ms(), start_time)

def format_time():
    """格式化当前时间为字符串"""
    try:
        timestamp = time.localtime()
        return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5]
        )
    except:
        # 如果RTC未设置，使用运行时间
        runtime = time.ticks_ms() // 1000
        hours = runtime // 3600
        minutes = (runtime % 3600) // 60
        seconds = runtime % 60
        return f"运行时间 {hours:02d}:{minutes:02d}:{seconds:02d}"

def log(message, level="INFO"):
    """
    记录带级别和时间戳的消息
    
    参数:
        message: 日志消息
        level: 日志级别 (INFO, WARNING, ERROR, DEBUG)
    """
    global _log_file_size
    
    # 检查是否需要输出
    if level == "DEBUG" and not DEBUG_MODE:
        return
    
    if level not in LOG_LEVELS:
        level = "INFO"
    
    # 格式化日志消息
    time_str = format_time()
    log_message = f"[{time_str}] {level:>7}: {message}"
    
    # 控制台输出
    if level == "ERROR":
        print(f"\033[91m{log_message}\033[0m")  # 红色
    elif level == "WARNING":
        print(f"\033[93m{log_message}\033[0m")  # 黄色
    elif level == "DEBUG":
        print(f"\033[90m{log_message}\033[0m")  # 灰色
    else:
        print(log_message)
    
    # 写入日志文件
    try:
        # 检查文件大小
        if _log_file_size > LOG_MAX_SIZE:
            initialize_logging()  # 重新初始化
        
        with open(LOG_FILENAME, "a") as f:
            f.write(log_message + "\n")
            _log_file_size += len(log_message) + 1
            
    except Exception as e:
        if DEBUG_MODE:
            print(f"日志写入失败: {e}")

def display_led_pattern(pattern_name, duration=None):
    """
    显示LED闪烁模式
    
    参数:
        pattern_name: LED_PATTERNS中定义的模式名称
        duration: 显示持续时间（秒），None表示持续显示
    """
    global _led_pattern_active, _current_led_pattern
    
    if not led:
        return
    
    if pattern_name not in LED_PATTERNS:
        log(f"未知的LED模式: {pattern_name}", "WARNING")
        return
    
    # 如果是相同的模式且正在运行，不重复启动
    if _current_led_pattern == pattern_name and _led_pattern_active:
        return
    
    _current_led_pattern = pattern_name
    pattern = LED_PATTERNS[pattern_name]
    
    # 简单实现：只执行一次模式
    try:
        for state, delay in pattern:
            led.value(state)
            time.sleep(delay)
        
        # 如果有持续时间，保持最后状态
        if duration:
            time.sleep(duration)
            led.value(0)  # 关闭LED
    except Exception as e:
        log(f"LED模式显示错误: {e}", "ERROR")

def print_system_info():
    """打印系统信息"""
    print("\n" + "="*60)
    print("系统信息:")
    print(f"  MicroPython版本: {uos.uname().version}")
    print(f"  硬件平台: {uos.uname().machine}")
    print(f"  总内存: {gc.mem_alloc() + gc.mem_free()} 字节")
    print(f"  已用内存: {gc.mem_alloc()} 字节")
    print(f"  空闲内存: {gc.mem_free()} 字节")
    
    try:
        # 获取Flash信息
        stat = uos.statvfs('/')
        total = stat[0] * stat[2]
        free = stat[0] * stat[3]
        used = total - free
        print(f"  Flash总容量: {total//1024}KB")
        print(f"  Flash已用: {used//1024}KB")
        print(f"  Flash剩余: {free//1024}KB")
    except:
        pass
    
    print("="*60 + "\n")

def apply_moving_average(values, new_value, window_size=5):
    """
    应用移动平均滤波器
    
    参数:
        values: 历史值列表
        new_value: 新值
        window_size: 窗口大小
    
    返回:
        (更新后的列表, 滤波后的值)
    """
    # 添加新值
    values.append(new_value)
    
    # 保持窗口大小
    if len(values) > window_size:
        values.pop(0)
    
    # 计算平均值
    filtered_value = sum(values) / len(values)
    
    return values, filtered_value

def clamp(value, min_val, max_val):
    """
    限制值在指定范围内
    
    参数:
        value: 输入值
        min_val: 最小值
        max_val: 最大值
    
    返回:
        限制后的值
    """
    return max(min_val, min(max_val, value))

def calculate_distance_3d(pos1, pos2):
    """
    计算两个3D点之间的欧几里得距离
    
    参数:
        pos1: 第一个点 [x, y, z]
        pos2: 第二个点 [x, y, z]
    
    返回:
        距离（米）
    """
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    dz = pos2[2] - pos1[2]
    return (dx**2 + dy**2 + dz**2)**0.5

def format_test_result(test_start_time, test_complete_time, success=True):
    """
    格式化测试结果消息
    
    参数:
        test_start_time: 测试开始时间
        test_complete_time: 测试完成时间
        success: 是否成功
    
    返回:
        格式化的结果字符串
    """
    if test_start_time and test_complete_time:
        duration = test_complete_time - test_start_time
        status = "成功" if success else "失败"
        
        return (f"起飞降落测试{status} - "
                f"完成时间: {format_time()}, "
                f"总时长: {duration:.1f}秒")
    else:
        return f"起飞降落测试{'成功' if success else '失败'}"

def check_memory():
    """检查并报告内存使用情况"""
    gc.collect()
    free = gc.mem_free()
    used = gc.mem_alloc()
    total = free + used
    usage_percent = (used / total) * 100
    
    if DEBUG_MODE:
        log(f"内存使用: {used}/{total} 字节 ({usage_percent:.1f}%)", "DEBUG")
    
    # 如果内存使用超过80%，发出警告
    if usage_percent > 80:
        log(f"内存使用率高: {usage_percent:.1f}%", "WARNING")
    
    return free, used, total

def safe_sleep(duration):
    """
    安全的睡眠函数，支持中断
    
    参数:
        duration: 睡眠时间（秒）
    """
    start_time = time.time()
    while time.time() - start_time < duration:
        time.sleep(0.1)  # 100ms间隔，允许中断

def create_error_report(error_code, error_message, context=None):
    """
    创建错误报告
    
    参数:
        error_code: 错误代码
        error_message: 错误消息
        context: 错误上下文（可选）
    
    返回:
        错误报告字典
    """
    report = {
        "时间": format_time(),
        "错误代码": error_code,
        "错误消息": error_message,
        "运行时间": time.ticks_ms() // 1000
    }
    
    if context:
        report["上下文"] = context
    
    # 添加系统状态
    free, used, total = check_memory()
    report["内存状态"] = {
        "空闲": free,
        "已用": used,
        "总计": total
    }
    
    return report

def blink_led(times=3, on_time=0.1, off_time=0.1):
    """
    LED快速闪烁
    
    参数:
        times: 闪烁次数
        on_time: 亮起时间（秒）
        off_time: 熄灭时间（秒）
    """
    if not led:
        return
        
    for _ in range(times):
        led.value(1)
        time.sleep(on_time)
        led.value(0)
        time.sleep(off_time)

# 测试函数
def run_self_test():
    """运行自检程序"""
    log("=== 系统自检开始 ===")
    
    # 测试LED
    log("测试LED...")
    blink_led(3)
    
    # 测试内存
    log("测试内存...")
    free, used, total = check_memory()
    log(f"内存测试通过 - 空闲: {free}字节")
    
    # 测试日志系统
    log("测试日志系统...")
    log("测试INFO级别", "INFO")
    log("测试WARNING级别", "WARNING")
    log("测试ERROR级别", "ERROR")
    log("测试DEBUG级别", "DEBUG")
    
    log("=== 系统自检完成 ===")
    return True

"""
# 注释掉的扩展功能（完整任务时使用）
def calculate_velocity_vector(current_pos, target_pos, max_speed):
    # 计算朝向目标的速度向量
    pass

def interpolate_path(waypoints, resolution=0.1):
    # 路径插值
    pass

def check_collision_risk(current_pos, obstacle_pos, safety_margin):
    # 碰撞风险检查
    pass
"""