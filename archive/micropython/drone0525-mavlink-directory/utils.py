"""
工具函数模块 - MAVLink增强版本
日志记录、LED控制、系统诊断和调试功能

作者: Score99团队
日期: 2025年4月
"""
import time
import machine
import gc
from config import (
    LED_PATTERNS, DEBUG_MODE, MAVLINK_DEBUG, 
    SENSOR_DEBUG, PERFORMANCE_DEBUG,
    DIAGNOSTICS_INTERVAL, PERFORMANCE_LOG_INTERVAL
)

# 初始化内置LED
led = machine.Pin(2, machine.Pin.OUT)

# 日志级别定义
LOG_LEVEL_DEBUG = 0
LOG_LEVEL_INFO = 1
LOG_LEVEL_WARNING = 2
LOG_LEVEL_ERROR = 3
LOG_LEVEL_CRITICAL = 4

# 当前日志级别
current_log_level = LOG_LEVEL_INFO if not DEBUG_MODE else LOG_LEVEL_DEBUG

# 日志级别颜色（串口终端支持）
LOG_COLORS = {
    "调试": "\033[36m",    # 青色
    "信息": "",            # 默认
    "警告": "\033[93m",    # 黄色
    "错误": "\033[91m",    # 红色
    "严重": "\033[95m",    # 紫色
    "成功": "\033[92m",    # 绿色
}
COLOR_RESET = "\033[0m"

# 性能监控变量
performance_stats = {
    "total_logs": 0,
    "error_count": 0,
    "warning_count": 0,
    "last_diagnostics": 0,
    "last_performance_log": 0,
    "loop_times": [],
    "memory_usage": []
}

def set_log_level(level):
    """设置日志级别"""
    global current_log_level
    current_log_level = level

def log(message, level="信息", subsystem=None):
    """
    增强的日志记录功能
    
    参数:
        message: 日志消息
        level: 日志级别（调试/信息/警告/错误/严重/成功）
        subsystem: 子系统标识（MAVLink/传感器/性能等）
    """
    # 级别映射
    level_map = {
        "调试": LOG_LEVEL_DEBUG,
        "信息": LOG_LEVEL_INFO,
        "警告": LOG_LEVEL_WARNING,
        "错误": LOG_LEVEL_ERROR,
        "严重": LOG_LEVEL_CRITICAL,
        "成功": LOG_LEVEL_INFO
    }
    
    # 检查日志级别
    if level_map.get(level, LOG_LEVEL_INFO) < current_log_level:
        return
    
    # 子系统过滤
    if subsystem == "MAVLink" and not MAVLINK_DEBUG:
        return
    if subsystem == "传感器" and not SENSOR_DEBUG:
        return
    if subsystem == "性能" and not PERFORMANCE_DEBUG:
        return
    
    # 更新统计
    performance_stats["total_logs"] += 1
    if level == "错误" or level == "严重":
        performance_stats["error_count"] += 1
    elif level == "警告":
        performance_stats["warning_count"] += 1
    
    # 构建时间戳
    timestamp = time.localtime()
    time_str = "{:02d}:{:02d}:{:02d}".format(
        timestamp[3], timestamp[4], timestamp[5]
    )
    
    # 构建子系统标签
    subsystem_tag = "[{}]".format(subsystem) if subsystem else ""
    
    # 添加颜色（如果支持）
    color = LOG_COLORS.get(level, "")
    if color:
        log_message = "{}[{}] {}{}: {}{}".format(
            color, time_str, subsystem_tag, level, message, COLOR_RESET
        )
    else:
        log_message = "[{}] {}{}: {}".format(
            time_str, subsystem_tag, level, message
        )
    
    print(log_message)
    
    # 严重错误时闪烁LED
    if level == "严重":
        display_led_pattern("ERROR", 1)

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

def format_hex(data, prefix=""):
    """格式化字节数据为十六进制字符串（调试用）"""
    if not data:
        return prefix + "无数据"
    
    hex_str = ' '.join(['{:02X}'.format(b) for b in data])
    return prefix + hex_str

def format_mavlink_message(message):
    """格式化MAVLink消息用于调试"""
    if not MAVLINK_DEBUG or not message:
        return None
    
    if len(message) < 6:
        return "无效消息"
    
    # 解析MAVLink v1消息头
    start_byte = message[0]
    payload_len = message[1]
    sequence = message[2]
    sys_id = message[3]
    comp_id = message[4]
    msg_id = message[5]
    
    return "MAVLink: Start=0x{:02X} Len={} Seq={} Sys={} Comp={} Msg={}".format(
        start_byte, payload_len, sequence, sys_id, comp_id, msg_id
    )

def check_connection_status(fcm_interface=None):
    """检查连接状态"""
    status = {
        "uart_connected": False,
        "mavlink_active": False,
        "offboard_enabled": False,
        "armed": False
    }
    
    if fcm_interface:
        status["uart_connected"] = fcm_interface.initialized
        status["mavlink_active"] = fcm_interface.connection_established
        status["offboard_enabled"] = fcm_interface.offboard_enabled
        status["armed"] = fcm_interface.armed
    
    return status

def system_info():
    """打印系统信息"""
    import os
    
    log("="*50)
    log("系统信息:")
    log("- 平台: ESP32-S3")
    
    # MicroPython版本
    try:
        import sys
        log("- MicroPython版本: " + sys.version)
    except:
        log("- MicroPython版本: 未知")
    
    # 内存信息
    gc.collect()
    free_mem = gc.mem_free()
    alloc_mem = gc.mem_alloc()
    total_mem = free_mem + alloc_mem
    
    log("- 总内存: {} KB".format(total_mem // 1024))
    log("- 已用内存: {} KB ({:.1f}%)".format(
        alloc_mem // 1024, (alloc_mem / total_mem) * 100))
    log("- 可用内存: {} KB ({:.1f}%)".format(
        free_mem // 1024, (free_mem / total_mem) * 100))
    
    # CPU频率
    log("- 主频: {} MHz".format(machine.freq() // 1000000))
    
    # Flash信息
    try:
        import esp
        log("- Flash大小: {} MB".format(esp.flash_size() // (1024 * 1024)))
    except:
        pass
    
    log("="*50)

def memory_monitor():
    """监控内存使用情况"""
    gc.collect()
    free_mem = gc.mem_free()
    alloc_mem = gc.mem_alloc()
    
    # 更新统计
    performance_stats["memory_usage"].append(alloc_mem)
    if len(performance_stats["memory_usage"]) > 100:
        performance_stats["memory_usage"].pop(0)
    
    # 检查内存压力
    if free_mem < 20000:  # 小于20KB
        log("内存不足警告: 可用 {} KB".format(free_mem // 1024), "警告", "性能")
        gc.collect()
        free_mem = gc.mem_free()
        log("垃圾回收后: 可用 {} KB".format(free_mem // 1024), "信息", "性能")
    
    return free_mem, alloc_mem

def performance_monitor(loop_time):
    """监控性能指标"""
    current_time = time.time()
    
    # 记录循环时间
    performance_stats["loop_times"].append(loop_time)
    if len(performance_stats["loop_times"]) > 100:
        performance_stats["loop_times"].pop(0)
    
    # 定期输出性能日志
    if current_time - performance_stats["last_performance_log"] > PERFORMANCE_LOG_INTERVAL:
        if PERFORMANCE_DEBUG and performance_stats["loop_times"]:
            avg_loop_time = sum(performance_stats["loop_times"]) / len(performance_stats["loop_times"])
            max_loop_time = max(performance_stats["loop_times"])
            
            log("性能统计:", "信息", "性能")
            log("- 平均循环时间: {:.3f}秒".format(avg_loop_time), "信息", "性能")
            log("- 最大循环时间: {:.3f}秒".format(max_loop_time), "信息", "性能")
            
            if performance_stats["memory_usage"]:
                avg_memory = sum(performance_stats["memory_usage"]) / len(performance_stats["memory_usage"])
                log("- 平均内存使用: {} KB".format(int(avg_memory) // 1024), "信息", "性能")
        
        performance_stats["last_performance_log"] = current_time

def system_diagnostics(fcm_interface=None, perception_system=None):
    """系统诊断功能"""
    current_time = time.time()
    
    # 检查是否到诊断时间
    if current_time - performance_stats["last_diagnostics"] < DIAGNOSTICS_INTERVAL:
        return
    
    performance_stats["last_diagnostics"] = current_time
    
    log("="*40, "信息", "诊断")
    log("系统诊断报告", "信息", "诊断")
    
    # 连接状态
    if fcm_interface:
        conn_status = check_connection_status(fcm_interface)
        log("飞控连接:", "信息", "诊断")
        for key, value in conn_status.items():
            status_str = "是" if value else "否"
            log("  - {}: {}".format(key, status_str), "信息", "诊断")
    
    # 传感器状态
    if perception_system:
        sensor_status = perception_system.get_sensor_status()
        log("传感器状态:", "信息", "诊断")
        log("  - 初始化: {}".format("是" if sensor_status["initialized"] else "否"), "信息", "诊断")
        log("  - 当前高度: {:.3f}米".format(
            sensor_status["current_height"] if sensor_status["current_height"] else 0), "信息", "诊断")
        log("  - 垂直速度: {:.3f}米/秒".format(
            sensor_status["current_velocity_z"]), "信息", "诊断")
    
    # 内存状态
    free_mem, alloc_mem = memory_monitor()
    log("内存状态:", "信息", "诊断")
    log("  - 已用: {} KB".format(alloc_mem // 1024), "信息", "诊断")
    log("  - 可用: {} KB".format(free_mem // 1024), "信息", "诊断")
    
    # 错误统计
    log("错误统计:", "信息", "诊断")
    log("  - 总日志: {}".format(performance_stats["total_logs"]), "信息", "诊断")
    log("  - 错误数: {}".format(performance_stats["error_count"]), "信息", "诊断")
    log("  - 警告数: {}".format(performance_stats["warning_count"]), "信息", "诊断")
    
    log("="*40, "信息", "诊断")

def emergency_log_dump():
    """紧急情况下输出所有关键信息"""
    log("!!!! 紧急日志转储 !!!!", "严重")
    
    # 输出所有性能统计
    log("性能统计:", "严重")
    for key, value in performance_stats.items():
        if isinstance(value, list):
            log("  - {}: {} 条记录".format(key, len(value)), "严重")
        else:
            log("  - {}: {}".format(key, value), "严重")
    
    # 内存状态
    free_mem, alloc_mem = memory_monitor()
    log("内存状态: 已用={} KB, 可用={} KB".format(
        alloc_mem // 1024, free_mem // 1024), "严重")
    
    # 调用栈信息
    try:
        import sys
        log("调用栈信息可用: {}".format(hasattr(sys, 'print_exception')), "严重")
    except:
        pass

def test_hardware_indicators():
    """测试硬件指示器（LED和蜂鸣器）"""
    log("测试硬件指示器...", "信息")
    
    # 测试所有LED模式
    patterns_to_test = ["STARTUP", "CONNECTING", "ARMED", "ERROR", "COMPLETE"]
    
    for pattern in patterns_to_test:
        log("测试LED模式: {}".format(pattern), "信息")
        display_led_pattern(pattern, 1)
        time.sleep(0.5)
    
    log("硬件指示器测试完成", "信息")

def create_diagnostics_report():
    """创建诊断报告字符串"""
    report = []
    report.append("=== 系统诊断报告 ===")
    report.append("时间: {}".format(time.time()))
    
    # 内存信息
    gc.collect()
    report.append("内存: 已用={} KB, 可用={} KB".format(
        gc.mem_alloc() // 1024, gc.mem_free() // 1024))
    
    # 错误统计
    report.append("错误: {}, 警告: {}".format(
        performance_stats["error_count"], 
        performance_stats["warning_count"]))
    
    # 性能信息
    if performance_stats["loop_times"]:
        avg_loop = sum(performance_stats["loop_times"]) / len(performance_stats["loop_times"])
        report.append("平均循环时间: {:.3f}秒".format(avg_loop))
    
    return "\n".join(report)