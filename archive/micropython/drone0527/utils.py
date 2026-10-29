"""
无人机系统工具函数 - 起飞降落测试版本（修复版）
修复LED阻塞问题，改为非阻塞控制
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

def set_led(state):
    """
    直接设置LED状态（非阻塞）
    
    参数:
        state: 0=关闭, 1=开启
    """
    led.value(state)

def blink_led_once(on_time=0.1, off_time=0.1):
    """
    LED闪烁一次（阻塞版本，仅在初始化时使用）
    
    参数:
        on_time: LED亮起时间（秒）
        off_time: LED熄灭时间（秒）
    """
    led.value(1)
    time.sleep(on_time)
    led.value(0)
    time.sleep(off_time)

def display_startup_pattern():
    """显示启动模式（快速闪烁3次）"""
    for _ in range(3):
        blink_led_once(0.1, 0.1)

def display_error_pattern():
    """显示错误模式（快速闪烁10次）"""
    for _ in range(10):
        blink_led_once(0.1, 0.1)

def display_test_complete_pattern():
    """显示测试完成模式（持续亮起5秒）"""
    led.value(1)
    time.sleep(5)
    led.value(0)

# 保留原有的display_led_pattern函数以保持兼容性，但标记为已弃用
def display_led_pattern(pattern_name, duration=None):
    """
    显示LED闪烁模式 - 简化非阻塞版本
    注意：此函数已简化，避免阻塞主循环
    
    参数:
        pattern_name: 模式名称
        duration: 持续时间（忽略，保持兼容性）
    """
    # 简化实现，只在特定情况下使用阻塞模式
    if pattern_name == "STARTUP":
        display_startup_pattern()
    elif pattern_name == "ERROR":
        display_error_pattern()
    elif pattern_name == "TEST_COMPLETE":
        if duration:
            led.value(1)
            time.sleep(duration)
            led.value(0)
        else:
            display_test_complete_pattern()
    else:
        # 对于其他模式，只设置LED状态，不阻塞
        if pattern_name in ["TAKEOFF", "HOVERING", "LANDING"]:
            led.value(1)  # 简单点亮
        else:
            led.value(0)  # 关闭

class NonBlockingLED:
    """
    非阻塞LED控制器
    可以在主循环中使用而不会造成阻塞
    """
    
    def __init__(self, pin=2):
        self.led = machine.Pin(pin, machine.Pin.OUT)
        self.pattern = None
        self.pattern_index = 0
        self.last_update = 0
        self.current_delay = 0
        self.pattern_repeat = False
    
    def set_pattern(self, pattern_name, repeat=True):
        """
        设置LED模式
        
        参数:
            pattern_name: LED_PATTERNS中的模式名称
            repeat: 是否重复模式
        """
        if pattern_name in LED_PATTERNS:
            self.pattern = LED_PATTERNS[pattern_name]
            self.pattern_index = 0
            self.last_update = time.ticks_ms()
            self.pattern_repeat = repeat
            
            # 立即设置第一个状态
            if self.pattern:
                state, delay = self.pattern[0]
                self.led.value(state)
                self.current_delay = delay * 1000  # 转换为毫秒
    
    def update(self):
        """
        更新LED状态（需要在主循环中定期调用）
        """
        if not self.pattern:
            return
        
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_update) >= self.current_delay:
            # 移动到下一个状态
            self.pattern_index += 1
            
            if self.pattern_index >= len(self.pattern):
                if self.pattern_repeat:
                    self.pattern_index = 0
                else:
                    self.pattern = None
                    self.led.value(0)
                    return
            
            # 设置新状态
            if self.pattern_index < len(self.pattern):
                state, delay = self.pattern[self.pattern_index]
                self.led.value(state)
                self.current_delay = delay * 1000  # 转换为毫秒
                self.last_update = current_time
    
    def stop(self):
        """停止当前模式并关闭LED"""
        self.pattern = None
        self.led.value(0)
    
    def set_solid(self, state):
        """设置LED为常亮或常灭"""
        self.pattern = None
        self.led.value(state)

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

def check_system_resources():
    """检查系统资源使用情况"""
    try:
        import gc
        gc.collect()
        free_mem = gc.mem_free()
        allocated_mem = gc.mem_alloc()
        total_mem = free_mem + allocated_mem
        
        log(f"内存使用: {allocated_mem} bytes, 可用: {free_mem} bytes, 总计: {total_mem} bytes")
        
        if free_mem < 10000:  # 少于10KB可用内存时警告
            log("内存不足警告", "WARNING")
            return False
        return True
    except:
        return True  # 如果检查失败，假设正常