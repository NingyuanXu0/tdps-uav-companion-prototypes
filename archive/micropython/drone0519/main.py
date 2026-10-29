"""
Main program entry point for the ACM core.
Initializes all system components and runs the main control loop.
Modified to skip Task 1 (takeoff) and start with Task 2.
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, DEBUG_MODE
from utils import log, display_led_pattern
from perception import PerceptionSystem
from communication import FCMInterface, WirelessCommunication
from task_manager import TaskManager, TaskState

def initialize_system():
    """Initialize all system components"""
    log("正在初始化系统组件")
    
    # Initialize interfaces
    fcm = FCMInterface()
    perception = PerceptionSystem()
    wireless = WirelessCommunication()
    
    # Initialize task manager
    task_manager = TaskManager(fcm, perception, wireless)
    
    # Perform system checks
    if not fcm.initialized:
        log("FCM 接口初始化失败", "ERROR")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def main():
    """Main program entry point"""
    log("ACM 核心启动中")
    
    # Configure watchdog timer for safety
    wdt = machine.WDT(timeout=15000)  # 15 second timeout
    
    # Initialize system components
    task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，中止", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # Initialize task manager
    if not task_manager.initialize():
        log("任务管理器初始化失败，中止", "ERROR")
        display_led_pattern("ERROR")
        return
    
    log("ACM 核心初始化完成，进入主循环")
    last_loop_time = time.ticks_ms()
    
    # Main control loop
    try:
        # Wait for operator confirmation to start
        display_led_pattern("WAITING_FOR_START")
        log("等待开始按钮按下...")
        
        # Use a GPIO pin for start button
        start_button = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
        while start_button.value() == 1:  # Active low button
            wdt.feed()
            time.sleep(0.1)
        
        log("开始按钮已按下，任务开始")
        display_led_pattern("MISSION_START", 2)
            
        while True:
            # Feed watchdog
            wdt.feed()
            
            # Run garbage collection periodically
            gc.collect()
            
            # Check for manual abort button
            if hasattr(task_manager, 'abort_button') and task_manager.abort_button.value() == 0:
                log("手动中止触发")
                task_manager.abort_mission()
                break
            
            # Check battery voltage
            battery_voltage = task_manager.get_battery_voltage()
            if battery_voltage < task_manager.LOW_BATTERY_THRESHOLD:
                log(f"电池电量低: {battery_voltage:.2f}V", "WARNING")
                if battery_voltage < task_manager.CRITICAL_BATTERY_THRESHOLD:
                    log("电池电量严重不足，紧急降落", "ERROR")
                    task_manager.emergency_landing()
                    break
            
            # Update task manager
            if not task_manager.update():
                log("任务管理器发出退出信号")
                break
            
            # Calculate loop time and sleep to maintain consistent frequency
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            if DEBUG_MODE:
                log(f"循环时间: {loop_time * 1000:.2f}ms")
            
            # Sleep to maintain loop frequency
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        log(f"主循环中出现未处理异常: {e}", "ERROR")
        display_led_pattern("ERROR")
        
        # Try to perform emergency landing if possible
        try:
            task_manager.emergency_landing()
        except:
            # Last resort - try to stop motors
            try:
                task_manager.fcm.emergency_stop()
            except:
                pass
    
    finally:
        log("ACM 核心关闭中")
        # Perform cleanup
        if task_manager.state != TaskState.ERROR:
            display_led_pattern("MISSION_COMPLETE", 5)

if __name__ == "__main__":
    main()