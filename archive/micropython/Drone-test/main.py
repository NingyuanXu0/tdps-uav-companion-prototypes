"""
无人机主程序
实现起飞到0.5米高度、悬停10秒、降落功能
"""
import time
import machine
import gc
from config import (
    TASK_EXECUTION_FREQUENCY, MAIN_LOOP_DELAY, DEBUG_MODE,
    TAKEOFF_HEIGHT, HOVERING_TIME,
    START_BUTTON_PIN, EMERGENCY_BUTTON_PIN
)
from utils import log, display_led_pattern
from height_sensor import HeightSensor
from communication import FCMInterface
from flight_controller import FlightController, FlightState

def main():
    """主程序入口"""
    log("简易无人机系统启动")
    
    # 初始化按钮
    start_button = machine.Pin(START_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    emergency_button = machine.Pin(EMERGENCY_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    
    # 初始化组件
    log("初始化系统组件")
    fcm = FCMInterface()
    height_sensor = HeightSensor()
    
    # 初始化飞行控制器
    flight_controller = FlightController(fcm, height_sensor)
    
    # 初始化用于调试的看门狗
    wdt = None
    if DEBUG_MODE:
        try:
            # 设置30秒超时的看门狗
            wdt = machine.WDT(timeout=30000)
            log("看门狗定时器已启用")
        except Exception as e:
            log(f"无法启用看门狗: {e}", "WARNING")
    
    # 显示就绪状态
    log("系统就绪，等待启动按钮按下...")
    display_led_pattern("READY")
    
    # 等待启动按钮按下
    flight_started = False
    last_loop_time = time.ticks_ms()
    
    try:
        while True:
            # 喂狗
            if wdt:
                wdt.feed()
                
            # 检查启动按钮
            if not flight_started and not start_button.value():
                # 按钮去抖动
                time.sleep(0.1)
                if not start_button.value():  # 再次检查
                    log("启动按钮按下，开始飞行任务")
                    flight_controller.start_flight()
                    flight_started = True
                    while not start_button.value():  # 等待按钮释放
                        time.sleep(0.05)
            
            # 检查紧急停止按钮
            if not emergency_button.value():
                # 按钮去抖动
                time.sleep(0.1)
                if not emergency_button.value():  # 再次检查
                    log("紧急停止按钮按下")
                    flight_controller.emergency_stop()
                    break  # 退出主循环
            
            # 垃圾回收
            gc.collect()
            
            # 更新飞行控制器
            if flight_started:
                mission_complete = flight_controller.update(TAKEOFF_HEIGHT, HOVERING_TIME)
                if mission_complete:
                    log("任务完成")
                    break  # 退出主循环
            
            # 计算循环时间并维持固定频率
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            if DEBUG_MODE and flight_started:
                if loop_time > 0.1:  # 只在循环时间异常时记录
                    log(f"循环时间: {loop_time * 1000:.1f}毫秒")
            
            # 休眠以维持目标频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        log("程序被用户中断")
    except Exception as e:
        log(f"发生未处理异常: {e}", "ERROR")
    finally:
        # 确保电机关闭
        if fcm.initialized:
            fcm.motors_off()
        log("程序结束")

if __name__ == "__main__":
    main()