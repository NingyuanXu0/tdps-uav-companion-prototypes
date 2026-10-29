"""
ACM核心主程序入口点
初始化所有系统组件并运行主控制循环
简化版本：上电15秒后自动起飞

作者: Score99团队
日期: 2025年4月
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, DEBUG_MODE, AUTO_TAKEOFF_DELAY
from utils import log, display_led_pattern
from perception import PerceptionSystem
from communication import FCMInterface
from task_manager import TaskManager

def initialize_system():
    """初始化所有系统组件"""
    log("初始化系统组件")
    
    # 初始化接口
    fcm = FCMInterface()
    perception = PerceptionSystem()
    
    # 初始化任务管理器
    task_manager = TaskManager(fcm, perception)
    
    # 执行系统检查
    if not fcm.initialized:
        log("飞控接口初始化失败", "错误")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def main():
    """主程序入口 - 简化版本，自动起飞"""
    log("ACM核心启动")
    
    # 配置看门狗定时器
    wdt = machine.WDT(timeout=15000)  # 15秒超时
    
    # 初始化系统组件
    task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，中止", "错误")
        display_led_pattern("ERROR")
        return
    
    # 初始化任务管理器
    if not task_manager.initialize():
        log("任务管理器初始化失败，中止", "错误")
        display_led_pattern("ERROR")
        return
    
    log("ACM核心初始化完成")
    
    # 等待15秒后自动起飞
    log("等待{}秒后自动起飞...".format(AUTO_TAKEOFF_DELAY))
    display_led_pattern("WAITING")
    
    start_time = time.time()
    while time.time() - start_time < AUTO_TAKEOFF_DELAY:
        wdt.feed()
        time.sleep(0.5)
        # 闪烁LED表示等待中
        if int(time.time() - start_time) % 2 == 0:
            display_led_pattern("WAITING", 0.5)
    
    log("开始执行任务")
    task_manager.start_mission()
    
    last_loop_time = time.ticks_ms()
    
    # 主控制循环
    try:
        while True:
            # 喂狗
            wdt.feed()
            
            # 定期垃圾回收
            if time.ticks_ms() % 5000 < 50:  # 每5秒执行一次
                gc.collect()
            
            # 更新飞控心跳
            task_manager.fcm.update()
            
            # 检查电池电压
            battery_voltage = task_manager.get_battery_voltage()
            if battery_voltage < task_manager.LOW_BATTERY_THRESHOLD:
                log("电池电量低: {:.2f}V".format(battery_voltage), "警告")
                if battery_voltage < task_manager.CRITICAL_BATTERY_THRESHOLD:
                    log("电池电量严重不足，紧急降落", "错误")
                    task_manager.emergency_landing()
                    break
            
            # 更新任务管理器
            if not task_manager.update():
                log("任务管理器发出退出信号")
                break
            
            # 计算循环时间并休眠以保持一致的频率
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            if DEBUG_MODE and loop_time > MAIN_LOOP_DELAY * 2:
                log("循环时间: {:.2f}ms".format(loop_time * 1000))
            
            # 休眠以保持循环频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        log("主循环中出现未处理异常: " + str(e), "错误")
        display_led_pattern("ERROR")
        
        # 尝试紧急降落
        try:
            task_manager.emergency_landing()
        except:
            # 最后手段 - 尝试停止电机
            try:
                task_manager.fcm.emergency_stop()
            except:
                pass
    
    finally:
        log("ACM核心关闭")
        display_led_pattern("COMPLETE", 5)

if __name__ == "__main__":
    main()