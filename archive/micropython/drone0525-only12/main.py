"""
ACM核心主程序 - 起飞降落测试版本
仅测试：起飞至0.5米高度——停留3秒钟——降落

作者: Score99团队
日期: 2025年4月
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, AUTO_TAKEOFF_DELAY
from utils import log, display_led_pattern
from communication import FCMInterface
from task_manager import TaskManager

def initialize_system():
    """初始化系统组件"""
    log("初始化系统组件 - 测试模式")
    
    # 初始化飞控接口
    fcm = FCMInterface()
    
    # 初始化任务管理器
    task_manager = TaskManager(fcm)
    
    # 执行系统检查
    if not fcm.initialized:
        log("飞控接口初始化失败", "错误")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def main():
    """主程序入口 - 测试版本"""
    log("ACM核心启动 - 起飞降落测试")
    
    # 配置看门狗定时器
    wdt = machine.WDT(timeout=30000)  # 30秒超时
    
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
    
    log("系统初始化完成")
    
    # 等待指定时间后自动起飞
    log("等待{}秒后自动起飞...".format(AUTO_TAKEOFF_DELAY))
    display_led_pattern("WAITING")
    
    start_time = time.time()
    while time.time() - start_time < AUTO_TAKEOFF_DELAY:
        wdt.feed()
        time.sleep(0.5)
        # 每秒闪烁LED
        if int(time.time() - start_time) % 2 == 0:
            display_led_pattern("WAITING", 0.5)
    
    log("开始执行测试任务")
    task_manager.start_mission()
    
    last_loop_time = time.ticks_ms()
    
    # 主控制循环
    try:
        while True:
            # 喂狗
            wdt.feed()
            
            # 定期垃圾回收
            if time.ticks_ms() % 5000 < 50:
                gc.collect()
            
            # 更新任务管理器
            if not task_manager.update():
                log("测试任务完成")
                break
            
            # 计算循环时间
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            # 休眠以保持循环频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        log("主循环中出现异常: " + str(e), "错误")
        display_led_pattern("ERROR")
        
        # 尝试紧急停止
        try:
            task_manager.emergency_stop()
        except:
            pass
    
    finally:
        log("测试程序结束")
        display_led_pattern("COMPLETE", 3)

if __name__ == "__main__":
    main()