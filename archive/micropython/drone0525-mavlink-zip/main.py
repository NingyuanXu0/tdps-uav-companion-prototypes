"""
ACM核心主程序 - MAVLink版本
适配Pixhawk4飞控的起飞降落测试

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

def check_hardware():
    """硬件检查"""
    log("执行硬件自检...")
    
    # 检查可用内存
    free_mem = gc.mem_free()
    log("可用内存: {} 字节".format(free_mem))
    
    # 检查UART引脚
    try:
        tx_pin = machine.Pin(17, machine.Pin.OUT)
        rx_pin = machine.Pin(16, machine.Pin.IN)
        log("UART引脚检查通过")
    except Exception as e:
        log("UART引脚检查失败: " + str(e), "错误")
        return False
    
    return True

def initialize_system():
    """初始化系统组件"""
    log("初始化系统组件 - MAVLink模式")
    
    # 硬件检查
    if not check_hardware():
        return None
    
    # 初始化飞控接口
    log("正在连接Pixhawk4飞控...")
    display_led_pattern("CONNECTING", 2)
    
    fcm = FCMInterface()
    
    # 等待飞控响应
    log("等待飞控响应...")
    time.sleep(2)
    
    # 发送几次心跳确保连接
    for i in range(5):
        fcm.update()
        time.sleep(0.2)
    
    # 初始化任务管理器
    task_manager = TaskManager(fcm)
    
    # 执行系统检查
    if not fcm.initialized:
        log("飞控接口初始化失败", "错误")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def main():
    """主程序入口 - MAVLink版本"""
    log("="*50)
    log("ACM核心启动 - Pixhawk4 MAVLink测试")
    log("测试任务: 起飞1米 → 悬停5秒 → 降落")
    log("="*50)
    
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
    log("等待{}秒后自动开始任务...".format(AUTO_TAKEOFF_DELAY))
    log("请确保:")
    log("- 飞机放置在平坦地面")
    log("- 周围有足够空间")
    log("- 人员保持安全距离")
    
    display_led_pattern("WAITING")
    
    start_time = time.time()
    countdown = AUTO_TAKEOFF_DELAY
    while time.time() - start_time < AUTO_TAKEOFF_DELAY:
        wdt.feed()
        
        # 倒计时显示
        new_countdown = int(AUTO_TAKEOFF_DELAY - (time.time() - start_time))
        if new_countdown != countdown:
            countdown = new_countdown
            log("倒计时: {} 秒".format(countdown))
            display_led_pattern("WAITING", 0.5)
        
        time.sleep(0.1)
    
    log("开始执行测试任务")
    task_manager.start_mission()
    
    last_loop_time = time.ticks_ms()
    loop_count = 0
    
    # 主控制循环
    try:
        while True:
            # 喂狗
            wdt.feed()
            
            # 定期垃圾回收
            if loop_count % 100 == 0:
                gc.collect()
            
            # 更新飞控心跳
            task_manager.fcm.update()
            
            # 更新任务管理器
            if not task_manager.update():
                log("测试任务完成")
                break
            
            # 计算循环时间
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            # 调试信息
            if loop_count % 50 == 0 and loop_count > 0:
                log("循环运行中... [{}]".format(loop_count))
            
            loop_count += 1
            
            # 休眠以保持循环频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        log("用户中断", "警告")
        task_manager.emergency_stop()
        
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
        
        # 确保电机锁定
        try:
            task_manager.fcm.motors_off()
        except:
            pass

if __name__ == "__main__":
    main()