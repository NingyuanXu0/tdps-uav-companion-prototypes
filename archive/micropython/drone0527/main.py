"""
ACM核心主程序入口 - 起飞降落测试版本（修复版）
移除WiFi功能，修复循环阻塞问题
"""
import time
import gc
import machine
from config import (
    MAIN_LOOP_DELAY, DEBUG_MODE, USE_AUTO_START, 
    AUTO_START_DELAY
)
from utils import log, display_startup_pattern, display_error_pattern
from perception import PerceptionSystem
from communication import FCMInterface
from task_manager import TaskManager, TaskState

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
        log("FCM接口初始化失败", "ERROR")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def main():
    """
    主程序入口 - 起飞降落测试（修复版）
    """
    log("=== ACM核心启动 - 起飞降落测试模式 ===")
    
    # 显示启动模式
    display_startup_pattern()
    
    # 配置看门狗定时器以确保安全
    wdt = machine.WDT(timeout=30000)  # 30秒超时（增加超时时间）
    
    # 初始化系统组件
    task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，中止", "ERROR")
        display_error_pattern()
        return
    
    # 初始化任务管理器
    if not task_manager.initialize():
        log("任务管理器初始化失败，中止", "ERROR")
        display_error_pattern()
        return
    
    log("ACM核心初始化完成")
    last_loop_time = time.ticks_ms()
    loop_count = 0
    
    # 主控制循环
    try:
        # 启动逻辑
        if USE_AUTO_START:
            # 自动启动模式
            log(f"自动启动模式已启用，{AUTO_START_DELAY}秒后起飞")
            
            countdown_start = time.time()
            last_countdown_log = -1
            
            while time.time() - countdown_start < AUTO_START_DELAY:
                wdt.feed()
                
                # 检查中止按钮
                if task_manager.abort_button.value() == 0:
                    log("自动启动期间按下中止按钮")
                    return
                
                # 显示倒计时（每秒一次）
                remaining = AUTO_START_DELAY - (time.time() - countdown_start)
                current_second = int(remaining)
                if current_second != last_countdown_log and current_second >= 0:
                    log(f"倒计时: {current_second}秒")
                    last_countdown_log = current_second
                
                time.sleep(0.1)
            
            log("自动启动倒计时结束，开始测试")
            
        else:
            # 手动启动模式
            log("等待启动信号...")
            log("按下GPIO15按钮开始测试")
            
            # 使用GPIO引脚作为开始按钮
            start_button = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
            
            while True:
                wdt.feed()
                
                # 检查手动按钮
                if start_button.value() == 0:
                    log("手动启动按钮已按下")
                    break
                
                # 检查中止按钮
                if task_manager.abort_button.value() == 0:
                    log("在等待期间按下中止按钮")
                    return
                
                time.sleep(0.1)
        
        log("起飞降落测试开始")
        
        # 开始测试
        if not task_manager.start_test():
            log("无法开始测试", "ERROR")
            display_error_pattern()
            return
            
        while True:
            loop_count += 1
            
            # 喂狗
            wdt.feed()
            
            # 定期运行垃圾回收（每100次循环）
            if loop_count % 100 == 0:
                gc.collect()
            
            # 更新FCM心跳
            task_manager.fcm.update()
            
            # 检查手动中止按钮
            if task_manager.abort_button.value() == 0:
                log("手动中止触发")
                task_manager.abort_test()
                break
            
            # 更新任务管理器
            if not task_manager.update():
                log("任务管理器发出退出信号")
                break
            
            # 计算循环时间并休眠以保持一致的频率
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            # 只在DEBUG模式下且循环时间明显超标时显示警告
            if DEBUG_MODE and loop_time > MAIN_LOOP_DELAY * 2:
                log(f"循环时间: {loop_time * 1000:.2f}ms (警告：超过目标时间)")
            
            # 休眠以保持循环频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        log("收到键盘中断信号", "WARNING")
    except Exception as e:
        log(f"主循环中的未处理异常: {e}", "ERROR")
        display_error_pattern()
        
        # 尝试执行紧急降落
        try:
            log("尝试紧急降落...")
            task_manager.emergency_landing()
        except:
            # 最后手段 - 尝试停止电机
            try:
                log("尝试停止电机...")
                task_manager.fcm.emergency_stop()
            except:
                pass
    
    finally:
        log("ACM核心关闭")
        
        # 执行清理
        if hasattr(task_manager, 'state') and task_manager.state == TaskState.TEST_COMPLETE:
            log("起飞降落测试成功完成！")
        else:
            log("起飞降落测试异常结束")

if __name__ == "__main__":
    # 打印启动信息
    print("\n" + "="*50)
    print("Score99团队 - 无人机起飞降落测试程序")
    print("ESP32-S3 MicroPython ACM核心")
    print("版本: 1.3 (修复阻塞问题)")
    print("="*50 + "\n")
    
    # 短暂延迟以便查看启动信息
    time.sleep(2)
    
    main()