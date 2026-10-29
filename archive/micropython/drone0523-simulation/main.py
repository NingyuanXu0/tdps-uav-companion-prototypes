"""
ACM核心主程序入口 - 起飞降落测试版本
初始化所有系统组件并运行测试控制循环
支持自动启动和手动控制
支持仿真模式运行
"""
import time
import gc
import machine
import network
import socket
from config import (
    MAIN_LOOP_DELAY, DEBUG_MODE, USE_AUTO_START, 
    AUTO_START_DELAY, USE_WIFI_CONTROL, WIFI_SSID, 
    WIFI_PASSWORD, WIFI_PORT, SIMULATION_MODE
)
from utils import log, display_led_pattern
from perception import PerceptionSystem
from communication import FCMInterface
from task_manager import TaskManager, TaskState

def initialize_wifi_control():
    """初始化WiFi控制接口"""
    if not USE_WIFI_CONTROL:
        return None
    
    if SIMULATION_MODE:
        log("仿真模式：跳过WiFi初始化")
        return None
        
    try:
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=WIFI_SSID, password=WIFI_PASSWORD)
        
        log(f"WiFi AP已创建: {WIFI_SSID}")
        log(f"IP地址: {ap.ifconfig()[0]}")
        
        # 创建socket服务器
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('', WIFI_PORT))
        s.listen(1)
        s.setblocking(False)
        
        return s
    except Exception as e:
        log(f"WiFi初始化失败: {e}", "ERROR")
        return None

def check_wifi_command(sock):
    """检查WiFi命令"""
    if sock is None:
        return None
        
    try:
        conn, addr = sock.accept()
        data = conn.recv(1024).decode()
        conn.close()
        
        if data.strip().upper() == "START":
            log(f"收到WiFi启动命令来自 {addr}")
            return "START"
        elif data.strip().upper() == "ABORT":
            log(f"收到WiFi中止命令来自 {addr}")
            return "ABORT"
    except:
        pass
    
    return None

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
    
    mode_str = "仿真模式" if SIMULATION_MODE else "硬件模式"
    log(f"系统组件初始化成功 ({mode_str})")
    return task_manager

def initialize_hardware_buttons():
    """初始化硬件按钮"""
    if SIMULATION_MODE:
        log("仿真模式：跳过硬件按钮初始化")
        return None, None
    
    try:
        start_button = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
        log("硬件按钮初始化成功")
        return start_button, None
    except Exception as e:
        log(f"硬件按钮初始化失败: {e}", "WARNING")
        return None, None

def initialize_watchdog():
    """初始化看门狗定时器"""
    if SIMULATION_MODE:
        log("仿真模式：跳过看门狗初始化")
        return None
    
    try:
        wdt = machine.WDT(timeout=15000)  # 15秒超时
        log("看门狗定时器初始化成功")
        return wdt
    except Exception as e:
        log(f"看门狗定时器初始化失败: {e}", "WARNING")
        return None

def check_start_conditions(start_button, wifi_sock, task_manager):
    """检查启动条件"""
    # 检查手动按钮（如果可用）
    if start_button is not None:
        try:
            if start_button.value() == 0:
                log("手动启动按钮已按下")
                return True
        except:
            pass
    
    # 检查WiFi命令
    wifi_cmd = check_wifi_command(wifi_sock)
    if wifi_cmd == "START":
        return True
    elif wifi_cmd == "ABORT":
        log("收到WiFi中止命令")
        return False
    
    # 检查中止按钮
    if task_manager.check_abort_signal():
        log("在等待期间按下中止按钮")
        return False
    
    # 仿真模式下的自动启动逻辑
    if SIMULATION_MODE and not USE_AUTO_START:
        # 在仿真模式下且未启用自动启动时，自动开始测试
        log("仿真模式：自动开始测试")
        return True
    
    return None  # 继续等待

def main():
    """
    主程序入口 - 起飞降落测试
    支持自动启动和手动/WiFi控制
    支持仿真模式运行
    """
    mode_str = "仿真模式" if SIMULATION_MODE else "硬件模式"
    log(f"=== ACM核心启动 - 起飞降落测试模式 ({mode_str}) ===")
    
    # 配置看门狗定时器以确保安全
    wdt = initialize_watchdog()
    
    # 初始化系统组件
    task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，中止", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # 初始化任务管理器
    if not task_manager.initialize():
        log("任务管理器初始化失败，中止", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # 初始化WiFi控制（如果启用）
    wifi_sock = initialize_wifi_control()
    
    # 初始化硬件按钮
    start_button, _ = initialize_hardware_buttons()
    
    log("ACM核心初始化完成")
    last_loop_time = time.ticks_ms()
    
    # 主控制循环
    try:
        # 启动逻辑
        if USE_AUTO_START:
            # 自动启动模式
            log(f"自动启动模式已启用，{AUTO_START_DELAY}秒后起飞")
            display_led_pattern("AUTO_START_COUNTDOWN")
            
            countdown_start = time.time()
            while time.time() - countdown_start < AUTO_START_DELAY:
                if wdt:
                    wdt.feed()
                
                # 检查中止信号
                if task_manager.check_abort_signal():
                    log("自动启动期间收到中止信号")
                    return
                
                # 显示倒计时
                remaining = AUTO_START_DELAY - (time.time() - countdown_start)
                if int(remaining) != int(remaining + 0.1):  # 每秒显示一次
                    log(f"倒计时: {int(remaining)}秒")
                
                time.sleep(0.1)
            
            log("自动启动倒计时结束，开始测试")
            
        else:
            # 手动启动模式
            display_led_pattern("WAITING_FOR_START")
            log("等待启动信号...")
            
            if USE_WIFI_CONTROL and not SIMULATION_MODE:
                log("通过WiFi发送'START'命令启动")
            elif not SIMULATION_MODE:
                log("按下GPIO15按钮开始测试")
            else:
                log("仿真模式：等待启动条件")
            
            while True:
                if wdt:
                    wdt.feed()
                
                # 检查启动条件
                start_result = check_start_conditions(start_button, wifi_sock, task_manager)
                if start_result is True:
                    break
                elif start_result is False:
                    return
                
                time.sleep(0.1)
        
        log("起飞降落测试开始")
        
        # 开始测试
        if not task_manager.start_test():
            log("无法开始测试", "ERROR")
            display_led_pattern("ERROR")
            return
            
        while True:
            # 喂狗
            if wdt:
                wdt.feed()
            
            # 定期运行垃圾回收
            gc.collect()
            
            # 更新FCM心跳
            task_manager.fcm.update()
            
            # 检查手动中止信号
            if task_manager.check_abort_signal():
                log("手动中止触发")
                task_manager.abort_test()
                break
            
            # 检查WiFi中止命令
            if check_wifi_command(wifi_sock) == "ABORT":
                log("WiFi中止触发")
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
            
            if DEBUG_MODE and loop_time > MAIN_LOOP_DELAY * 1.5:
                log(f"循环时间: {loop_time * 1000:.2f}ms (警告：超过目标时间)")
            
            # 休眠以保持循环频率
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except Exception as e:
        log(f"主循环中的未处理异常: {e}", "ERROR")
        display_led_pattern("ERROR")
        
        # 尝试执行紧急降落
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
        # 关闭WiFi socket
        if wifi_sock:
            try:
                wifi_sock.close()
            except:
                pass
        
        # 执行清理
        if hasattr(task_manager, 'state') and task_manager.state == TaskState.TEST_COMPLETE:
            display_led_pattern("TEST_COMPLETE", 5)
            log("起飞降落测试成功完成！")
        else:
            log("起飞降落测试异常结束")

if __name__ == "__main__":
    # 打印启动信息
    print("\n" + "="*50)
    print("Score99团队 - 无人机起飞降落测试程序")
    print("ESP32-S3 MicroPython ACM核心")
    if SIMULATION_MODE:
        print("版本: 1.2 (仿真模式)")
    else:
        print("版本: 1.2 (硬件模式)")
    print("="*50 + "\n")
    
    # 短暂延迟以便查看启动信息
    time.sleep(2)
    
    main()