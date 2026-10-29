"""
ACM核心主程序入口 - 起飞降落测试版本
初始化所有系统组件并运行测试控制循环
支持自动启动和手动控制
"""
import time
import gc
import machine
import network
import socket
import sys
from config import (
    MAIN_LOOP_DELAY, DEBUG_MODE, USE_AUTO_START, 
    AUTO_START_DELAY, USE_WIFI_CONTROL, WIFI_SSID, 
    WIFI_PASSWORD, WIFI_PORT, WIFI_TIMEOUT,
    START_BUTTON_PIN, ABORT_BUTTON_PIN
)
from utils import log, display_led_pattern, print_system_info
from perception import PerceptionSystem
from communication import FCMInterface
from task_manager import TaskManager, TaskState

def initialize_wifi_control():
    """初始化WiFi控制接口"""
    if not USE_WIFI_CONTROL:
        return None
        
    try:
        # 创建WiFi接入点
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid=WIFI_SSID, password=WIFI_PASSWORD)
        
        # 等待WiFi启动
        time.sleep(2)
        
        log(f"WiFi AP已创建: {WIFI_SSID}")
        log(f"IP地址: {ap.ifconfig()[0]}")
        
        # 创建socket服务器
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', WIFI_PORT))
        s.listen(1)
        s.setblocking(False)
        
        log(f"WiFi控制服务器监听端口: {WIFI_PORT}")
        return s
        
    except Exception as e:
        log(f"WiFi初始化失败: {e}", "ERROR")
        return None

def check_wifi_command(sock):
    """检查WiFi命令（非阻塞）"""
    if sock is None:
        return None
        
    try:
        conn, addr = sock.accept()
        conn.settimeout(0.5)  # 设置超时
        
        try:
            data = conn.recv(1024).decode().strip().upper()
            conn.send(b"OK\n")  # 发送确认
            
            if data == "START":
                log(f"收到WiFi启动命令来自 {addr}")
                return "START"
            elif data == "ABORT":
                log(f"收到WiFi中止命令来自 {addr}")
                return "ABORT"
            elif data == "STATUS":
                return "STATUS"
                
        finally:
            conn.close()
            
    except OSError:
        # 无连接，这是正常的
        pass
    except Exception as e:
        if DEBUG_MODE:
            log(f"WiFi命令检查错误: {e}", "DEBUG")
    
    return None

def initialize_system():
    """初始化所有系统组件"""
    log("=== 初始化系统组件 ===")
    
    try:
        # 初始化飞控接口
        log("初始化飞控接口...")
        fcm = FCMInterface()
        if not fcm.initialized:
            log("飞控接口初始化失败", "ERROR")
            return None, None, None
        
        # 初始化感知系统
        log("初始化感知系统...")
        perception = PerceptionSystem()
        
        # 初始化任务管理器
        log("初始化任务管理器...")
        task_manager = TaskManager(fcm, perception)
        
        log("系统组件初始化成功")
        return fcm, perception, task_manager
        
    except Exception as e:
        log(f"系统初始化异常: {e}", "ERROR")
        return None, None, None

def handle_startup_sequence(task_manager, wifi_sock):
    """处理启动序列"""
    if USE_AUTO_START:
        # 自动启动模式
        log(f"自动启动模式已启用，{AUTO_START_DELAY}秒后起飞")
        display_led_pattern("AUTO_START_COUNTDOWN")
        
        countdown_start = time.time()
        last_count = int(AUTO_START_DELAY)
        
        while time.time() - countdown_start < AUTO_START_DELAY:
            # 检查中止
            if task_manager.abort_button.value() == 0:
                log("自动启动期间按下中止按钮")
                return False
                
            # WiFi命令检查
            if check_wifi_command(wifi_sock) == "ABORT":
                return False
            
            # 显示倒计时
            remaining = AUTO_START_DELAY - (time.time() - countdown_start)
            current_count = int(remaining)
            if current_count != last_count:
                log(f"倒计时: {current_count}秒")
                last_count = current_count
            
            time.sleep(0.1)
        
        log("自动启动倒计时结束")
        return True
        
    else:
        # 手动启动模式
        display_led_pattern("WAITING_FOR_START")
        log("等待启动信号...")
        
        if USE_WIFI_CONTROL:
            log("通过WiFi发送'START'命令启动")
        log(f"或按下GPIO{START_BUTTON_PIN}按钮开始测试")
        
        start_button = machine.Pin(START_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        
        while True:
            # 检查按钮
            if start_button.value() == 0:
                time.sleep(0.05)  # 消抖
                if start_button.value() == 0:
                    log("手动启动按钮已按下")
                    return True
            
            # 检查WiFi命令
            wifi_cmd = check_wifi_command(wifi_sock)
            if wifi_cmd == "START":
                return True
            elif wifi_cmd == "ABORT":
                return False
            
            # 检查中止按钮
            if task_manager.abort_button.value() == 0:
                log("在等待期间按下中止按钮")
                return False
            
            time.sleep(0.1)

def main():
    """主程序入口 - 起飞降落测试"""
    # 打印系统信息
    print_system_info()
    log("=== ACM核心启动 - 起飞降落测试模式 ===")
    
    # 配置看门狗定时器
    wdt = machine.WDT(timeout=15000)  # 15秒超时
    log("看门狗定时器已启用（15秒超时）")
    
    # 初始化系统
    fcm, perception, task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，程序退出", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # 初始化任务管理器
    if not task_manager.initialize():
        log("任务管理器初始化失败，程序退出", "ERROR")
        display_led_pattern("ERROR")
        return
    
    # 初始化WiFi控制
    wifi_sock = initialize_wifi_control() if USE_WIFI_CONTROL else None
    
    # 主控制循环
    last_loop_time = time.ticks_ms()
    loop_counter = 0
    
    try:
        # 处理启动序列
        if not handle_startup_sequence(task_manager, wifi_sock):
            log("启动序列中止")
            return
        
        # 开始测试
        log("开始起飞降落测试")
        if not task_manager.start_test():
            log("无法开始测试", "ERROR")
            display_led_pattern("ERROR")
            return
        
        # 主循环
        while True:
            loop_counter += 1
            
            # 喂狗
            wdt.feed()
            
            # 定期垃圾回收
            if loop_counter % 50 == 0:
                gc.collect()
                if DEBUG_MODE and loop_counter % 500 == 0:
                    log(f"内存状态 - 空闲: {gc.mem_free()} 字节")
            
            # 更新飞控通信
            task_manager.fcm.update()
            
            # 检查中止条件
            if task_manager.abort_button.value() == 0:
                log("检测到手动中止信号")
                task_manager.abort_test()
                break
            
            # 检查WiFi命令
            wifi_cmd = check_wifi_command(wifi_sock)
            if wifi_cmd == "ABORT":
                log("收到WiFi中止命令")
                task_manager.abort_test()
                break
            elif wifi_cmd == "STATUS":
                # 可以通过WiFi返回状态信息
                pass
            
            # 更新任务状态
            if not task_manager.update():
                log("任务管理器完成或发生错误")
                break
            
            # 控制循环频率
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            # 性能监控
            if DEBUG_MODE and loop_time > MAIN_LOOP_DELAY * 1.5:
                log(f"警告：循环时间过长 {loop_time*1000:.1f}ms", "WARNING")
            
            # 精确延时
            sleep_time = max(0, MAIN_LOOP_DELAY - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        log("检测到键盘中断")
        task_manager.abort_test()
        
    except Exception as e:
        log(f"主循环异常: {e}", "ERROR")
        display_led_pattern("ERROR")
        
        # 尝试紧急处理
        try:
            task_manager.emergency_landing()
        except:
            try:
                if fcm:
                    fcm.emergency_stop()
            except:
                pass
    
    finally:
        # 清理资源
        log("=== 清理资源 ===")
        
        # 关闭WiFi
        if wifi_sock:
            try:
                wifi_sock.close()
                log("WiFi服务器已关闭")
            except:
                pass
        
        # 关闭通信
        if fcm:
            fcm.close()
        
        # 显示最终状态
        if hasattr(task_manager, 'state'):
            if task_manager.state == TaskState.TEST_COMPLETE:
                display_led_pattern("TEST_COMPLETE", 5)
                log("✓ 起飞降落测试成功完成！")
            else:
                log("✗ 测试未正常完成")
        
        log("=== ACM核心已关闭 ===")

if __name__ == "__main__":
    # 启动信息
    print("\n" + "="*60)
    print("  Score99团队 - 无人机起飞降落测试程序")
    print("  ESP32-S3 MicroPython ACM核心")
    print("  版本: 1.2 (优化版)")
    print("="*60 + "\n")
    
    # 短暂延迟查看信息
    time.sleep(2)
    
    # 运行主程序
    main()