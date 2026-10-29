"""
ACM核心主程序 - MAVLink增强版本
适配Pixhawk4飞控的起飞降落测试，包含完整初始化序列

作者: Score99团队
日期: 2025年4月
"""
import time
import gc
import machine
from config import MAIN_LOOP_DELAY, AUTO_TAKEOFF_DELAY
from utils import log, display_led_pattern, system_info
from communication import FCMInterface
from task_manager import TaskManager

def check_hardware():
    """硬件检查"""
    log("执行硬件自检...")
    
    # 检查可用内存
    free_mem = gc.mem_free()
    log("可用内存: {} 字节".format(free_mem))
    
    if free_mem < 50000:  # 小于50KB
        log("内存不足，执行垃圾回收", "警告")
        gc.collect()
        free_mem = gc.mem_free()
        log("垃圾回收后可用内存: {} 字节".format(free_mem))
    
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
    log("初始化系统组件 - MAVLink增强模式")
    
    # 硬件检查
    if not check_hardware():
        return None
    
    # 初始化飞控接口
    log("正在连接Pixhawk4飞控...")
    display_led_pattern("CONNECTING", 2)
    
    fcm = FCMInterface()
    
    # 检查连接状态
    if not fcm.connection_established:
        log("无法连接到Pixhawk4飞控", "错误")
        log("请检查:")
        log("- UART连接是否正确 (TX->RX, RX->TX)")
        log("- Pixhawk4是否已上电")
        log("- 波特率是否匹配 (57600)")
        return None
    
    # 等待飞控稳定
    log("等待飞控系统稳定...")
    time.sleep(3)
    
    # 初始化任务管理器
    task_manager = TaskManager(fcm)
    
    # 执行系统检查
    if not fcm.initialized:
        log("飞控接口初始化失败", "错误")
        return None
    
    log("系统组件初始化成功")
    return task_manager

def safety_check():
    """安全检查"""
    log("="*50)
    log("安全检查清单:")
    log("✓ 螺旋桨安装牢固")
    log("✓ 电池电量充足")
    log("✓ 测试区域清空")
    log("✓ 人员保持安全距离")
    log("✓ 紧急停止程序就绪")
    log("="*50)

def main():
    """主程序入口 - MAVLink增强版本"""
    log("="*50)
    log("ACM核心启动 - Pixhawk4 MAVLink增强测试")
    log("测试任务: 起飞0.25米 → 悬停5秒 → 降落")
    log("="*50)
    
    # 显示系统信息
    system_info()
    
    # 安全检查提醒
    safety_check()
    
    # 配置看门狗定时器
    wdt = machine.WDT(timeout=30000)  # 30秒超时
    
    # 初始化系统组件
    task_manager = initialize_system()
    if task_manager is None:
        log("系统初始化失败，中止", "错误")
        display_led_pattern("ERROR", 5)
        return
    
    # 初始化任务管理器
    if not task_manager.initialize():
        log("任务管理器初始化失败，中止", "错误")
        display_led_pattern("ERROR", 5)
        return
    
    log("系统初始化完成")
    
    # 等待指定时间后自动起飞
    log("等待{}秒后自动开始任务...".format(AUTO_TAKEOFF_DELAY))
    log("重要提示:")
    log("- 确保飞机放置在平坦地面")
    log("- 确保GPS/光流传感器无遮挡")
    log("- 确保周围有足够空间（至少2米）")
    log("- 人员必须保持安全距离")
    log("- 按Ctrl+C可紧急中断")
    
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
            
            # 最后3秒额外警告
            if countdown <= 3:
                log("即将起飞！请保持安全距离！", "警告")
        
        # 保持心跳
        task_manager.fcm.update()
        time.sleep(0.1)
    
    log("开始执行测试任务")
    task_manager.start_mission()
    
    last_loop_time = time.ticks_ms()
    loop_count = 0
    error_count = 0
    
    # 主控制循环
    try:
        while True:
            # 喂狗
            wdt.feed()
            
            # 定期垃圾回收
            if loop_count % 100 == 0:
                gc.collect()
            
            try:
                # 更新任务管理器
                if not task_manager.update():
                    log("测试任务完成")
                    break
                
                # 重置错误计数
                error_count = 0
                
            except Exception as e:
                error_count += 1
                log("更新循环异常 [{}]: {}".format(error_count, str(e)), "警告")
                
                if error_count > 5:
                    log("错误次数过多，执行紧急停止", "错误")
                    task_manager.emergency_stop()
                    break
            
            # 计算循环时间
            current_time = time.ticks_ms()
            loop_time = time.ticks_diff(current_time, last_loop_time) / 1000.0
            last_loop_time = current_time
            
            # 性能监控
            if loop_time > MAIN_LOOP_DELAY * 2:
                log("循环时间过长: {:.3f}秒".format(loop_time), "警告")
            
            # 调试信息
            if loop_count % 50 == 0 and loop_count > 0:
                if hasattr(task_manager, 'state'):
                    log("循环运行中... [{}] 状态: {}".format(
                        loop_count, task_manager.state))
            
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
        
        # 尝试直接锁定电机
        try:
            for _ in range(5):
                task_manager.fcm.motors_off()
                time.sleep(0.1)
        except:
            pass
    
    finally:
        log("清理资源...")
        
        # 确保电机锁定
        try:
            if hasattr(task_manager, 'fcm'):
                task_manager.fcm.position_stream_active = False
                task_manager.fcm.offboard_enabled = False
                for _ in range(3):
                    task_manager.fcm.motors_off()
                    time.sleep(0.5)
        except:
            pass
        
        log("测试程序结束")
        display_led_pattern("COMPLETE", 3)
        
        # 最终状态报告
        log("="*50)
        log("任务执行总结:")
        log("- 总循环次数: {}".format(loop_count))
        log("- 错误次数: {}".format(error_count))
        if hasattr(task_manager, 'state'):
            log("- 最终状态: {}".format(task_manager.state))
        log("="*50)

if __name__ == "__main__":
    main()