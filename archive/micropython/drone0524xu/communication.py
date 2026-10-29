import machine
import time
from config import UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE
from utils import log

class FCMInterface:
    """
    飞控模块通信接口类 - MAVLink + UART 通信版本
    ESP32 通过串口监听 Pixhawk4 的 MAVLink 数据，并发送模拟控制命令
    """

    def __init__(self):
        """初始化UART通信接口"""
        try:
            # 初始化UART
            self.uart = machine.UART(1, 
                                   baudrate=UART_BAUDRATE, 
                                   tx=machine.Pin(UART_TX_PIN), 
                                   rx=machine.Pin(UART_RX_PIN),
                                   timeout=100,
                                   rxbuf=256)
            
            # 清空接收缓冲区
            while self.uart.any():
                self.uart.read()
            
            self.initialized = True
            self.last_heartbeat = time.ticks_ms()
            log("Pixhawk UART 接口初始化成功")

            # 飞控状态
            self.armed = False
            self.flight_mode = "STABILIZE"
            self.battery_voltage = 14.8  # 默认电压
            self.current_altitude = 0.0
            
        except Exception as e:
            self.initialized = False
            log(f"Pixhawk UART 初始化失败: {e}", "ERROR")

    def update(self):
        """周期性读取 UART 数据并处理"""
        if not self.initialized:
            return
            
        try:
            # 检查心跳超时
            if time.ticks_diff(time.ticks_ms(), self.last_heartbeat) > 5000:
                log("警告：FCM心跳超时", "WARNING")
            
            # 读取UART数据
            if self.uart.any():
                data_size = self.uart.any()
                data = self.uart.read(data_size)
                
                # 解析数据（这里简化处理，实际应解析MAVLink协议）
                if data and len(data) > 0:
                    log(f"[UART] 收到数据 ({data_size} bytes)")
                    # 更新心跳时间
                    self.last_heartbeat = time.ticks_ms()
                    
        except Exception as e:
            log(f"UART 接收失败: {e}", "WARNING")

    def send_command(self, command, params=None):
        """
        发送命令到飞控
        
        参数:
            command: 命令字符串
            params: 命令参数（可选）
        
        返回:
            是否发送成功
        """
        try:
            # 构建命令字符串
            if params:
                cmd_str = f"{command}:{params}\n"
            else:
                cmd_str = f"{command}\n"
            
            # 发送命令
            self.uart.write(cmd_str.encode())
            log(f"[UART] 发送命令: {cmd_str.strip()}")
            return True
            
        except Exception as e:
            log(f"命令发送失败: {e}", "ERROR")
            return False

    def motors_on(self):
        """启动电机（解锁）"""
        log("命令: 启动电机（解锁）")
        self.armed = True
        return self.send_command("ARM")

    def motors_off(self):
        """关闭电机（锁定）"""
        log("命令: 关闭电机（锁定）")
        self.armed = False
        return self.send_command("DISARM")

    def emergency_stop(self):
        """紧急停止"""
        log("命令: 紧急停止", "WARNING")
        self.armed = False
        return self.send_command("EMERGENCY_STOP")

    def set_position(self, x, y, z):
        """设置目标位置"""
        log(f"命令: 设置位置到 ({x:.2f}, {y:.2f}, {z:.2f}) 米")
        return self.send_command("SET_POSITION", f"{x:.2f},{y:.2f},{z:.2f}")

    def set_altitude(self, altitude):
        """设置目标高度"""
        log(f"命令: 设置高度到 {altitude:.2f} 米")
        self.current_altitude = altitude  # 更新内部状态
        return self.send_command("SET_ALTITUDE", f"{altitude:.2f}")

    def set_flight_mode(self, mode):
        """设置飞行模式"""
        valid_modes = ["STABILIZE", "POSITION", "ALTITUDE", "LAND", "LOITER"]
        if mode not in valid_modes:
            log(f"无效的飞行模式: {mode}", "ERROR")
            return False
            
        log(f"命令: 设置飞行模式为 {mode}")
        self.flight_mode = mode
        return self.send_command("SET_MODE", mode)

    def takeoff(self, altitude):
        """自动起飞"""
        if not self.armed:
            log("起飞前先解锁电机")
            self.motors_on()
            time.sleep(1.0)  # 等待解锁完成
        
        log(f"命令: 自动起飞到 {altitude:.2f} 米")
        return self.send_command("TAKEOFF", f"{altitude:.2f}")

    def land(self):
        """自动降落"""
        log("命令: 自动降落")
        return self.send_command("LAND")
    
    def get_status(self):
        """获取飞控状态"""
        return {
            "armed": self.armed,
            "mode": self.flight_mode,
            "battery": self.battery_voltage,
            "altitude": self.current_altitude,
            "connected": self.initialized
        }
    
    def close(self):
        """关闭UART连接"""
        if self.initialized and self.uart:
            try:
                self.uart.deinit()
                log("UART连接已关闭")
            except:
                pass