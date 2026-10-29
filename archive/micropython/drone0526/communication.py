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
        try:
            self.uart = machine.UART(1, baudrate=UART_BAUDRATE, tx=machine.Pin(UART_TX_PIN), rx=machine.Pin(UART_RX_PIN))
            self.initialized = True
            log("Pixhawk UART 接口初始化成功")

            self.armed = False
            self.flight_mode = "STABILIZE"
        except Exception as e:
            self.initialized = False
            log(f"Pixhawk UART 初始化失败: {e}", "ERROR")

    def update(self):
        """周期性读取 UART 数据并打印"""
        try:
            if self.uart.any():
                data = self.uart.read()
                log(f"[UART] 收到数据: {data}")
        except Exception as e:
            log(f"UART 接收失败: {e}", "WARNING")

    def send_command(self, name):
        """模拟发送命令的日志输出"""
        log(f"[UART模拟] 发送命令: {name}")
        return True

    def motors_on(self):
        log("命令: 启动电机（解锁）")
        self.armed = True
        return self.send_command("MOTORS_ON")

    def motors_off(self):
        log("命令: 关闭电机（锁定）")
        self.armed = False
        return self.send_command("MOTORS_OFF")

    def emergency_stop(self):
        log("命令: 紧急停止", "WARNING")
        return self.send_command("EMERGENCY_STOP")

    def set_position(self, x, y, z):
        log(f"命令: 设置位置到 ({x:.2f}, {y:.2f}, {z:.2f}) 米")
        return self.send_command(f"SET_POSITION {x},{y},{z}")

    def set_altitude(self, altitude):
        log(f"命令: 设置高度到 {altitude:.2f} 米")
        return self.send_command(f"SET_ALTITUDE {altitude}")

    def set_flight_mode(self, mode):
        log(f"命令: 设置飞行模式为 {mode}")
        self.flight_mode = mode
        return self.send_command(f"SET_MODE {mode}")

    def takeoff(self, altitude):
        if not self.armed:
            self.motors_on()
            time.sleep(0.5)
        log(f"命令: 自动起飞到 {altitude:.2f} 米")
        return self.send_command(f"TAKEOFF {altitude}")

    def land(self):
        log("命令: 自动降落")
        return self.send_command("LAND")
