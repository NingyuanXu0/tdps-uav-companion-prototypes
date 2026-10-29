import machine
import time
from config import UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE
from utils import log
from mavlink import (
    build_command_long, build_set_mode,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_NAV_TAKEOFF,
    MAV_MODE_GUIDED_ARMED
)

# LAND 命令 ID
MAV_CMD_NAV_LAND = 21

class FCMInterface:
    """
    飞控模块通信接口类 - MAVLink + UART 通信版本
    ESP32 通过串口发送 MAVLink 指令给 Pixhawk4
    """

    def __init__(self):
        try:
            self.uart = machine.UART(1,
                                     baudrate=UART_BAUDRATE,
                                     tx=machine.Pin(UART_TX_PIN),
                                     rx=machine.Pin(UART_RX_PIN))
            self.initialized = True
            log("Pixhawk UART 接口初始化成功")

            # ARM 状态与飞行模式
            self.armed = False
            self.flight_mode = None

            # MAVLink 序列号
            self._seq = 0
        except Exception as e:
            self.initialized = False
            log(f"Pixhawk UART 初始化失败: {e}", "ERROR")

    def update(self):
        """周期性读取 UART 数据并打印 (如心跳)"""
        try:
            if self.uart.any():
                data = self.uart.read(64)
                log(f"[UART] 收到数据 ({len(data)} bytes)")
        except Exception as e:
            log(f"UART 接收失败: {e}", "WARNING")

    def _send_mavlink(self, frame):
        """发送 MAVLink 数据帧并递增序列号"""
        self.uart.write(frame)
        self._seq = (self._seq + 1) & 0xFF

    def motors_on(self):
        """ARM 电机"""
        if not self.initialized:
            log("接口未初始化，无法 ARM", "ERROR")
            return False
        # Build ARM 命令
        frame = build_command_long(
            seq=self._seq,
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0
        )
        log("发送 ARM 解锁命令")
        self._send_mavlink(frame)
        self.armed = True
        return True

    def motors_off(self):
        """DISARM 电机"""
        if not self.initialized:
            log("接口未初始化，无法 DISARM", "ERROR")
            return False
        frame = build_command_long(
            seq=self._seq,
            command=MAV_CMD_COMPONENT_ARM_DISARM,
            param1=0.0
        )
        log("发送 DISARM 锁定命令")
        self._send_mavlink(frame)
        self.armed = False
        return True

    def set_flight_mode(self, mode):
        if not self.initialized:
            return False
        modes = {'MANUAL': 1 << 16, 'STABILIZE': 7 << 16,
                 'STABILIZED': 7 << 16, 'ALTITUDE': 2 << 16,
                 'POSITION': 3 << 16, 'OFFBOARD': 6 << 16,
                 'LAND': (6 << 24) | (4 << 16)}
        if mode not in modes:
            return False
        frame = build_set_mode(self._seq, 1, modes[mode])
        self._send_mavlink(frame)
        self.flight_mode = mode
        return True

    def takeoff(self, altitude):
        """起飞到指定高度"""
        if not self.initialized:
            log("接口未初始化，无法执行起飞", "ERROR")
            return False
        # 确保已 ARM
        if not self.armed:
            self.motors_on()
            time.sleep(0.5)
        # 构建 TAKEOFF 命令
        frame = build_command_long(
            seq=self._seq,
            command=MAV_CMD_NAV_TAKEOFF,
            param7=float(altitude)
        )
        log(f"发送起飞命令，目标高度: {altitude} 米")
        self._send_mavlink(frame)
        return True

    def land(self):
        """降落并关闭电机"""
        if not self.initialized:
            log("接口未初始化，无法执行降落", "ERROR")
            return False
        frame = build_command_long(
            seq=self._seq,
            command=MAV_CMD_NAV_LAND
        )
        log("发送降落命令")
        self._send_mavlink(frame)
        return True

    def emergency_stop(self):
        """紧急停止: ARM/DISARM 一起发"""
        log("命令: 紧急停止", "WARNING")
        # 作为紧急停止可以 DISARM
        return self.motors_off()

    def set_position(self, x, y, z):
        """本周期接口不支持位置控制，可通过 OFFBOARD 扩展"""
        log(f"接口不支持位置控制: ({x},{y},{z})", "WARNING")
        return False

    def set_altitude(self, altitude):
        """同 set_position"""
        log(f"接口不支持高度控制: {altitude}", "WARNING")
        return False
