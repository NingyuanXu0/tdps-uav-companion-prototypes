"""
通信模块 - 用于与飞控模块(FCM)接口
使用UART协议实现与Pixhawk4的通信
简化版本，专注于预设路径飞行

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
import struct
from config import (
    UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE, FCM_COMMANDS,
    HC12_TX_PIN, HC12_RX_PIN, HC12_SET_PIN, HC12_BAUD_RATE, HC12_CHANNEL
)
from utils import log

class HC12Interface:
    """HC12无线模块接口类，用于任务6的任务完成消息传输"""
    
    def __init__(self):
        """初始化HC12通信"""
        try:
            self.set_pin = machine.Pin(HC12_SET_PIN, machine.Pin.OUT)
            self.set_pin.value(1)  # 正常操作模式
            
            # 初始化UART
            self.uart = machine.UART(2, baudrate=HC12_BAUD_RATE, tx=HC12_TX_PIN, rx=HC12_RX_PIN)
            self.uart.init(bits=8, parity=None, stop=1, timeout=1000)
            
            # 配置HC12通道
            self._configure_channel(HC12_CHANNEL)
            
            self.initialized = True
            log("HC12通信接口初始化成功")
            
        except Exception as e:
            self.initialized = False
            log("HC12通信接口初始化失败: " + str(e), "错误")
    
    def _configure_channel(self, channel):
        """配置HC12通道"""
        self.set_pin.value(0)  # 进入AT命令模式
        time.sleep(0.1)
        
        command = "AT+C{:03d}\r\n".format(channel).encode()
        self.uart.write(command)
        time.sleep(0.1)
        
        self.set_pin.value(1)  # 退出AT命令模式
        time.sleep(0.1)
    
    def send_message(self, message):
        """发送消息"""
        if not self.initialized:
            return False
            
        try:
            self.uart.write(message.encode())
            log("通过HC12发送消息: " + message)
            return True
        except Exception as e:
            log("HC12发送失败: " + str(e), "错误")
            return False

class FCMInterface:
    """飞控模块接口类 - 使用UART与Pixhawk4通信"""
    
    def __init__(self):
        """初始化UART通信"""
        try:
            # 配置UART接口
            self.uart = machine.UART(1, baudrate=UART_BAUD_RATE, 
                                   tx=UART_TX_PIN, rx=UART_RX_PIN)
            self.uart.init(bits=8, parity=None, stop=1, timeout=100)
            
            self.initialized = True
            self.cmd_sequence = 0
            self.armed = False
            self.flight_mode = "STABILIZE"
            
            log("飞控UART接口初始化成功")
            
        except Exception as e:
            self.initialized = False
            log("飞控UART接口初始化失败: " + str(e), "错误")
    
    def _calculate_checksum(self, data):
        """计算校验和"""
        checksum = 0xFFFF
        for byte in data:
            checksum ^= byte << 8
            for _ in range(8):
                if checksum & 0x8000:
                    checksum = (checksum << 1) ^ 0x1021
                else:
                    checksum = checksum << 1
            checksum &= 0xFFFF
        return checksum
    
    def send_command(self, command, data=None):
        """发送命令到飞控"""
        if not self.initialized:
            log("飞控接口未初始化", "错误")
            return False
        
        cmd_byte = FCM_COMMANDS.get(command)
        if cmd_byte is None:
            log("未知命令: " + command, "错误")
            return False
        
        try:
            # 构建命令帧
            header = bytearray([0xFE, 0, self.cmd_sequence & 0xFF, 1, 1, cmd_byte])
            
            # 构建数据部分
            payload = bytearray()
            if data:
                if isinstance(data, (list, tuple)):
                    for value in data:
                        if isinstance(value, float):
                            payload.extend(struct.pack('<f', value))
                        elif isinstance(value, int):
                            payload.extend(struct.pack('<i', value))
            
            # 更新帧长度
            header[1] = len(payload)
            
            # 组合完整帧
            frame = header + payload
            
            # 添加校验和
            checksum = self._calculate_checksum(frame[1:])
            frame.extend(struct.pack('<H', checksum))
            
            # 发送到飞控
            self.uart.write(frame)
            time.sleep(0.01)
            
            log("命令已发送: " + command)
            self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
            
            # 更新内部状态
            if command == "MOTORS_ON":
                self.armed = True
            elif command == "MOTORS_OFF":
                self.armed = False
                
            return True
            
        except Exception as e:
            log("发送命令失败: " + str(e), "错误")
            return False
    
    def update(self):
        """定期更新（心跳）"""
        # 简化版本，不发送心跳
        return True
    
    # 基本飞行命令
    
    def motors_on(self):
        """启动电机"""
        log("命令: 启动电机")
        return self.send_command("MOTORS_ON")
    
    def motors_off(self):
        """关闭电机"""
        log("命令: 关闭电机")
        return self.send_command("MOTORS_OFF")
    
    def set_position(self, x, y, z):
        """设置目标位置"""
        log("命令: 设置位置 ({:.2f}, {:.2f}, {:.2f})米".format(x, y, z))
        return self.send_command("SET_POSITION", (x, y, z))
    
    def set_flight_mode(self, mode):
        """设置飞行模式"""
        valid_modes = ["POSITION", "STABILIZE", "LAND"]
        if mode not in valid_modes:
            log("无效飞行模式: " + mode, "错误")
            return False
            
        mode_map = {"STABILIZE": 0, "POSITION": 3, "LAND": 9}
        mode_number = mode_map.get(mode, 0)
        
        log("命令: 设置飞行模式为 " + mode)
        result = self.send_command("SET_MODE", (mode_number,))
        if result:
            self.flight_mode = mode
        return result
    
    def takeoff(self, altitude):
        """自动起飞"""
        log("命令: 自动起飞到 {:.2f}米".format(altitude))
        
        if not self.armed:
            self.motors_on()
            time.sleep(0.5)
            
        return self.send_command("TAKEOFF", (altitude,))
    
    def land(self):
        """自动降落"""
        log("命令: 自动降落")
        return self.set_flight_mode("LAND")
    
    def emergency_stop(self):
        """紧急停止"""
        log("命令: 紧急停止", "警告")
        self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")
        self.armed = False
        return True