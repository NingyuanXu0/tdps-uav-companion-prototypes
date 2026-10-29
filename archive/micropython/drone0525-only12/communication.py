"""
通信模块 - 起飞降落测试版本
仅保留与飞控的UART通信功能

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
import struct
from config import UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE, FCM_COMMANDS
from utils import log

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
    
    def land(self):
        """自动降落"""
        log("命令: 自动降落")
        return self.set_flight_mode("LAND")
    
    def emergency_stop(self):
        """紧急停止"""
        log("命令: 紧急停止", "警告")
        self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")  # 发送两次确保执行
        self.armed = False
        return True
