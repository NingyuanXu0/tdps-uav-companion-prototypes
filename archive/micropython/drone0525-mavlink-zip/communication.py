"""
通信模块 - MAVLink协议版本
实现与Pixhawk4的MAVLink通信

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
import struct
from config import UART_TX_PIN, UART_RX_PIN, MAVLINK_BAUD_RATE
from utils import log

# MAVLink消息ID定义
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128

# Pixhawk4飞行模式
PX4_CUSTOM_MODE_MANUAL = 1
PX4_CUSTOM_MODE_POSCTL = 3
PX4_CUSTOM_MODE_AUTO_LAND = 9

class MAVLinkMessage:
    """MAVLink消息构建器"""
    
    def __init__(self, system_id=255, component_id=191):
        self.system_id = system_id
        self.component_id = component_id
        self.sequence = 0
    
    def _calculate_checksum(self, data, msg_id, crc_extra):
        """计算MAVLink校验和"""
        crc = 0xFFFF
        
        # 处理数据
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        
        # 添加CRC_EXTRA
        tmp = crc_extra ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        
        return crc & 0xFFFF
    
    def build_heartbeat(self):
        payload = struct.pack('<IBBBBB', 0, 18, 8, 0, 0, 3)
        return self._build_message(0, payload, 50)

    def build_set_mode(self, base_mode, custom_mode):
        """构建设置模式消息"""
        msg_id = 11  # SET_MODE
        payload = struct.pack('<IBB', custom_mode, 1, base_mode)
        return self._build_message(msg_id, payload, 89)
    
    def build_command_long(self, command, param1=0, param2=0, param3=0,
                           param4=0, param5=0, param6=0, param7=0):
        payload = struct.pack('<7fHBBB', param1, param2, param3, param4,
                              param5, param6, param7, command,
                              1, 1, 0)
        return self._build_message(76, payload, 152)

    def build_set_position_target(self, x, y, z, vx=0, vy=0, vz=0):
        type_mask = 0x0DC0 if (vx or vy or vz) else 0x0DF8
        payload = struct.pack('<I11fHBBB', int(time.ticks_ms()),
                              x, y, -z, vx, vy, -vz, 0, 0, 0,
                              0, 0, type_mask, 1, 1, 1)
        return self._build_message(84, payload, 143)

    def _build_message(self, msg_id, payload, crc_extra):
        """构建完整的MAVLink消息"""
        # MAVLink v1.0 header
        header = bytearray([
            0xFE,  # 起始标记
            len(payload),  # 负载长度
            self.sequence & 0xFF,
            self.system_id,
            self.component_id,
            msg_id
        ])
        
        # 计算校验和
        checksum_data = header[1:] + payload
        checksum = self._calculate_checksum(checksum_data, msg_id, crc_extra)
        
        # 组合完整消息
        message = header + payload + struct.pack('<H', checksum)
        
        self.sequence = (self.sequence + 1) & 0xFF
        return message

class FCMInterface:
    """飞控模块接口类 - MAVLink协议版本"""
    
    def __init__(self):
        """初始化MAVLink通信"""
        try:
            # 配置UART接口，Pixhawk4 TELEM端口通常使用57600波特率
            self.uart = machine.UART(1, baudrate=MAVLINK_BAUD_RATE, 
                                   tx=UART_TX_PIN, rx=UART_RX_PIN)
            self.uart.init(bits=8, parity=None, stop=1, timeout=100)
            
            self.mavlink = MAVLinkMessage()
            self.initialized = True
            self.armed = False
            self.flight_mode = "MANUAL"
            self.last_heartbeat = 0
            
            log("MAVLink通信接口初始化成功")
            
            # 发送初始心跳
            self._send_heartbeat()
            
        except Exception as e:
            self.initialized = False
            log("MAVLink通信接口初始化失败: " + str(e), "错误")
    
    def _send_heartbeat(self):
        """发送心跳包"""
        message = self.mavlink.build_heartbeat()
        self.uart.write(message)
        self.last_heartbeat = time.ticks_ms()
    
    def update(self):
        """定期更新，发送心跳"""
        # 每秒发送一次心跳
        if time.ticks_diff(time.ticks_ms(), self.last_heartbeat) > 1000:
            self._send_heartbeat()
        return True
    
    def motors_on(self):
        """解锁电机"""
        log("命令: 解锁电机")
        # 发送解锁命令
        message = self.mavlink.build_command_long(
            MAV_CMD_COMPONENT_ARM_DISARM,
            1.0,  # 1=解锁
            0, 0, 0, 0, 0, 0
        )
        self.uart.write(message)
        time.sleep(0.1)
        
        # 重复发送确保接收
        self.uart.write(message)
        self.armed = True
        return True
    
    def motors_off(self):
        """锁定电机"""
        log("命令: 锁定电机")
        # 发送锁定命令
        message = self.mavlink.build_command_long(
            MAV_CMD_COMPONENT_ARM_DISARM,
            0.0,  # 0=锁定
            0, 0, 0, 0, 0, 0
        )
        self.uart.write(message)
        time.sleep(0.1)
        
        # 重复发送确保接收
        self.uart.write(message)
        self.armed = False
        return True
    
    def set_position(self, x, y, z):
        """设置目标位置"""
        log("命令: 设置位置 ({:.2f}, {:.2f}, {:.2f})米".format(x, y, z))
        
        # 发送位置目标消息
        message = self.mavlink.build_set_position_target(x, y, z)
        self.uart.write(message)
        
        # 确保心跳保持
        self._send_heartbeat()
        return True
    
    def set_flight_mode(self, mode):
        """设置飞行模式"""
        log("命令: 设置飞行模式为 " + mode)
        
        mode_map = {
            "MANUAL": PX4_CUSTOM_MODE_MANUAL << 16,
            "POSITION": PX4_CUSTOM_MODE_POSCTL << 16,
            "LAND": (6 << 24) | (4 << 16)
        }
        
        custom_mode = mode_map.get(mode, PX4_CUSTOM_MODE_MANUAL << 16)
        base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        
        if self.armed:
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
        
        # 发送模式设置消息
        message = self.mavlink.build_set_mode(base_mode, custom_mode)
        self.uart.write(message)
        time.sleep(0.1)
        
        # 重复发送确保接收
        self.uart.write(message)
        self.flight_mode = mode
        return True
    
    def takeoff(self, altitude):
        """起飞命令"""
        log("命令: 起飞到 {:.2f}米".format(altitude))
        
        # 确保已解锁
        if not self.armed:
            self.motors_on()
            time.sleep(2)
        
        # 设置位置控制模式
        self.set_flight_mode("POSITION")
        time.sleep(1)
        
        # 发送起飞命令
        message = self.mavlink.build_command_long(
            MAV_CMD_NAV_TAKEOFF,
            0,  # pitch
            0,  # empty
            0,  # empty
            0,  # yaw angle
            0,  # latitude
            0,  # longitude
            altitude  # altitude
        )
        self.uart.write(message)
        return True
    
    def land(self):
        """降落命令"""
        log("命令: 降落")
        
        # 发送降落命令
        message = self.mavlink.build_command_long(
            MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0
        )
        self.uart.write(message)
        
        # 或者切换到降落模式
        self.set_flight_mode("LAND")
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        log("命令: 紧急停止", "警告")
        
        # 立即锁定电机
        self.motors_off()
        
        # 发送多次确保执行
        for _ in range(3):
            self.motors_off()
            time.sleep(0.1)
        
        return True