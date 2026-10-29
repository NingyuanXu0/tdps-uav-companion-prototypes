"""
通信模块 - MAVLink协议修复版本
增强与Pixhawk4的连接可靠性

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
import struct
from config import UART_TX_PIN, UART_RX_PIN, MAVLINK_BAUD_RATE
from utils import log, format_hex

# MAVLink消息ID定义
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_GUIDED_ENABLED = 8

# Pixhawk4飞行模式
PX4_CUSTOM_MODE_MANUAL = 1
PX4_CUSTOM_MODE_POSCTL = 3
PX4_CUSTOM_MODE_OFFBOARD = 6
PX4_CUSTOM_MODE_AUTO_LAND = 9

# MAVLink系统参数
MAVLINK_SYSTEM_ID = 255  # 地面站ID
MAVLINK_COMPONENT_ID = 190  # 伴随计算机组件ID
TARGET_SYSTEM_ID = 1  # Pixhawk系统ID
TARGET_COMPONENT_ID = 1  # Pixhawk组件ID

# 支持的波特率列表（按优先级排序）
SUPPORTED_BAUD_RATES = [57600, 115200, 921600, 38400, 9600]

class MAVLinkMessage:
    """增强的MAVLink消息构建器"""
    
    def __init__(self, system_id=MAVLINK_SYSTEM_ID, component_id=MAVLINK_COMPONENT_ID):
        self.system_id = system_id
        self.component_id = component_id
        self.sequence = 0
        self.target_system = TARGET_SYSTEM_ID
        self.target_component = TARGET_COMPONENT_ID
    
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

    def build_ping(self, seq):
        payload = struct.pack('<QIBB', int(time.ticks_us()), seq, 0, 0)
        return self._build_message(4, payload, 237)

    def build_param_request_list(self):
        """请求参数列表"""
        msg_id = 21  # PARAM_REQUEST_LIST
        payload = struct.pack('<BB', 
                            self.target_system,
                            self.target_component)
        return self._build_message(msg_id, payload, 159)
    
    def build_set_mode(self, base_mode, custom_mode):
        """构建设置模式消息"""
        msg_id = 11  # SET_MODE
        payload = struct.pack('<IBB', 
                            custom_mode, 
                            self.target_system, 
                            base_mode)
        return self._build_message(msg_id, payload, 89)
    
    def build_command_long(self, command, param1=0, param2=0, param3=0,
                           param4=0, param5=0, param6=0, param7=0):
        payload = struct.pack('<7fHBBB', param1, param2, param3, param4,
                              param5, param6, param7, command,
                              self.target_system, self.target_component, 0)
        return self._build_message(76, payload, 152)

    def build_set_position_target(self, x, y, z, vx=0, vy=0, vz=0,
                                  afx=0, afy=0, afz=0, yaw=0):
        # Position and yaw; velocity is additionally enabled when requested.
        type_mask = 0x09C0 if (vx or vy or vz) else 0x09F8
        payload = struct.pack('<I11fHBBB', int(time.ticks_ms()),
                              x, y, -z, vx, vy, -vz, afx, afy, -afz,
                              yaw, 0, type_mask, self.target_system,
                              self.target_component, 1)
        return self._build_message(84, payload, 143)

    def build_request_data_stream(self, stream_id, rate_hz):
        payload = struct.pack('<HBBBB', rate_hz, self.target_system,
                              self.target_component, stream_id, 1)
        return self._build_message(66, payload, 148)

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
    """增强的飞控模块接口类 - 修复连接问题"""
    
    def __init__(self):
        """初始化MAVLink通信"""
        self.uart = None
        self.mavlink = MAVLinkMessage()
        self.initialized = False
        self.armed = False
        self.flight_mode = "MANUAL"
        self.offboard_enabled = False
        self.last_heartbeat = 0
        self.connection_established = False
        self.position_stream_active = False
        self.current_baud_rate = None
        
        # 尝试多种连接方式
        if self._auto_connect():
            log("MAVLink通信接口初始化成功，波特率: {}".format(self.current_baud_rate))
        else:
            log("MAVLink通信接口初始化失败", "错误")
    
    def _auto_connect(self):
        """自动检测并连接到Pixhawk4"""
        log("开始自动检测Pixhawk4连接...")
        
        # 尝试不同的UART端口
        uart_ports = [1, 2]  # ESP32通常有UART1和UART2可用
        
        for uart_port in uart_ports:
            log("尝试UART{}...".format(uart_port))
            
            for baud_rate in SUPPORTED_BAUD_RATES:
                if self._try_connect(uart_port, baud_rate):
                    self.current_baud_rate = baud_rate
                    log("成功连接到UART{}，波特率: {}".format(uart_port, baud_rate))
                    return True
        
        # 如果自动检测失败，尝试使用配置的参数
        log("自动检测失败，尝试使用配置参数...")
        if self._try_connect(1, MAVLINK_BAUD_RATE):
            self.current_baud_rate = MAVLINK_BAUD_RATE
            return True
        
        return False
    
    def _try_connect(self, uart_port, baud_rate):
        """尝试使用指定参数连接"""
        try:
            # 清理之前的连接
            if self.uart:
                self.uart.deinit()
                time.sleep(0.1)
            
            # 初始化UART
            log("测试 UART{} @ {} bps".format(uart_port, baud_rate))
            self.uart = machine.UART(uart_port, baudrate=baud_rate, 
                                   tx=UART_TX_PIN, rx=UART_RX_PIN,
                                   timeout=1000)  # 增加超时时间
            
            # 清空接收缓冲区
            while self.uart.any():
                self.uart.read()
            
            # 发送心跳测试连接
            for i in range(5):
                heartbeat = self.mavlink.build_heartbeat()
                self.uart.write(heartbeat)
                time.sleep(0.1)
                
                # 检查响应
                if self.uart.any():
                    response = self.uart.read()
                    if response and len(response) > 0:
                        # 检查是否为有效的MAVLink消息
                        if response[0] == 0xFE:  # MAVLink v1起始字节
                            log("收到MAVLink响应: {}".format(format_hex(response[:6])))
                            
                            # 建立连接
                            if self._establish_connection():
                                return True
            
            # 尝试发送PING消息
            for i in range(3):
                ping = self.mavlink.build_ping(i)
                self.uart.write(ping)
                time.sleep(0.2)
                
                if self.uart.any():
                    log("收到PING响应")
                    return self._establish_connection()
            
        except Exception as e:
            log("连接尝试失败: {}".format(str(e)), "调试")
        
        return False
    
    def _build_system_time_message(self):
        """构建系统时间消息（帮助在QGC中显示）"""
        try:
            msg_id = 2  # SYSTEM_TIME
            time_unix_usec = int(time.time() * 1000000)
            time_boot_ms = int(time.ticks_ms())
        
            payload = struct.pack('<QI', time_unix_usec, time_boot_ms)
            return self.mavlink._build_message(msg_id, payload, 137)
        except:
            return None    

        
    def _establish_connection(self):
        """建立与飞控的连接"""
        log("正在建立与Pixhawk4的连接...")
        
        # 持续发送心跳
        for i in range(10):
            self._send_heartbeat()
            time.sleep(0.1)
            
            # 处理接收的消息
            if self._process_incoming_messages():
                self.connection_established = True
        
        if self.connection_established:
            # 请求数据流
            self._request_data_streams()
            
            # 请求参数列表（可选）
            # msg = self.mavlink.build_param_request_list()
            # self.uart.write(msg)
            
            self.initialized = True
            log("与Pixhawk4连接建立成功")
            return True
        else:
            log("无法建立稳定连接", "警告")
            return False
    
    def _process_incoming_messages(self):
        """处理接收的MAVLink消息"""
        if not self.uart or not self.uart.any():
            return False
        
        try:
            data = self.uart.read()
            if not data:
                return False
            
            # 查找MAVLink消息起始字节
            for i in range(len(data)):
                if data[i] == 0xFE:  # MAVLink v1
                    # 确保有足够的数据
                    if i + 6 <= len(data):
                        payload_len = data[i + 1]
                        total_len = 6 + payload_len + 2  # 头部 + 负载 + 校验和
                        
                        if i + total_len <= len(data):
                            # 提取消息
                            msg = data[i:i + total_len]
                            msg_id = msg[5]
                            
                            # 处理心跳消息
                            if msg_id == 0:  # HEARTBEAT
                                log("收到飞控心跳")
                                return True
                            # 处理其他消息类型
                            elif msg_id == 1:  # SYS_STATUS
                                log("收到系统状态")
                                return True
                            elif msg_id == 30:  # ATTITUDE
                                log("收到姿态数据")
                                return True
            
        except Exception as e:
            log("处理消息异常: {}".format(str(e)), "调试")
        
        return False
    
    def _request_data_streams(self):
        """请求必要的数据流"""
        log("请求数据流...")
        
        # 请求不同类型的数据流
        streams = [
            (0, 2),   # ALL - 2Hz
            (1, 10),  # RAW_SENSORS - 10Hz
            (2, 2),   # EXTENDED_STATUS - 2Hz
            (6, 10),  # POSITION - 10Hz
        ]
        
        for stream_id, rate in streams:
            msg = self.mavlink.build_request_data_stream(stream_id, rate)
            self.uart.write(msg)
            time.sleep(0.05)
    
    def _send_heartbeat(self):
        """发送心跳包"""
        if self.uart:
            message = self.mavlink.build_heartbeat()
            self.uart.write(message)
            self.last_heartbeat = time.ticks_ms()
    
    def update(self):
        """定期更新，发送心跳和位置目标"""
        if not self.initialized:
            return False
        
        # 每秒发送一次心跳
        if time.ticks_diff(time.ticks_ms(), self.last_heartbeat) > 1000:
            self._send_heartbeat()
        
        # 如果在OFFBOARD模式，需要持续发送位置目标
        if self.offboard_enabled and self.position_stream_active:
            # 发送当前位置目标以保持OFFBOARD模式
            self._send_position_stream()
        
        # 处理接收的消息
        self._process_incoming_messages()
        
        return True
    
    def _send_position_stream(self):
        """发送位置流以维持OFFBOARD模式"""
        # 这里应该发送实际的目标位置
        # 暂时发送当前位置作为目标
        if hasattr(self, '_current_target'):
            msg = self.mavlink.build_set_position_target(
                self._current_target[0], 
                self._current_target[1], 
                self._current_target[2]
            )
            self.uart.write(msg)
    
    def enable_offboard_mode(self):
        """启用OFFBOARD模式"""
        log("启用OFFBOARD控制模式...")
        
        # 首先发送一些位置目标消息
        self._current_target = [0, 0, 0]  # 初始位置
        for i in range(30):
            msg = self.mavlink.build_set_position_target(0, 0, 0)
            self.uart.write(msg)
            time.sleep(0.05)
        
        # 设置OFFBOARD模式
        base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg = self.mavlink.build_set_mode(base_mode, PX4_CUSTOM_MODE_OFFBOARD << 16)
        self.uart.write(msg)
        time.sleep(0.1)
        
        # 使用命令方式再次尝试
        msg = self.mavlink.build_command_long(
            MAV_CMD_DO_SET_MODE,
            base_mode,
            PX4_CUSTOM_MODE_OFFBOARD,
            0, 0, 0, 0, 0
        )
        self.uart.write(msg)
        
        self.offboard_enabled = True
        self.position_stream_active = True
        log("OFFBOARD模式启用命令已发送")
    
    def motors_on(self):
        """解锁电机"""
        if not self.initialized:
            log("飞控未初始化，无法解锁电机", "错误")
            return False
        
        log("命令: 解锁电机")
        
        # 确保在OFFBOARD模式
        if not self.offboard_enabled:
            self.enable_offboard_mode()
            time.sleep(2)
        
        # 发送解锁命令
        message = self.mavlink.build_command_long(
            MAV_CMD_COMPONENT_ARM_DISARM,
            1.0,  # 1=解锁
            0.0,  # Normal arm command; retain PX4 checks
            0, 0, 0, 0, 0
        )
        
        for i in range(3):
            self.uart.write(message)
            time.sleep(0.5)
        
        self.armed = True
        return True
    
    def motors_off(self):
        """锁定电机"""
        if not self.initialized:
            return False
        
        log("命令: 锁定电机")
        
        # 发送锁定命令
        message = self.mavlink.build_command_long(
            MAV_CMD_COMPONENT_ARM_DISARM,
            0.0,  # 0=锁定
            0.0,  # Normal disarm command
            0, 0, 0, 0, 0
        )
        
        for i in range(3):
            self.uart.write(message)
            time.sleep(0.5)
        
        self.armed = False
        self.position_stream_active = False
        return True
    
    def set_position(self, x, y, z):
        """设置目标位置"""
        if not self.initialized:
            return False
        
        log("命令: 设置位置 ({:.2f}, {:.2f}, {:.2f})米".format(x, y, z))
        
        # 保存当前目标
        self._current_target = [x, y, z]
        
        # 发送位置目标消息
        message = self.mavlink.build_set_position_target(x, y, z)
        
        # 多次发送确保接收
        for i in range(3):
            self.uart.write(message)
            time.sleep(0.02)
        
        return True
    
    def set_flight_mode(self, mode):
        """设置飞行模式"""
        if not self.initialized:
            return False
        
        log("命令: 设置飞行模式为 " + mode)
        
        mode_map = {
            "MANUAL": PX4_CUSTOM_MODE_MANUAL << 16,
            "POSITION": PX4_CUSTOM_MODE_POSCTL << 16,
            "OFFBOARD": PX4_CUSTOM_MODE_OFFBOARD << 16,
            "LAND": (6 << 24) | (4 << 16)
        }
        
        custom_mode = mode_map.get(mode, PX4_CUSTOM_MODE_MANUAL << 16)
        base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        
        if self.armed:
            base_mode |= MAV_MODE_FLAG_SAFETY_ARMED
        
        # 发送模式设置消息
        message = self.mavlink.build_set_mode(base_mode, custom_mode)
        
        for i in range(3):
            self.uart.write(message)
            time.sleep(0.1)
        
        self.flight_mode = mode
        return True
    
    def takeoff(self, altitude):
        """起飞命令"""
        if not self.initialized:
            return False
        
        log("命令: 起飞到 {:.2f}米".format(altitude))
        
        # 确保已解锁
        if not self.armed:
            self.motors_on()
            time.sleep(2)
        
        # 在OFFBOARD模式下起飞
        if self.offboard_enabled:
            # 逐步增加高度
            steps = 10
            for i in range(steps):
                intermediate_alt = altitude * (i + 1) / steps
                self.set_position(0, 0, intermediate_alt)
                time.sleep(0.5)
        else:
            # 使用起飞命令
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
        if not self.initialized:
            return False
        
        log("命令: 降落")
        
        # 发送降落命令
        message = self.mavlink.build_command_long(
            MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0
        )
        self.uart.write(message)
        
        # 或者切换到降落模式
        self.set_flight_mode("LAND")
        
        # 停止位置流
        self.position_stream_active = False
        
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        log("命令: 紧急停止", "警告")
        
        # 停止位置流
        self.position_stream_active = False
        self.offboard_enabled = False
        
        # 立即锁定电机
        for _ in range(5):
            self.motors_off()
            time.sleep(0.1)
        
        return True
    
    def get_connection_info(self):
        """获取连接信息"""
        return {
            "initialized": self.initialized,
            "connected": self.connection_established,
            "baud_rate": self.current_baud_rate,
            "armed": self.armed,
            "mode": self.flight_mode,
            "offboard": self.offboard_enabled
        }