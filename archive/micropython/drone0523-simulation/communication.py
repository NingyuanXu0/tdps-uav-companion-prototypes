"""
通信模块 - 起飞降落测试版本
仅保留与飞控模块(FCM)通信的基本功能
用于测试起飞和降落命令
添加仿真模式支持，无硬件环境下可正常运行
"""
import machine
import time
import struct
import math
from config import (
    I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQUENCY, FCM_I2C_ADDRESS,
    FCM_COMMANDS, SIMULATION_MODE, SIMULATION_TAKEOFF_RATE, 
    SIMULATION_LANDING_RATE, SIMULATION_HEIGHT_NOISE
)
from utils import log

"""
# 注释掉HC12无线通信类（测试阶段不需要）
# class HC12Interface:
#     HC12无线模块接口类
#     用于任务6的任务完成消息传输
#     ...
"""

class FCMInterface:
    """
    飞控模块通信接口类 - 起飞降落测试版本
    通过I2C协议与Holybro Pixhawk4飞控模块通信
    仅保留起飞降落所需的基本功能
    支持仿真模式，无硬件环境下可正常运行
    """
    
    def __init__(self):
        """初始化I2C通信或仿真模式"""
        self.simulation_mode = SIMULATION_MODE
        
        if self.simulation_mode:
            log("启动仿真模式 - 无需实际硬件连接")
            self.initialized = True
            self._init_simulation()
        else:
            self._init_hardware()
        
        # 通用初始化
        self.cmd_sequence = 0
        self.heartbeat_interval = 1.0
        self.last_heartbeat = time.time()
        
        # Pixhawk4状态参数
        self.armed = False
        self.flight_mode = "STABILIZE"
        
        # RC通道设置（中心值）
        self.rc_channels = [1500, 1500, 1000, 1500, 1100, 1100, 1100, 1100]
    
    def _init_hardware(self):
        """初始化实际硬件"""
        try:
            # 配置I2C接口
            self.i2c = machine.I2C(1, 
                                   scl=machine.Pin(I2C_SCL_PIN), 
                                   sda=machine.Pin(I2C_SDA_PIN), 
                                   freq=I2C_FREQUENCY)
            self.fcm_addr = FCM_I2C_ADDRESS
            
            # 检查Pixhawk4是否响应
            devices = self.i2c.scan()
            if self.fcm_addr not in devices:
                log(f"未在I2C总线上找到Pixhawk4 (地址:0x{self.fcm_addr:02X}). 可用设备: {[hex(d) for d in devices]}", "WARNING")
                log("自动切换到仿真模式", "INFO")
                self.simulation_mode = True
                self.initialized = True
                self._init_simulation()
                return
            else:
                log(f"Pixhawk4 I2C接口初始化成功，设备地址:0x{self.fcm_addr:02X}")
                self.initialized = True
                
        except Exception as e:
            log(f"Pixhawk4 I2C接口初始化失败: {e}，切换到仿真模式", "WARNING")
            self.simulation_mode = True
            self.initialized = True
            self._init_simulation()
    
    def _init_simulation(self):
        """初始化仿真模式参数"""
        log("仿真模式初始化完成")
        
        # 仿真状态变量
        self.sim_height = 0.0
        self.sim_target_height = 0.0
        self.sim_last_update = time.time()
        self.sim_taking_off = False
        self.sim_landing = False
        self.sim_motors_running = False
    
    def _calculate_checksum(self, data):
        """计算数据校验和"""
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return crc
    
    def send_command(self, command, data=None):
        """发送命令到Pixhawk4或仿真系统"""
        if not self.initialized:
            log("FCM接口未初始化，无法发送命令", "ERROR")
            return False
        
        cmd_byte = FCM_COMMANDS.get(command)
        if cmd_byte is None:
            log(f"未知命令: {command}", "ERROR")
            return False
        
        if self.simulation_mode:
            return self._simulate_command(command, data)
        else:
            return self._send_hardware_command(command, cmd_byte, data)
    
    def _simulate_command(self, command, data):
        """仿真命令执行"""
        log(f"仿真命令: {command}")
        
        # 更新命令序列号
        self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
        
        # 仿真不同命令的效果
        if command == "MOTORS_ON":
            self.armed = True
            self.sim_motors_running = True
            log("仿真: 电机已启动")
            
        elif command == "MOTORS_OFF":
            self.armed = False
            self.sim_motors_running = False
            self.sim_taking_off = False
            self.sim_landing = False
            log("仿真: 电机已停止")
            
        elif command == "TAKEOFF":
            if data and len(data) > 0:
                self.sim_target_height = data[0]
                self.sim_taking_off = True
                self.sim_landing = False
                log(f"仿真: 开始起飞到 {self.sim_target_height:.2f}米")
            
        elif command == "LAND":
            self.sim_target_height = 0.0
            self.sim_taking_off = False
            self.sim_landing = True
            log("仿真: 开始降落")
            
        elif command == "SET_POSITION":
            if data and len(data) >= 3:
                self.sim_target_height = data[2]  # Z坐标
                log(f"仿真: 设置目标位置 ({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f})")
            
        elif command == "SET_ALTITUDE":
            if data and len(data) > 0:
                self.sim_target_height = data[0]
                log(f"仿真: 设置目标高度 {data[0]:.2f}米")
        
        elif command == "EMERGENCY_STOP":
            self.armed = False
            self.sim_motors_running = False
            self.sim_taking_off = False
            self.sim_landing = True
            self.sim_target_height = 0.0
            log("仿真: 紧急停止")
        
        return True
    
    def _send_hardware_command(self, command, cmd_byte, data):
        """发送命令到实际硬件"""
        try:
            # 准备命令包
            start_marker = 0xFE
            system_id = 1
            component_id = 1
            
            # 构建帧头
            header = bytearray([start_marker, 0, self.cmd_sequence & 0xFF, system_id, component_id, cmd_byte])
            
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
            
            # 计算并添加校验和
            checksum = self._calculate_checksum(frame[1:])
            frame.extend(struct.pack('<H', checksum))
            
            # 发送数据包
            self.i2c.writeto(self.fcm_addr, frame)
            
            # 等待确认
            time.sleep(0.01)
            
            # 记录命令发送状态
            log(f"命令已发送到Pixhawk4: {command}, 序列号: {self.cmd_sequence}")
            self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
            
            # 更新内部状态
            if command == "MOTORS_ON":
                self.armed = True
            elif command == "MOTORS_OFF":
                self.armed = False
                
            return True
            
        except Exception as e:
            log(f"向Pixhawk4发送命令失败: {e}", "ERROR")
            return False
    
    def update(self):
        """定期更新，发送心跳包保持连接或更新仿真状态"""
        current_time = time.time()
        
        if self.simulation_mode:
            self._update_simulation()
        
        # 心跳包逻辑
        if current_time - self.last_heartbeat >= self.heartbeat_interval:
            if self.simulation_mode:
                log("仿真: 心跳包发送", "DEBUG")
                self.last_heartbeat = current_time
                return True
            else:
                return self._send_hardware_heartbeat(current_time)
        
        return True
    
    def _update_simulation(self):
        """更新仿真状态"""
        current_time = time.time()
        dt = current_time - self.sim_last_update
        self.sim_last_update = current_time
        
        if not self.sim_motors_running:
            return
        
        # 仿真高度变化
        if self.sim_taking_off and self.sim_height < self.sim_target_height:
            self.sim_height += SIMULATION_TAKEOFF_RATE * dt
            if self.sim_height >= self.sim_target_height:
                self.sim_height = self.sim_target_height
                self.sim_taking_off = False
                log(f"仿真: 到达目标高度 {self.sim_height:.2f}米")
        
        elif self.sim_landing and self.sim_height > self.sim_target_height:
            self.sim_height -= SIMULATION_LANDING_RATE * dt
            if self.sim_height <= self.sim_target_height:
                self.sim_height = self.sim_target_height
                self.sim_landing = False
                if self.sim_height <= 0.1:
                    log("仿真: 着陆完成")
        
        # 添加一些噪声使仿真更真实
        noise = (math.sin(current_time * 10) + math.cos(current_time * 7)) * SIMULATION_HEIGHT_NOISE
        self.sim_height += noise
        
        # 确保高度不为负
        if self.sim_height < 0:
            self.sim_height = 0
    
    def _send_hardware_heartbeat(self, current_time):
        """发送硬件心跳包"""
        try:
            if 'HEARTBEAT' in FCM_COMMANDS:
                heartbeat_cmd = FCM_COMMANDS['HEARTBEAT']
                cmd_frame = bytearray([0xFE, 0, self.cmd_sequence & 0xFF, 1, 1, heartbeat_cmd])
                checksum = self._calculate_checksum(cmd_frame[1:])
                cmd_frame.extend(struct.pack('<H', checksum))
                
                self.i2c.writeto(self.fcm_addr, cmd_frame)
                self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
                self.last_heartbeat = current_time
                return True
        except Exception as e:
            log(f"发送心跳包失败: {e}", "WARNING")
            return False
    
    def get_simulated_height(self):
        """获取仿真高度（仅在仿真模式下使用）"""
        if self.simulation_mode:
            return self.sim_height
        return None
    
    # 基本命令API - 仅保留起飞降落相关
    
    def motors_on(self):
        """启动电机（解锁Pixhawk4）"""
        log("命令: 启动电机（解锁）")
        return self.send_command("MOTORS_ON")
    
    def motors_off(self):
        """关闭电机（锁定Pixhawk4）"""
        log("命令: 关闭电机（锁定）")
        return self.send_command("MOTORS_OFF")
    
    def emergency_stop(self):
        """紧急停止所有电机"""
        log("命令: 紧急停止", "WARNING")
        success = self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")
        self.armed = False
        return success
    
    def set_position(self, x, y, z):
        """设置目标位置（位置模式）"""
        log(f"命令: 设置位置到 ({x:.2f}, {y:.2f}, {z:.2f})米")
        return self.send_command("SET_POSITION", (x, y, z))
    
    def set_altitude(self, altitude):
        """设置目标高度"""
        log(f"命令: 设置高度到 {altitude:.2f}米")
        return self.send_command("SET_ALTITUDE", (altitude,))
    
    def set_flight_mode(self, mode):
        """设置Pixhawk4飞行模式"""
        if not self.initialized:
            log("FCM接口未初始化，无法设置飞行模式", "ERROR")
            return False
            
        try:
            valid_modes = ["POSITION", "STABILIZE", "ALTITUDE", "LOITER", "LAND"]
            if mode not in valid_modes:
                log(f"无效的飞行模式: {mode}，有效模式为: {valid_modes}", "ERROR")
                return False
            
            # 映射模式到Pixhawk4模式编号
            mode_map = {
                "STABILIZE": 0,
                "POSITION": 3,
                "ALTITUDE": 2,
                "LOITER": 5,
                "LAND": 9
            }
            
            mode_number = mode_map[mode]
            
            if 'SET_MODE' in FCM_COMMANDS:
                log(f"命令: 设置飞行模式为 {mode}")
                result = self.send_command("SET_MODE", (mode_number,))
                if result:
                    self.flight_mode = mode
                return result
            else:
                log("飞行模式设置命令未定义", "ERROR")
                return False
                
        except Exception as e:
            log(f"设置飞行模式时出错: {e}", "ERROR")
            return False
    
    def takeoff(self, altitude):
        """自动起飞到指定高度"""
        log(f"命令: 自动起飞到 {altitude:.2f}米")
        
        # 确保电机已解锁
        if not self.armed:
            self.motors_on()
            time.sleep(0.5)
            
        # 发送起飞命令
        if 'TAKEOFF' in FCM_COMMANDS:
            return self.send_command("TAKEOFF", (altitude,))
        else:
            # 如果没有专用起飞命令，使用位置控制模拟
            self.set_flight_mode("POSITION")
            time.sleep(0.5)
            return self.set_position(0, 0, altitude)
    
    def land(self):
        """自动降落"""
        log("命令: 自动降落")
        
        if 'LAND' in FCM_COMMANDS:
            return self.send_command("LAND")
        else:
            return self.set_flight_mode("LAND")
    
    def rc_override(self, channels):
        """发送RC通道覆盖命令"""
        if len(channels) != 8:
            log(f"RC通道数量错误：需要8个，提供了{len(channels)}个", "ERROR")
            return False
            
        # 验证通道值范围
        for i, value in enumerate(channels):
            if value < 800 or value > 2200:
                log(f"通道{i+1}的值超出范围(800-2200): {value}", "WARNING")
                channels[i] = max(800, min(2200, value))
        
        self.rc_channels = channels.copy()
        log(f"命令: RC通道覆盖 {channels}")
        return self.send_command("RC_OVERRIDE", channels)

"""
# 注释掉的高级功能（完整任务时使用）
# def set_velocity(self, vx, vy, vz):
#     设置速度向量
#     pass

# def set_attitude(self, roll, pitch, yaw):
#     设置目标姿态
#     pass

# def read_status(self):
#     读取飞控状态
#     pass

# RC控制相关函数
# def arm_motors_rc(self):
#     pass

# def disarm_motors_rc(self):
#     pass

# 其他移动控制函数...
"""