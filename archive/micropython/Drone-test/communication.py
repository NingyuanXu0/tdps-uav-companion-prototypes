"""
通信模块
处理与FCM的I²C通信
"""
import machine
import struct
from config import I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQUENCY, FCM_I2C_ADDRESS, FCM_COMMANDS
from utils import log

class FCMInterface:
    """与飞行控制模块(FCM)通过I²C通信的接口"""
    
    def __init__(self):
        """初始化I²C通信"""
        try:
            self.i2c = machine.I2C(1, 
                                  scl=machine.Pin(I2C_SCL_PIN), 
                                  sda=machine.Pin(I2C_SDA_PIN), 
                                  freq=I2C_FREQUENCY)
            self.fcm_addr = FCM_I2C_ADDRESS
            
            # 检查FCM是否在I²C总线上
            devices = self.i2c.scan()
            self.initialized = self.fcm_addr in devices
            
            if self.initialized:
                log("FCM I²C接口初始化成功")
            else:
                log(f"未检测到FCM (地址 0x{self.fcm_addr:02X})", "WARNING")
                log("切换到模拟模式", "INFO")
                self.initialized = True  # 允许继续操作，但会模拟命令发送
                
        except Exception as e:
            self.initialized = False
            log(f"FCM I²C接口初始化失败: {e}", "ERROR")
    
    def send_command(self, command, data=None):
        """
        向FCM发送命令
        
        参数:
            command: FCM_COMMANDS中的命令
            data: 可选数据负载
        
        返回:
            成功发送返回True，否则返回False
        """
        if not self.initialized:
            log("FCM接口未初始化", "ERROR")
            return False
        
        cmd_byte = FCM_COMMANDS.get(command)
        if cmd_byte is None:
            log(f"未知命令: {command}", "ERROR")
            return False
        
        try:
            # 准备命令包
            packet = bytearray([cmd_byte])
            
            # 添加数据（如果提供）
            if data:
                if isinstance(data, (list, tuple)):
                    # 将数值数据转换为字节
                    for value in data:
                        if isinstance(value, float):
                            # 将浮点数转换为4字节
                            packet.extend(struct.pack('f', value))
                        elif isinstance(value, int):
                            # 将整数转换为2字节
                            packet.extend(bytes([value >> 8, value & 0xFF]))
            
            # 发送数据到FCM
            try:
                self.i2c.writeto(self.fcm_addr, packet)
                log(f"向FCM发送命令: {command}")
            except Exception as e:
                log(f"无法发送到FCM: {e}", "WARNING")
                log("以模拟模式继续", "INFO")
                
            return True
            
        except Exception as e:
            log(f"准备FCM命令时发生错误: {e}", "ERROR")
            return False
    
    def motors_on(self):
        """打开电机"""
        return self.send_command("MOTORS_ON")
    
    def motors_off(self):
        """关闭电机"""
        return self.send_command("MOTORS_OFF")
    
    def emergency_stop(self):
        """紧急停止所有电机"""
        return self.send_command("EMERGENCY_STOP")
    
    def set_altitude(self, altitude):
        """设置目标高度"""
        return self.send_command("SET_ALTITUDE", (altitude,))