"""
简化的感知系统模块 - 起飞降落测试版本
仅保留高度测量功能，用于起飞和降落控制
"""
import machine
import time
from micropython import const
from config import (
    HEIGHT_THRESHOLD, VL53L0X_SCL_PIN, VL53L0X_SDA_PIN, 
    HCSR04_TRIGGER_PIN, HCSR04_ECHO_PIN, SENSOR_READ_RETRIES
)
from utils import log

# VL53L0X常量
_VL53L0X_ADDR = const(0x29)
_REG_ID = const(0xC0)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_RANGE_STATUS = const(0x14)

class VL53L0XSensor:
    """
    VL53L0X飞行时间距离传感器类
    提供精确的高度测量功能
    """
    def __init__(self, scl_pin=VL53L0X_SCL_PIN, sda_pin=VL53L0X_SDA_PIN, address=_VL53L0X_ADDR):
        """初始化VL53L0X传感器的I2C连接"""
        try:
            self.i2c = machine.I2C(0, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
            
            # 检查传感器是否存在
            devices = self.i2c.scan()
            if address not in devices:
                log(f"未找到VL53L0X传感器，地址0x{address:02X}。可用设备: {devices}", "ERROR")
                self.initialized = False
                return
                
            # 读取传感器ID验证通信
            self.address = address
            try:
                sensor_id = self.read_reg16(_REG_ID)
                if sensor_id != 0xEEAA:
                    log(f"无效的VL53L0X ID: 0x{sensor_id:04X}", "ERROR")
                    self.initialized = False
                    return
            except Exception as e:
                log(f"读取VL53L0X ID时出错: {e}", "ERROR")
                self.initialized = False
                return
                
            self.initialized = True
            log("VL53L0X传感器初始化成功")
            
            # 校准变量
            self.offset = 0.0
            self.calibrated = False
            
        except Exception as e:
            self.initialized = False
            log(f"VL53L0X传感器初始化失败: {e}", "ERROR")
    
    def read_reg8(self, reg):
        """读取8位寄存器"""
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def read_reg16(self, reg):
        """读取16位寄存器"""
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return (data[0] << 8) | data[1]
    
    def write_reg8(self, reg, val):
        """写入8位寄存器"""
        self.i2c.writeto_mem(self.address, reg, bytes([val]))
    
    def start_ranging(self):
        """启动测距"""
        self.write_reg8(_REG_SYSRANGE_START, 0x01)
    
    def check_measurement_ready(self):
        """检查测量是否完成"""
        return (self.read_reg8(_REG_RESULT_RANGE_STATUS) & 0x01) > 0
    
    def get_distance(self):
        """
        获取距离测量值（米）
        测量失败返回None
        """
        if not self.initialized:
            return None
        
        try:
            # 启动测距
            self.start_ranging()
            
            # 等待结果（含超时）
            start_time = time.ticks_ms()
            while not self.check_measurement_ready():
                if time.ticks_diff(time.ticks_ms(), start_time) > 100:  # 100ms超时
                    log("VL53L0X测量超时", "WARNING")
                    return None
                time.sleep(0.001)
            
            # 读取距离
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + 10)
            
            # 转换为米
            distance_m = distance_mm / 1000.0
            
            # 应用校准偏移
            if self.calibrated:
                distance_m += self.offset
            
            return distance_m
            
        except Exception as e:
            log(f"读取VL53L0X距离时出错: {e}", "ERROR")
            return None
    
    def calibrate(self, known_distance):
        """使用已知距离校准传感器"""
        if not self.initialized:
            return False
        
        try:
            # 多次读取并取平均
            readings = []
            for _ in range(10):
                reading = self.get_distance()
                if reading is not None:
                    readings.append(reading)
                time.sleep(0.05)
            
            if not readings:
                log("校准失败：无有效读数", "ERROR")
                return False
            
            # 计算平均读数
            avg_reading = sum(readings) / len(readings)
            
            # 计算偏移
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"VL53L0X已校准。偏移: {self.offset:.3f}米")
            return True
            
        except Exception as e:
            log(f"VL53L0X校准时出错: {e}", "ERROR")
            return False

"""
# 注释掉HC-SR04超声波传感器类（测试阶段不需要）
# class HCSR04Sensor:
#     HC-SR04超声波传感器类
#     提供冗余的距离测量功能
#     ...
"""

class PerceptionSystem:
    """
    感知系统主类 - 起飞降落测试版本
    仅集成高度传感器，用于基本的起飞降落控制
    """
    def __init__(self):
        """初始化传感器"""
        self.height_sensor = None  # VL53L0X高度传感器
        self._current_position = [0, 0, 0]  # x, y, z（米）
        self._last_update_time = time.ticks_ms()
        
        # 数据滤波
        self.height_readings = []
        self.filter_window = 5  # 平均读数数量
        
        # 初始化标志
        self.initialized = False
    
    def initialize_sensors(self):
        """初始化起飞降落测试所需的传感器"""
        log("初始化感知传感器")
        
        # 仅初始化高度传感器
        self.height_sensor = VL53L0XSensor()
        
        # 检查高度传感器是否正确初始化
        if not self.height_sensor.initialized:
            log("高度传感器初始化失败", "ERROR")
            return False
        
        self.initialized = True
        log("感知系统初始化成功")
        return True
    
    def calibrate_sensors(self):
        """校准传感器"""
        log("校准感知传感器")
        
        # 设置初始高度为0（地面）
        self._current_position[2] = 0.0
        
        # 模拟高度传感器校准
        if self.height_sensor.initialized:
            # 假设当前在地面，距离传感器到地面的实际距离
            ground_distance = 0.05  # 假设传感器离地5cm
            if self.height_sensor.calibrate(ground_distance):
                log("高度传感器校准成功")
            else:
                log("高度传感器校准失败，使用默认值", "WARNING")
                self.height_sensor.offset = 0.0
                self.height_sensor.calibrated = True
        
        log("传感器校准完成")
        return True
    
    def update(self):
        """
        更新感知数据 - 简化版本仅更新高度
        
        返回:
            当前位置估计 [x, y, z]（米）
        """
        if not self.initialized:
            log("感知系统未初始化", "ERROR")
            return self._current_position
        
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0  # 秒
        self._last_update_time = current_time
        
        # 更新高度
        height = self.get_height()
        if height is not None:
            self._current_position[2] = height
        
        return self._current_position
    
    def get_height(self):
        """
        获取当前高度估计
        使用VL53L0X传感器并进行滤波
        """
        if not self.initialized or not self.height_sensor.initialized:
            return None
        
        # 尝试多次读取
        for _ in range(SENSOR_READ_RETRIES):
            height = self.height_sensor.get_distance()
            if height is not None:
                # 添加到滤波窗口
                self.height_readings.append(height)
                # 只保留最近N个读数
                if len(self.height_readings) > self.filter_window:
                    self.height_readings.pop(0)
                
                # 返回滤波值（读数平均值）
                if self.height_readings:
                    filtered_height = sum(self.height_readings) / len(self.height_readings)
                    return filtered_height
                return height
            
            time.sleep(0.01)  # 重试间的短暂延迟
        
        # 如果所有重试都失败，返回最后已知高度
        if self.height_readings:
            return sum(self.height_readings) / len(self.height_readings)
        
        return None
    
    def get_current_position(self):
        """返回当前位置估计"""
        return self._current_position
    
    def is_at_target_height(self, target_height, threshold=HEIGHT_THRESHOLD):
        """检查是否达到目标高度"""
        current_height = self._current_position[2]
        return abs(current_height - target_height) < threshold

"""
# 注释掉的完整任务功能
# def get_wall_distance(self):
#     获取墙壁距离
#     pass

# def is_at_target(self, target_position, threshold):
#     检查是否到达目标位置
#     pass
"""