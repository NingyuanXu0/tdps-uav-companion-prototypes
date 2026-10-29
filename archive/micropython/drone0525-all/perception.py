"""
简化的感知系统模块
专注于高度测量和基本环境感知
优化用于预设路径跟踪

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
from config import SIMULATION_MODE
from micropython import const
from config import HEIGHT_THRESHOLD, VL53L0X_SCL_PIN, VL53L0X_SDA_PIN, SENSOR_READ_RETRIES
from utils import log

# VL53L0X寄存器常量
_VL53L0X_ADDR = const(0x29)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_RANGE_STATUS = const(0x14)

class VL53L0XSensor:
    """VL53L0X飞行时间距离传感器类 - 提供精确高度测量"""
    
    def __init__(self, scl_pin=VL53L0X_SCL_PIN, sda_pin=VL53L0X_SDA_PIN):
        """初始化VL53L0X传感器"""
        try:
            self.i2c = machine.I2C(0, scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
            
            # 检查传感器是否存在
            devices = self.i2c.scan()
            if _VL53L0X_ADDR not in devices:
                log("未找到VL53L0X传感器", "错误")
                self.initialized = False
                return
                
            self.address = _VL53L0X_ADDR
            self.initialized = True
            self.offset = 0.0
            
            log("VL53L0X传感器初始化成功")
            
        except Exception as e:
            self.initialized = False
            log("VL53L0X传感器初始化失败: " + str(e), "错误")
    
    def get_distance(self):
        """获取距离测量值（米）"""
        if not self.initialized:
            return None
        
        try:
            # 启动测距
            self.i2c.writeto_mem(self.address, _REG_SYSRANGE_START, bytes([0x01]))
            
            # 等待测量完成
            time.sleep(0.03)
            
            # 读取距离（简化版本，假设固定寄存器偏移）
            data = self.i2c.readfrom_mem(self.address, _REG_RESULT_RANGE_STATUS + 10, 2)
            distance_mm = (data[0] << 8) | data[1]
            
            # 转换为米并应用偏移
            distance_m = distance_mm / 1000.0 + self.offset
            
            return distance_m
            
        except Exception as e:
            log("读取VL53L0X距离失败: " + str(e), "错误")
            return None

class PerceptionSystem:
    """主感知系统类 - 简化版本，专注于高度测量"""
    
    def __init__(self):
        """初始化感知系统"""
        self.height_sensor = None
        self._current_position = [0, 0, 0]  # x, y, z 米
        self._last_update_time = time.ticks_ms()
        
        # 数据滤波
        self.height_readings = []
        self.filter_window = 3  # 减少滤波窗口大小以提高响应速度
        
        self.initialized = False
    
    def initialize_sensors(self):
        """初始化传感器"""
        log("初始化感知传感器")
        
        # 只初始化高度传感器
        self.height_sensor = VL53L0XSensor()
        
        if not self.height_sensor.initialized:
            log("高度传感器初始化失败，将使用预设路径", "警告")
            # 即使传感器失败也继续，依赖预设路径
        
        self.initialized = True
        log("感知系统初始化完成")
        return True
    
    def calibrate_sensors(self):
        """校准传感器 - 简化版本"""
        log("校准感知传感器")
        
        # 设置初始高度为0（地面）
        self._current_position[2] = 0.0
        
        if self.height_sensor and self.height_sensor.initialized:
            # 读取几次以获得基准值
            readings = []
            for _ in range(5):
                height = self.height_sensor.get_distance()
                if height is not None:
                    readings.append(height)
                time.sleep(0.05)
            
            if readings:
                # 使用平均值作为地面参考
                ground_ref = sum(readings) / len(readings)
                self.height_sensor.offset = -ground_ref
                log("高度传感器已校准，地面参考: {:.3f}米".format(ground_ref))
        
        log("传感器校准完成")
        return True
    
    def update(self, preset_position=None):
        """更新感知数据 - 主要依赖预设位置"""
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0
        self._last_update_time = current_time
        
        # 优先使用预设位置
        if SIMULATION_MODE and preset_position:
            self._current_position = list(preset_position)
        
        # 如果有高度传感器，更新高度读数
        if self.height_sensor and self.height_sensor.initialized:
            height = self.get_height()
            if height is not None:
                if not SIMULATION_MODE:
                    self._current_position[2] = height
                # 仅使用传感器高度作为参考，不完全替代预设值
                if abs(height - self._current_position[2]) > 0.5:
                    log("高度偏差较大 - 传感器: {:.2f}米, 预设: {:.2f}米".format(
                        height, self._current_position[2]), "警告")
        
        return self._current_position
    
    def get_height(self):
        """获取当前高度估计值"""
        if not self.initialized or not self.height_sensor or not self.height_sensor.initialized:
            return self._current_position[2]  # 返回预设高度
        
        # 尝试多次读取
        for _ in range(SENSOR_READ_RETRIES):
            height = self.height_sensor.get_distance()
            if height is not None:
                # 添加到滤波窗口
                self.height_readings.append(height)
                if len(self.height_readings) > self.filter_window:
                    self.height_readings.pop(0)
                
                # 返回滤波值
                if self.height_readings:
                    return sum(self.height_readings) / len(self.height_readings)
                return height
            
            time.sleep(0.01)
        
        # 读取失败，返回最后已知高度
        return self._current_position[2]
    
    def get_current_position(self):
        """返回当前位置估计"""
        return self._current_position
    
    def is_at_target(self, target_position, threshold):
        """检查是否到达目标位置"""
        # 使用更宽松的阈值用于预设路径跟踪
        xy_threshold = threshold * 2.0  # x,y更宽松
        z_threshold = threshold        # 高度保持正常精度
        
        xy_distance = ((self._current_position[0] - target_position[0])**2 + 
                      (self._current_position[1] - target_position[1])**2) ** 0.5
        z_distance = abs(self._current_position[2] - target_position[2])
        
        return xy_distance < xy_threshold and z_distance < z_threshold