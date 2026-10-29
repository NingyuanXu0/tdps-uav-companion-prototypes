"""
简化的感知系统模块 - 起飞降落测试版本
仅保留高度测量功能，用于起飞和降落控制
"""
import machine
import time
from micropython import const
from config import (
    HEIGHT_THRESHOLD, VL53L0X_SCL_PIN, VL53L0X_SDA_PIN, 
    HCSR04_TRIGGER_PIN, HCSR04_ECHO_PIN, SENSOR_READ_RETRIES,
    SENSOR_TIMEOUT_MS, HEIGHT_CHECK_STABLE_COUNT
)
from utils import log, apply_moving_average

# VL53L0X寄存器常量
_VL53L0X_ADDR = const(0x29)
_REG_ID = const(0xC0)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_RANGE_STATUS = const(0x14)
_REG_RESULT_INTERRUPT_STATUS = const(0x13)
_REG_RESULT_RANGE_STATUS_OFFSET = const(10)

class VL53L0XSensor:
    """
    VL53L0X飞行时间距离传感器类
    提供精确的高度测量功能
    """
    def __init__(self, scl_pin=VL53L0X_SCL_PIN, sda_pin=VL53L0X_SDA_PIN, address=_VL53L0X_ADDR):
        """初始化VL53L0X传感器"""
        self.initialized = False
        self.address = address
        self.offset = 0.0
        self.calibrated = False
        self.consecutive_errors = 0
        self.max_errors = 5
        
        try:
            # 初始化I2C
            self.i2c = machine.I2C(0, 
                                 scl=machine.Pin(scl_pin), 
                                 sda=machine.Pin(sda_pin),
                                 freq=400000)
            
            # 短暂延迟让I2C稳定
            time.sleep(0.1)
            
            # 扫描I2C设备
            devices = self.i2c.scan()
            log(f"I2C设备扫描结果: {[hex(d) for d in devices]}")
            
            if address not in devices:
                log(f"未找到VL53L0X传感器（地址: 0x{address:02X}）", "ERROR")
                return
            
            # 验证传感器ID
            try:
                sensor_id = self.read_reg16(_REG_ID)
                if sensor_id != 0xEEAA:
                    log(f"VL53L0X ID验证失败: 0x{sensor_id:04X}", "ERROR")
                    return
                log("VL53L0X ID验证成功")
            except Exception as e:
                log(f"读取VL53L0X ID失败: {e}", "ERROR")
                return
            
            # 初始化传感器设置
            self._init_sensor()
            
            self.initialized = True
            log("VL53L0X传感器初始化成功")
            
        except Exception as e:
            log(f"VL53L0X初始化异常: {e}", "ERROR")
    
    def _init_sensor(self):
        """初始化传感器基本设置"""
        try:
            # 设置默认配置
            # 这里可以添加更多初始化设置
            pass
        except Exception as e:
            log(f"传感器设置初始化失败: {e}", "ERROR")
    
    def read_reg8(self, reg):
        """读取8位寄存器"""
        try:
            return self.i2c.readfrom_mem(self.address, reg, 1)[0]
        except Exception as e:
            raise Exception(f"读取寄存器0x{reg:02X}失败: {e}")
    
    def read_reg16(self, reg):
        """读取16位寄存器（大端序）"""
        try:
            data = self.i2c.readfrom_mem(self.address, reg, 2)
            return (data[0] << 8) | data[1]
        except Exception as e:
            raise Exception(f"读取16位寄存器0x{reg:02X}失败: {e}")
    
    def write_reg8(self, reg, val):
        """写入8位寄存器"""
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([val]))
        except Exception as e:
            raise Exception(f"写入寄存器0x{reg:02X}失败: {e}")
    
    def start_measurement(self):
        """启动单次测距"""
        self.write_reg8(_REG_SYSRANGE_START, 0x01)
    
    def check_measurement_ready(self):
        """检查测量是否完成"""
        status = self.read_reg8(_REG_RESULT_INTERRUPT_STATUS)
        return (status & 0x07) != 0
    
    def get_distance(self):
        """
        获取距离测量值（米）
        
        返回:
            距离（米），失败返回None
        """
        if not self.initialized:
            return None
        
        try:
            # 启动测量
            self.start_measurement()
            
            # 等待测量完成
            start_time = time.ticks_ms()
            while not self.check_measurement_ready():
                if time.ticks_diff(time.ticks_ms(), start_time) > SENSOR_TIMEOUT_MS:
                    log("VL53L0X测量超时", "WARNING")
                    self.consecutive_errors += 1
                    return None
                time.sleep(0.001)
            
            # 读取测量结果
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + _REG_RESULT_RANGE_STATUS_OFFSET)
            
            # 检查测量有效性
            if distance_mm > 8190:  # 超出范围
                log(f"VL53L0X测量超出范围: {distance_mm}mm", "WARNING")
                self.consecutive_errors += 1
                return None
            
            # 转换为米并应用校准
            distance_m = distance_mm / 1000.0
            if self.calibrated:
                distance_m += self.offset
            
            # 重置错误计数
            self.consecutive_errors = 0
            
            return distance_m
            
        except Exception as e:
            log(f"VL53L0X读取错误: {e}", "ERROR")
            self.consecutive_errors += 1
            
            # 检查是否需要重新初始化
            if self.consecutive_errors >= self.max_errors:
                log("VL53L0X错误过多，尝试重新初始化", "WARNING")
                self.initialized = False
                
            return None
    
    def calibrate(self, known_distance):
        """
        使用已知距离校准传感器
        
        参数:
            known_distance: 已知的实际距离（米）
        
        返回:
            是否校准成功
        """
        if not self.initialized:
            log("传感器未初始化，无法校准", "ERROR")
            return False
        
        try:
            log(f"开始校准，已知距离: {known_distance:.3f}米")
            
            # 多次测量取平均
            readings = []
            for i in range(10):
                reading = self.get_distance()
                if reading is not None:
                    readings.append(reading)
                time.sleep(0.05)
                
            if len(readings) < 5:
                log("校准失败：有效读数不足", "ERROR")
                return False
            
            # 计算平均值和偏移
            avg_reading = sum(readings) / len(readings)
            self.offset = known_distance - avg_reading
            self.calibrated = True
            
            log(f"校准成功 - 平均读数: {avg_reading:.3f}米, 偏移: {self.offset:.3f}米")
            return True
            
        except Exception as e:
            log(f"校准过程异常: {e}", "ERROR")
            return False

class PerceptionSystem:
    """
    感知系统主类 - 起飞降落测试版本
    集成高度传感器，提供稳定的高度估计
    """
    def __init__(self):
        """初始化感知系统"""
        self.height_sensor = None
        self._current_position = [0.0, 0.0, 0.0]  # x, y, z（米）
        self._last_update_time = time.ticks_ms()
        
        # 数据滤波
        self.height_readings = []
        self.filter_window = 5
        
        # 高度稳定性检查
        self.stable_height_buffer = []
        self.height_stable = False
        
        # 状态标志
        self.initialized = False
        self.sensor_healthy = True
        
        # 统计信息
        self.total_readings = 0
        self.failed_readings = 0
    
    def initialize_sensors(self):
        """初始化传感器"""
        log("=== 初始化感知传感器 ===")
        
        # 初始化VL53L0X高度传感器
        log("初始化VL53L0X高度传感器...")
        self.height_sensor = VL53L0XSensor()
        
        if not self.height_sensor.initialized:
            log("高度传感器初始化失败", "ERROR")
            return False
        
        # 进行初始测试读取
        log("测试传感器读取...")
        test_reading = self.height_sensor.get_distance()
        if test_reading is None:
            log("传感器测试读取失败", "WARNING")
            self.sensor_healthy = False
        else:
            log(f"传感器测试读取成功: {test_reading:.3f}米")
            self.sensor_healthy = True
        
        self.initialized = True
        log("感知系统初始化完成")
        return True
    
    def calibrate_sensors(self):
        """校准传感器（地面校准）"""
        log("=== 校准感知传感器 ===")
        
        if not self.initialized:
            log("感知系统未初始化", "ERROR")
            return False
        
        # 假设传感器到地面的实际距离
        ground_distance = 0.05  # 5厘米
        
        if self.height_sensor and self.height_sensor.initialized:
            log("校准高度传感器...")
            if self.height_sensor.calibrate(ground_distance):
                log("高度传感器校准成功")
                # 重置高度为0
                self._current_position[2] = 0.0
                return True
            else:
                log("高度传感器校准失败", "WARNING")
                # 使用默认值继续
                self.height_sensor.offset = 0.0
                self.height_sensor.calibrated = True
                return True
        
        return False
    
    def update(self):
        """
        更新感知数据
        
        返回:
            当前位置 [x, y, z]（米）
        """
        if not self.initialized:
            return self._current_position
        
        # 更新时间戳
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0
        self._last_update_time = current_time
        
        # 更新高度测量
        height = self.get_height()
        if height is not None:
            self._current_position[2] = height
            
            # 更新统计
            self.total_readings += 1
        else:
            self.failed_readings += 1
            
            # 检查传感器健康状态
            if self.failed_readings > 10:
                self.sensor_healthy = False
                log("传感器健康状态异常", "WARNING")
        
        return self._current_position.copy()
    
    def get_height(self):
        """
        获取滤波后的高度估计
        
        返回:
            高度（米），失败返回None
        """
        if not self.initialized or not self.height_sensor:
            return None
        
        # 尝试多次读取
        valid_reading = None
        for attempt in range(SENSOR_READ_RETRIES):
            reading = self.height_sensor.get_distance()
            if reading is not None:
                valid_reading = reading
                break
            time.sleep(0.01)
        
        if valid_reading is None:
            # 如果有历史数据，返回最后的滤波值
            if self.height_readings:
                return sum(self.height_readings) / len(self.height_readings)
            return None
        
        # 应用移动平均滤波
        self.height_readings, filtered_height = apply_moving_average(
            self.height_readings.copy(), 
            valid_reading, 
            self.filter_window
        )
        
        # 更新高度稳定性
        self._update_height_stability(filtered_height)
        
        return filtered_height
    
    def _update_height_stability(self, height):
        """更新高度稳定性状态"""
        self.stable_height_buffer.append(height)
        
        if len(self.stable_height_buffer) > HEIGHT_CHECK_STABLE_COUNT:
            self.stable_height_buffer.pop(0)
        
        if len(self.stable_height_buffer) == HEIGHT_CHECK_STABLE_COUNT:
            # 计算高度变化范围
            height_range = max(self.stable_height_buffer) - min(self.stable_height_buffer)
            self.height_stable = height_range < HEIGHT_THRESHOLD
    
    def get_current_position(self):
        """获取当前位置估计"""
        return self._current_position.copy()
    
    def is_at_target_height(self, target_height, threshold=HEIGHT_THRESHOLD):
        """
        检查是否达到目标高度
        
        参数:
            target_height: 目标高度（米）
            threshold: 容差阈值（米）
        
        返回:
            是否在目标高度范围内
        """
        current_height = self._current_position[2]
        in_range = abs(current_height - target_height) < threshold
        
        # 同时检查高度稳定性
        return in_range and self.height_stable
    
    def get_sensor_status(self):
        """获取传感器状态信息"""
        return {
            "initialized": self.initialized,
            "healthy": self.sensor_healthy,
            "current_height": self._current_position[2],
            "height_stable": self.height_stable,
            "total_readings": self.total_readings,
            "failed_readings": self.failed_readings,
            "failure_rate": self.failed_readings / max(self.total_readings, 1)
        }