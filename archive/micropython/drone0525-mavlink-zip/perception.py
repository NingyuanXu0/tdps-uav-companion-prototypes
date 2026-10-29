"""
感知系统模块 - 集成VL53L0X TOF传感器
使用真实的飞行时间传感器进行高度测量

作者: Score99团队
日期: 2025年4月
"""
import machine
import time
from micropython import const
from utils import log

# VL53L0X寄存器定义
_VL53L0X_ADDR = const(0x29)
_REG_IDENTIFICATION_MODEL_ID = const(0xC0)
_REG_IDENTIFICATION_REVISION_ID = const(0xC2)
_REG_SYSRANGE_START = const(0x00)
_REG_RESULT_INTERRUPT_STATUS = const(0x13)
_REG_RESULT_RANGE_STATUS = const(0x14)
_REG_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = const(0xBC)
_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_REG_SYSTEM_SEQUENCE_CONFIG = const(0x01)
_REG_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)
_REG_SYSTEM_INTERRUPT_CONFIG_GPIO = const(0x0A)
_REG_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0B)

# VL53L0X配置参数
_VL53L0X_EXPECTED_DEVICE_ID = const(0xEE)
_VL53L0X_DEFAULT_I2C_ADDR = const(0x29)

class VL53L0XSensor:
    """VL53L0X飞行时间距离传感器驱动类"""
    
    def __init__(self, i2c, address=_VL53L0X_DEFAULT_I2C_ADDR):
        """
        初始化VL53L0X传感器
        
        参数:
            i2c: I2C总线对象
            address: I2C地址（默认0x29）
        """
        self.i2c = i2c
        self.address = address
        self.initialized = False
        self.offset = 0.0
        self.measurement_timing_budget = 33000  # 默认33ms
        
        # 初始化传感器
        if self._init_sensor():
            self.initialized = True
            log("VL53L0X传感器初始化成功")
        else:
            log("VL53L0X传感器初始化失败", "错误")
    
    def _init_sensor(self):
        """执行传感器初始化序列"""
        try:
            # 验证设备ID
            device_id = self._read_reg8(_REG_IDENTIFICATION_MODEL_ID)
            if device_id != _VL53L0X_EXPECTED_DEVICE_ID:
                log("VL53L0X设备ID不匹配: 0x{:02X} (期望: 0x{:02X})".format(
                    device_id, _VL53L0X_EXPECTED_DEVICE_ID), "错误")
                return False
            
            # 基本配置序列
            self._write_reg8(0x88, 0x00)  # 设置为标准模式
            self._write_reg8(0x80, 0x01)
            self._write_reg8(0xFF, 0x01)
            self._write_reg8(0x00, 0x00)
            self._read_reg8(0x91)
            self._write_reg8(0x91, 0x3C)
            self._write_reg8(0x00, 0x01)
            self._write_reg8(0xFF, 0x00)
            self._write_reg8(0x80, 0x00)
            
            # 配置单次测量模式
            self._write_reg8(_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)
            
            # 设置测量时序预算
            self._set_measurement_timing_budget(self.measurement_timing_budget)
            
            # 配置中断
            self._write_reg8(_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
            self._write_reg8(_REG_GPIO_HV_MUX_ACTIVE_HIGH, 
                           self._read_reg8(_REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10)
            self._write_reg8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            return True
            
        except Exception as e:
            log("VL53L0X初始化异常: " + str(e), "错误")
            return False
    
    def _read_reg8(self, reg):
        """读取8位寄存器"""
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]
    
    def _read_reg16(self, reg):
        """读取16位寄存器（大端序）"""
        data = self.i2c.readfrom_mem(self.address, reg, 2)
        return (data[0] << 8) | data[1]
    
    def _write_reg8(self, reg, value):
        """写入8位寄存器"""
        self.i2c.writeto_mem(self.address, reg, bytes([value]))
    
    def _write_reg16(self, reg, value):
        """写入16位寄存器（大端序）"""
        self.i2c.writeto_mem(self.address, reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))
    
    def _set_measurement_timing_budget(self, budget_us):
        """设置测量时序预算（微秒）"""
        # 简化的时序预算设置
        # 实际实现需要复杂的计算，这里使用固定值
        if budget_us < 20000:
            budget_us = 20000
        elif budget_us > 200000:
            budget_us = 200000
        
        self.measurement_timing_budget = budget_us
    
    def start_single_measurement(self):
        """启动单次测量"""
        self._write_reg8(_REG_SYSRANGE_START, 0x01)
    
    def is_measurement_ready(self):
        """检查测量是否完成"""
        return (self._read_reg8(_REG_RESULT_INTERRUPT_STATUS) & 0x07) != 0
    
    def read_range_single_millimeters(self):
        """执行单次测量并返回距离（毫米）"""
        try:
            # 清除之前的中断
            self._write_reg8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            # 启动测量
            self.start_single_measurement()
            
            # 等待测量完成（带超时）
            timeout = 100  # 100ms超时
            start_time = time.ticks_ms()
            
            while not self.is_measurement_ready():
                if time.ticks_diff(time.ticks_ms(), start_time) > timeout:
                    log("VL53L0X测量超时", "警告")
                    return None
                time.sleep_ms(1)
            
            # 读取测量结果
            range_mm = self._read_reg16(0x14 + 10)
            
            # 清除中断
            self._write_reg8(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            # 检查测量有效性
            if range_mm == 8190:  # 超出范围
                return None
            
            return range_mm
            
        except Exception as e:
            log("读取VL53L0X测量值失败: " + str(e), "错误")
            return None
    
    def get_distance(self):
        """获取距离测量值（米）"""
        distance_mm = self.read_range_single_millimeters()
        
        if distance_mm is not None:
            # 转换为米并应用校准偏移
            distance_m = distance_mm / 1000.0 + self.offset
            return distance_m
        
        return None
    
    def set_calibration_offset(self, offset):
        """设置校准偏移量（米）"""
        self.offset = offset
        log("VL53L0X校准偏移设置为: {:.3f}米".format(offset))

class PerceptionSystem:
    """集成真实传感器的感知系统类"""
    
    def __init__(self):
        """初始化感知系统"""
        self.height_sensor = None
        self._current_position = [0, 0, 0]  # x, y, z 米
        self._last_update_time = time.ticks_ms()
        
        # 高度数据滤波
        self.height_readings = []
        self.filter_window = 5  # 滤波窗口大小
        self.max_height_change_rate = 2.0  # 最大高度变化率（米/秒）
        
        # I2C配置（根据实际硬件连接调整）
        self.i2c_scl_pin = 5  # GPIO5
        self.i2c_sda_pin = 4  # GPIO4
        
        self.initialized = False
        
        log("感知系统创建 - 使用真实TOF传感器")
    
    def initialize_sensors(self):
        """初始化传感器硬件"""
        log("初始化感知传感器...")
        
        try:
            # 初始化I2C总线
            self.i2c = machine.I2C(0, 
                                 scl=machine.Pin(self.i2c_scl_pin), 
                                 sda=machine.Pin(self.i2c_sda_pin),
                                 freq=400000)
            
            # 扫描I2C设备
            devices = self.i2c.scan()
            log("发现I2C设备: " + str([hex(d) for d in devices]))
            
            # 初始化VL53L0X传感器
            if _VL53L0X_ADDR in devices:
                self.height_sensor = VL53L0XSensor(self.i2c)
                
                if self.height_sensor.initialized:
                    log("TOF高度传感器初始化成功")
                    self.initialized = True
                else:
                    log("TOF高度传感器初始化失败", "错误")
                    return False
            else:
                log("未检测到VL53L0X传感器 (地址: 0x{:02X})".format(_VL53L0X_ADDR), "错误")
                return False
                
            return True
            
        except Exception as e:
            log("传感器初始化异常: " + str(e), "错误")
            self.initialized = False
            return False
    
    def calibrate_sensors(self):
        """校准传感器（测量地面距离作为参考）"""
        if not self.initialized or not self.height_sensor:
            log("传感器未初始化，无法校准", "警告")
            return False
        
        log("开始传感器校准...")
        
        try:
            # 收集多个读数
            calibration_readings = []
            log("请将飞机放置在地面上...")
            
            for i in range(10):
                distance = self.height_sensor.get_distance()
                if distance is not None:
                    calibration_readings.append(distance)
                    log("校准读数 {}: {:.3f}米".format(i+1, distance))
                time.sleep(0.1)
            
            if len(calibration_readings) >= 5:
                # 计算平均值作为地面参考
                ground_reference = sum(calibration_readings) / len(calibration_readings)
                
                # 设置偏移量使地面读数为0
                self.height_sensor.set_calibration_offset(-ground_reference)
                
                log("传感器校准完成 - 地面参考: {:.3f}米".format(ground_reference))
                return True
            else:
                log("校准失败: 有效读数不足", "错误")
                return False
                
        except Exception as e:
            log("传感器校准异常: " + str(e), "错误")
            return False
    
    def update(self, preset_position=None):
        """更新感知数据"""
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0  # 秒
        self._last_update_time = current_time
        
        # 更新位置（如果提供了预设位置）
        if preset_position:
            self._current_position[0] = preset_position[0]
            self._current_position[1] = preset_position[1]
            # Z轴使用传感器数据
        
        # 更新高度测量
        if self.height_sensor and self.height_sensor.initialized:
            new_height = self.get_height()
            if new_height is not None:
                # 应用变化率限制（防止异常读数）
                if len(self.height_readings) > 0:
                    last_height = self.height_readings[-1]
                    height_change = new_height - last_height
                    max_change = self.max_height_change_rate * dt
                    
                    if abs(height_change) > max_change:
                        log("高度变化过快: {:.3f}米/秒".format(height_change/dt), "警告")
                        # 限制变化率
                        if height_change > 0:
                            new_height = last_height + max_change
                        else:
                            new_height = last_height - max_change
                
                self._current_position[2] = new_height
        
        return self._current_position
    
    def get_height(self):
        """获取当前高度估计值"""
        if not self.initialized or not self.height_sensor:
            return self._current_position[2]
        
        # 获取新的测量值
        raw_height = self.height_sensor.get_distance()
        
        if raw_height is not None:
            # 添加到滤波缓冲区
            self.height_readings.append(raw_height)
            
            # 维护滤波窗口大小
            if len(self.height_readings) > self.filter_window:
                self.height_readings.pop(0)
            
            # 计算滤波后的高度（去除最大最小值后取平均）
            if len(self.height_readings) >= 3:
                sorted_readings = sorted(self.height_readings)
                # 去除最高和最低值
                filtered_readings = sorted_readings[1:-1]
                filtered_height = sum(filtered_readings) / len(filtered_readings)
            else:
                # 读数不足时直接平均
                filtered_height = sum(self.height_readings) / len(self.height_readings)
            
            return filtered_height
        
        # 如果读取失败，返回最后已知高度
        if self.height_readings:
            return sum(self.height_readings) / len(self.height_readings)
        
        return self._current_position[2]
    
    def get_current_position(self):
        """返回当前位置估计"""
        return self._current_position
    
    def is_at_target(self, target_position, threshold):
        """检查是否到达目标位置"""
        # 计算与目标的距离
        distance = ((self._current_position[0] - target_position[0])**2 + 
                   (self._current_position[1] - target_position[1])**2 + 
                   (self._current_position[2] - target_position[2])**2) ** 0.5
        
        return distance < threshold
    
    def get_sensor_status(self):
        """获取传感器状态信息"""
        status = {
            "initialized": self.initialized,
            "height_sensor_active": self.height_sensor is not None and self.height_sensor.initialized,
            "current_height": self.get_height(),
            "filter_buffer_size": len(self.height_readings)
        }
        return status