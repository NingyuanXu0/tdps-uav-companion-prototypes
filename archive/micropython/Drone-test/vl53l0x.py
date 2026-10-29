"""
VL53L0X激光测距传感器驱动
基于MicroPython的I²C通信实现
"""
import machine
import time
import struct
from config import VL53L0X_SCL_PIN, VL53L0X_SDA_PIN, VL53L0X_ADDRESS, VL53L0X_XSHUT_PIN
from utils import log

class VL53L0X:
    """VL53L0X传感器类"""
    
    # 寄存器地址
    REG_IDENTIFICATION_MODEL_ID = 0xC0
    REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89
    REG_MSRC_CONFIG_CONTROL = 0x60
    REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
    REG_SYSTEM_SEQUENCE_CONFIG = 0x01
    REG_SYSTEM_RANGE_CONFIG = 0x09
    REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
    REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
    REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84
    REG_SYSTEM_INTERRUPT_CLEAR = 0x0B
    REG_RESULT_INTERRUPT_STATUS = 0x13
    REG_RESULT_RANGE_STATUS = 0x14
    
    def __init__(self, xshut_pin=None):
        """
        初始化VL53L0X传感器
        
        参数:
            xshut_pin: 可选的XSHUT引脚号，用于复位传感器
        """
        # 初始化I²C
        self.i2c = machine.I2C(0, scl=machine.Pin(VL53L0X_SCL_PIN), sda=machine.Pin(VL53L0X_SDA_PIN))
        self.address = VL53L0X_ADDRESS
        
        # 初始化XSHUT引脚（如果提供）
        self.xshut = None
        if xshut_pin is not None:
            self.xshut = machine.Pin(xshut_pin, machine.Pin.OUT)
            self.reset()
        
        # 检查传感器是否存在
        try:
            devices = self.i2c.scan()
            self.present = self.address in devices
            if not self.present:
                log(f"未找到VL53L0X传感器（地址：0x{self.address:02X}）", "ERROR")
                self.initialized = False
                return
            
            # 检查传感器ID
            model_id = self._read_byte(self.REG_IDENTIFICATION_MODEL_ID)
            if model_id != 0xEE:
                log(f"VL53L0X传感器ID错误：0x{model_id:02X}（应为0xEE）", "ERROR")
                self.initialized = False
                return
            
            # 初始化传感器
            self._init_sensor()
            self.initialized = True
            log("VL53L0X传感器初始化成功")
            
        except Exception as e:
            log(f"VL53L0X传感器初始化失败：{e}", "ERROR")
            self.initialized = False
    
    def reset(self):
        """使用XSHUT引脚重置传感器"""
        if self.xshut is None:
            return
            
        # 复位序列
        self.xshut.value(0)  # 低电平复位
        time.sleep(0.1)
        self.xshut.value(1)  # 高电平启用
        time.sleep(0.1)
    
    def _write_byte(self, register, value):
        """写入一个字节到指定寄存器"""
        self.i2c.writeto_mem(self.address, register, bytes([value]))
    
    def _write_reg(self, register, values):
        """写入多个字节到指定寄存器"""
        self.i2c.writeto_mem(self.address, register, values)
    
    def _read_byte(self, register):
        """从指定寄存器读取一个字节"""
        return self.i2c.readfrom_mem(self.address, register, 1)[0]
    
    def _read_reg(self, register, length=1):
        """从指定寄存器读取多个字节"""
        return self.i2c.readfrom_mem(self.address, register, length)
    
    def _init_sensor(self):
        """初始化传感器配置"""
        # 下面是VL53L0X初始化的简化版本
        # 实际应用中，完整的初始化更复杂，取决于特定的应用需求
        
        # 1. 设置高电压电平模式
        self._write_byte(self.REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
                        self._read_byte(self.REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01)
        
        # 2. 设置系统中断配置为新数据就绪
        self._write_byte(self.REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        
        # 3. 设置系统序列配置
        self._write_byte(self.REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        
        # 4. 设置范围配置
        self._write_byte(self.REG_SYSTEM_RANGE_CONFIG, 0x01)
        
        # 5. 设置最终范围配置
        self._write_byte(self.REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x00)
        
        # 6. 设置MSRC配置控制
        self._write_byte(self.REG_MSRC_CONFIG_CONTROL, 0x12)
    
    def measure_distance_mm(self):
        """测量距离（毫米）"""
        if not self.initialized:
            return 0
            
        try:
            # 1. 开始测量
            self._write_byte(self.REG_SYSTEM_RANGE_CONFIG, 0x01)
            
            # 2. 等待测量完成
            timeout = 100  # 最多等待100ms
            start_time = time.ticks_ms()
            
            while True:
                if time.ticks_diff(time.ticks_ms(), start_time) > timeout:
                    log("VL53L0X测量超时", "WARNING")
                    return 0
                    
                # 检查测量是否完成
                status = self._read_byte(self.REG_RESULT_INTERRUPT_STATUS) & 0x07
                if status == 0x04:  # 新范围数据就绪
                    break
                    
                time.sleep_ms(1)
            
            # 3. 读取测量结果
            data = self._read_reg(self.REG_RESULT_RANGE_STATUS + 10, 2)
            distance = struct.unpack('>H', data)[0]  # 大端字节序解析
            
            # 4. 清除中断状态
            self._write_byte(self.REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            return distance
            
        except Exception as e:
            log(f"VL53L0X测量失败：{e}", "ERROR")
            return 0
    
    def measure_distance_cm(self):
        """测量距离（厘米）"""
        return self.measure_distance_mm() / 10.0
    
    def measure_distance_m(self):
        """测量距离（米）"""
        return self.measure_distance_mm() / 1000.0