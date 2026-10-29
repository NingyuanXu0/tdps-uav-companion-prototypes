"""
高度传感器模块
基于VL53L0X测量飞行高度
"""
import time
from vl53l0x import VL53L0X
from config import HEIGHT_THRESHOLD, MAX_RANGE, VL53L0X_XSHUT_PIN
from utils import log

class HeightSensor:
    """高度传感器管理类"""
    
    def __init__(self):
        """初始化高度传感器"""
        self.vl53l0x = VL53L0X(xshut_pin=VL53L0X_XSHUT_PIN)
        self.initialized = self.vl53l0x.initialized
        
        # 测量历史记录（用于滤波）
        self.history = []
        self.history_size = 5  # 保留最近5次测量值
        
        # 初始化模拟模式（仅用于测试）
        self._simulation_mode = False
        self._simulated_height = 0.0
        
        log(f"高度传感器初始化{'成功' if self.initialized else '失败'}")
    
    def get_height(self):
        """
        获取当前高度（米）
        使用简单的移动平均滤波减少噪声
        """
        if self._simulation_mode:
            return self._simulated_height
            
        if not self.initialized:
            log("高度传感器未初始化，无法获取高度", "WARNING")
            return 0.0
        
        try:
            # 测量距离
            raw_distance = self.vl53l0x.measure_distance_m()
            
            # 验证测量值
            if raw_distance <= 0 or raw_distance > MAX_RANGE:
                log(f"高度测量超出范围：{raw_distance} m", "WARNING")
                # 如果历史记录为空，返回0，否则返回最近的有效值
                return 0.0 if len(self.history) == 0 else self.history[-1]
            
            # 更新历史记录
            self.history.append(raw_distance)
            if len(self.history) > self.history_size:
                self.history.pop(0)
            
            # 计算平均值
            filtered_distance = sum(self.history) / len(self.history)
            
            # 对于无人机，转换为高度（从安装位置到地面的距离）
            # 如果传感器向下安装，测量值直接对应高度
            # 如果传感器向上安装，可能需要其他转换
            height = filtered_distance
            
            return height
            
        except Exception as e:
            log(f"获取高度时发生错误：{e}", "ERROR")
            return 0.0 if len(self.history) == 0 else self.history[-1]
    
    def is_at_height(self, target_height):
        """检查当前高度是否在目标高度阈值范围内"""
        current_height = self.get_height()
        return abs(current_height - target_height) < HEIGHT_THRESHOLD
    
    def enable_simulation(self, enable=False):
        """启用或禁用模拟模式（仅用于测试）"""
        self._simulation_mode = enable
        log(f"高度传感器模拟模式{'启用' if enable else '禁用'}")
    
    def set_simulated_height(self, height):
        """设置模拟高度值（仅用于测试）"""
        self._simulated_height = height