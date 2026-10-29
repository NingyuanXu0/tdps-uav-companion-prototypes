"""
感知系统模块 - 起飞降落测试版本
简化版本，不使用实际传感器

作者: Score99团队
日期: 2025年4月
"""
import time
from utils import log

class PerceptionSystem:
    """感知系统类 - 测试桩版本"""
    
    def __init__(self):
        """初始化感知系统（空实现）"""
        self.initialized = True
        self._current_position = [0, 0, 0]  # x, y, z 米
        log("感知系统初始化 - 测试模式（不使用传感器）")
    
    def initialize_sensors(self):
        """初始化传感器（空实现）"""
        log("跳过传感器初始化 - 测试模式")
        return True
    
    def calibrate_sensors(self):
        """校准传感器（空实现）"""
        log("跳过传感器校准 - 测试模式")
        return True
    
    def update(self, preset_position=None):
        """更新感知数据（空实现）"""
        if preset_position:
            self._current_position = list(preset_position)
        return self._current_position
    
    def get_height(self):
        """获取当前高度（返回模拟值）"""
        return self._current_position[2]
    
    def get_current_position(self):
        """返回当前位置"""
        return self._current_position
    
    def is_at_target(self, target_position, threshold):
        """检查当前模拟位置是否到达目标位置。"""
        return all(abs(c - t) < threshold for c, t in zip(self._current_position, target_position))
