"""
传感器模拟器 - 模拟无人机传感器的行为
用于在计算机上测试ACM代码
"""
import time
import random
import math

class MockVL53L0XSensor:
    """模拟VL53L0X测距传感器"""
    
    def __init__(self, initial_height=0.0, noise_level=0.02):
        """
        初始化模拟传感器
        
        Args:
            initial_height: 初始高度值（米）
            noise_level: 噪声水平（米）
        """
        self.initialized = True
        self.current_height = initial_height
        self.noise_level = noise_level
        self.calibrated = False
        self.offset = 0.0
        print("模拟VL53L0X高度传感器已初始化")
    
    def set_current_height(self, height):
        """设置当前高度值（用于模拟环境控制）"""
        self.current_height = height
    
    def get_distance(self):
        """
        获取带噪声的距离测量值
        
        Returns:
            当前高度测量值（米）+ 随机噪声
        """
        if not self.initialized:
            return None
            
        # 添加随机噪声
        noise = (random.random() * 2 - 1) * self.noise_level
        measured_height = self.current_height + noise
        
        # 应用校准偏移
        if self.calibrated:
            measured_height += self.offset
            
        return measured_height
    
    def calibrate(self, known_distance):
        """
        校准传感器
        
        Args:
            known_distance: 已知距离（米）
        
        Returns:
            校准成功返回True，否则返回False
        """
        if not self.initialized:
            return False
            
        # 模拟测量
        measurements = []
        for _ in range(10):
            noise = (random.random() * 2 - 1) * self.noise_level
            measurements.append(self.current_height + noise)
            
        # 计算平均值
        avg_measurement = sum(measurements) / len(measurements)
        
        # 计算偏移
        self.offset = known_distance - avg_measurement
        self.calibrated = True
        
        print(f"模拟VL53L0X已校准. 偏移: {self.offset:.3f}m")
        return True

class MockHCSR04Sensor:
    """模拟HC-SR04超声波传感器"""
    
    def __init__(self, initial_distance=1.0, noise_level=0.05):
        """
        初始化模拟传感器
        
        Args:
            initial_distance: 初始距离值（米）
            noise_level: 噪声水平（米）
        """
        self.initialized = True
        self.current_distance = initial_distance
        self.noise_level = noise_level
        self.calibrated = False
        self.offset = 0.0
        print("模拟HC-SR04距离传感器已初始化")
    
    def set_current_distance(self, distance):
        """设置当前距离值（用于模拟环境控制）"""
        self.current_distance = distance
    
    def get_distance(self):
        """
        获取带噪声的距离测量值
        
        Returns:
            当前距离测量值（米）+ 随机噪声
        """
        if not self.initialized:
            return None
            
        # 添加随机噪声
        noise = (random.random() * 2 - 1) * self.noise_level
        measured_distance = self.current_distance + noise
        
        # 应用校准偏移
        if self.calibrated:
            measured_distance += self.offset
            
        # 模拟测量超时（5%概率）
        if random.random() < 0.05:
            return None
            
        return measured_distance
    
    def calibrate(self, known_distance):
        """
        校准传感器
        
        Args:
            known_distance: 已知距离（米）
        
        Returns:
            校准成功返回True，否则返回False
        """
        if not self.initialized:
            return False
            
        # 模拟测量
        measurements = []
        for _ in range(10):
            if random.random() < 0.05:  # 模拟5%测量失败率
                continue
                
            noise = (random.random() * 2 - 1) * self.noise_level
            measurements.append(self.current_distance + noise)
            
        if not measurements:
            return False
            
        # 计算平均值
        avg_measurement = sum(measurements) / len(measurements)
        
        # 计算偏移
        self.offset = known_distance - avg_measurement
        self.calibrated = True
        
        print(f"模拟HC-SR04已校准. 偏移: {self.offset:.3f}m")
        return True