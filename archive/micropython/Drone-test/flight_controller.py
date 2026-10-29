"""
飞行控制器模块
用于处理起飞和降落任务
"""
import time
from utils import log, display_led_pattern, format_mission_complete_message
from config import ASCENT_SPEED, DESCENT_SPEED

# 定义飞行状态
class FlightState:
    IDLE = 0
    STARTING = 1
    TAKEOFF = 2
    HOVERING = 3
    LANDING = 4
    COMPLETE = 5
    ERROR = 6

class FlightController:
    """
    飞行控制器，处理起飞和降落任务
    """
    def __init__(self, fcm, height_sensor):
        """
        初始化飞行控制器
        
        参数:
            fcm: FCMInterface实例
            height_sensor: HeightSensor实例
        """
        self.fcm = fcm
        self.height_sensor = height_sensor
        self.state = FlightState.IDLE
        
        # 计时器
        self.takeoff_start_time = None
        self.hover_start_time = None
        self.mission_complete_time = None
        
        # 记录上次输出高度的时间
        self.last_height_log = 0
    
    def start_flight(self):
        """开始飞行任务"""
        if self.state != FlightState.IDLE:
            log("无人机已在运行中", "WARNING")
            return False
            
        log("开始飞行任务")
        self.state = FlightState.STARTING
        return True
        
    def update(self, target_height, hover_time):
        """
        更新飞行控制器状态
        
        参数:
            target_height: 目标高度（米）
            hover_time: 悬停时间（秒）
            
        返回:
            任务是否完成
        """
        # 获取当前高度
        current_height = self.height_sensor.get_height()
        
        # 每秒记录一次高度
        current_time = time.time()
        if current_time - self.last_height_log >= 1.0:
            log(f"当前高度: {current_height:.2f} 米")
            self.last_height_log = current_time
        
        # 根据当前状态更新
        if self.state == FlightState.STARTING:
            # 初始化电机并开始起飞
            log("初始化电机")
            if self.fcm.motors_on():
                self.state = FlightState.TAKEOFF
                self.takeoff_start_time = time.time()
                self.fcm.set_altitude(target_height)
                log(f"开始起飞到 {target_height} 米")
                
                # 模拟模式下，逐渐增加高度
                if hasattr(self.height_sensor, "set_simulated_height"):
                    self.height_sensor.set_simulated_height(0.05)  # 起始高度
            else:
                log("启动电机失败", "ERROR")
                self.state = FlightState.ERROR
                
        elif self.state == FlightState.TAKEOFF:
            # 检查是否到达目标高度
            display_led_pattern("TAKEOFF", 0.1)
            
            # 模拟高度增加（仅在测试模式下）
            if hasattr(self.height_sensor, "set_simulated_height"):
                # 使用配置的上升速度参数
                simulated_height = min(target_height, current_height + ASCENT_SPEED)
                self.height_sensor.set_simulated_height(simulated_height)
            
            if abs(current_height - target_height) < 0.05:  # 5cm容差
                log(f"达到目标高度: {current_height:.2f} 米")
                self.state = FlightState.HOVERING
                self.hover_start_time = time.time()
            
            # 超时检查（2分钟）
            if time.time() - self.takeoff_start_time > 120:
                log("起飞超时", "WARNING")
                self.state = FlightState.LANDING
                
        elif self.state == FlightState.HOVERING:
            # 悬停指定时间
            display_led_pattern("HOVERING", 0.1)
            
            elapsed_time = time.time() - self.hover_start_time
            remaining_time = hover_time - elapsed_time
            
            if remaining_time <= 0:
                log(f"悬停完成，持续时间 {hover_time:.1f} 秒")
                self.state = FlightState.LANDING
                self.fcm.set_altitude(0.0)  # 降落命令
            else:
                if int(remaining_time) % 2 == 0 and int(remaining_time) != int(remaining_time + 0.1):  # 每2秒记录一次
                    log(f"悬停中... 剩余时间: {remaining_time:.1f} 秒")
                    
        elif self.state == FlightState.LANDING:
            # 降落过程
            display_led_pattern("LANDING", 0.1)
            
            # 模拟高度减少（仅在测试模式下）
            if hasattr(self.height_sensor, "set_simulated_height"):
                # 使用配置的下降速度参数
                simulated_height = max(0.0, current_height - DESCENT_SPEED)
                self.height_sensor.set_simulated_height(simulated_height)
            
            if current_height < 0.05:  # 接近地面（5厘米）
                log("降落完成")
                self.fcm.motors_off()
                self.state = FlightState.COMPLETE
                self.mission_complete_time = time.time()
                
        elif self.state == FlightState.COMPLETE:
            # 任务完成
            display_led_pattern("COMPLETE", 0.1)
            log("任务完成")
            return True
            
        elif self.state == FlightState.ERROR:
            # 错误状态
            display_led_pattern("ERROR", 0.1)
            self.fcm.emergency_stop()
            return True
            
        return False  # 任务仍在进行
        
    def emergency_stop(self):
        """紧急停止"""
        log("紧急停止", "WARNING")
        self.fcm.emergency_stop()
        self.state = FlightState.ERROR
        return True