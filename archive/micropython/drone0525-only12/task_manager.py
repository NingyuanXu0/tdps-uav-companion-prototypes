"""
任务管理器模块 - 起飞降落测试版本
仅实现：起飞至0.5米 → 悬停3秒 → 降落

作者: Score99团队
日期: 2025年4月
"""
import time
import machine
from config import TaskState, TEST_HOVER_HEIGHT, TEST_HOVER_TIME
from utils import log, display_led_pattern

class TaskManager:
    """任务管理器类 - 简化测试版本"""
    
    def __init__(self, fcm_interface):
        """初始化任务管理器"""
        self.fcm = fcm_interface
        self.state = TaskState.INITIALIZING
        self.mission_start_time = None
        self.hover_start_time = None
        
        log("任务管理器创建 - 测试模式")
    
    def initialize(self):
        """初始化任务管理器"""
        log("初始化任务管理器")
        display_led_pattern("STARTUP", 2)
        
        # 确保飞控接口已初始化
        if not self.fcm.initialized:
            log("飞控接口未初始化", "错误")
            self.state = TaskState.ERROR
            return False
        
        self.state = TaskState.READY
        log("任务管理器初始化完成")
        return True
    
    def start_mission(self):
        """开始执行任务"""
        self.mission_start_time = time.time()
        self.state = TaskState.TAKEOFF
        log("开始测试任务")
    
    def update(self):
        """更新任务管理器状态"""
        # 根据当前状态执行相应操作
        if self.state == TaskState.TAKEOFF:
            return self._handle_takeoff()
            
        elif self.state == TaskState.HOVERING:
            return self._handle_hovering()
            
        elif self.state == TaskState.LANDING:
            return self._handle_landing()
            
        elif self.state == TaskState.COMPLETE:
            return self._handle_complete()
            
        elif self.state == TaskState.ERROR:
            log("系统处于错误状态", "错误")
            self.fcm.emergency_stop()
            display_led_pattern("ERROR")
            return False
        
        return True
    
    def emergency_stop(self):
        """紧急停止"""
        log("紧急停止", "警告")
        try:
            self.fcm.emergency_stop()
            self.fcm.motors_off()
        except Exception as e:
            log("紧急停止失败: " + str(e), "错误")
    
    def _handle_takeoff(self):
        """处理起飞任务"""
        log("执行起飞任务")
        display_led_pattern("TAKEOFF", 0.5)
        
        # 启动电机
        if not self.fcm.armed:
            log("启动电机")
            self.fcm.motors_on()
            time.sleep(2)  # 等待电机启动
        
        # 设置飞行模式
        if self.fcm.flight_mode != "POSITION":
            log("设置位置控制模式")
            self.fcm.set_flight_mode("POSITION")
            time.sleep(1)
        
        # 发送起飞命令
        log("发送起飞命令 - 目标高度: {:.2f}米".format(TEST_HOVER_HEIGHT))
        self.fcm.set_position(0, 0, TEST_HOVER_HEIGHT)
        
        # 等待起飞完成（简化版本，使用时间延迟）
        log("等待起飞完成...")
        time.sleep(5)  # 假设5秒可以到达目标高度
        
        log("起飞完成，转换到悬停状态")
        self.state = TaskState.HOVERING
        self.hover_start_time = time.time()
        display_led_pattern("FLYING", 0.5)
        
        return True
    
    def _handle_hovering(self):
        """处理悬停任务"""
        # 检查悬停时间
        hover_time = time.time() - self.hover_start_time
        
        if hover_time < TEST_HOVER_TIME:
            # 继续悬停
            remaining = TEST_HOVER_TIME - hover_time
            if int(hover_time) % 1 == 0:  # 每秒输出一次
                log("悬停中... 剩余 {:.1f} 秒".format(remaining))
            return True
        
        # 悬停时间到，开始降落
        log("悬停完成，开始降落")
        self.state = TaskState.LANDING
        display_led_pattern("LANDING", 0.5)
        
        return True
    
    def _handle_landing(self):
        """处理降落任务"""
        log("执行降落任务")
        
        # 切换到降落模式
        if self.fcm.flight_mode != "LAND":
            log("切换到降落模式")
            self.fcm.set_flight_mode("LAND")
            time.sleep(1)
        
        # 发送降落命令
        log("发送降落命令")
        self.fcm.land()
        
        # 等待降落完成（简化版本，使用时间延迟）
        log("等待降落完成...")
        time.sleep(5)  # 假设5秒可以完成降落
        
        # 关闭电机
        log("关闭电机")
        self.fcm.motors_off()
        
        self.state = TaskState.COMPLETE
        display_led_pattern("COMPLETE")
        
        return True
    
    def _handle_complete(self):
        """处理任务完成状态"""
        total_time = time.time() - self.mission_start_time
        log("测试任务完成，总用时: {:.1f}秒".format(total_time))
        return False  # 返回False结束主循