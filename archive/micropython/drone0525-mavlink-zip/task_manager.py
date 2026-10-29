"""
任务管理器模块 - MAVLink版本
适配Pixhawk4的起飞降落控制

作者: Score99团队
日期: 2025年4月
"""
import time
import machine
from config import (
    TaskState, TEST_HOVER_HEIGHT, TEST_HOVER_TIME,
    ARM_RETRY_COUNT, COMMAND_RETRY_DELAY,
    TAKEOFF_TIMEOUT, LANDING_TIMEOUT
)
from utils import log, display_led_pattern

class TaskManager:
    """任务管理器类 - Pixhawk4 MAVLink版本"""
    
    def __init__(self, fcm_interface):
        """初始化任务管理器"""
        self.fcm = fcm_interface
        self.state = TaskState.INITIALIZING
        self.mission_start_time = None
        self.hover_start_time = None
        self.state_start_time = None
        self.arm_attempts = 0
        
        log("任务管理器创建 - MAVLink模式")
    
    def initialize(self):
        """初始化任务管理器"""
        log("初始化任务管理器")
        display_led_pattern("STARTUP", 2)
        
        # 确保飞控接口已初始化
        if not self.fcm.initialized:
            log("飞控接口未初始化", "错误")
            self.state = TaskState.ERROR
            return False
        
        # 发送初始心跳
        log("建立与飞控的通信...")
        for _ in range(3):
            self.fcm.update()
            time.sleep(0.2)
        
        self.state = TaskState.CONNECTING
        self.state_start_time = time.time()
        log("任务管理器初始化完成")
        return True
    
    def start_mission(self):
        """开始执行任务"""
        self.mission_start_time = time.time()
        self.state = TaskState.READY
        self.state_start_time = time.time()
        log("准备开始测试任务")
    
    def update(self):
        """更新任务管理器状态"""
        # 根据当前状态执行相应操作
        if self.state == TaskState.CONNECTING:
            return self._handle_connecting()
            
        elif self.state == TaskState.READY:
            return self._handle_ready()
            
        elif self.state == TaskState.ARMING:
            return self._handle_arming()
            
        elif self.state == TaskState.TAKEOFF:
            return self._handle_takeoff()
            
        elif self.state == TaskState.HOVERING:
            return self._handle_hovering()
            
        elif self.state == TaskState.LANDING:
            return self._handle_landing()
            
        elif self.state == TaskState.DISARMING:
            return self._handle_disarming()
            
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
        log("执行紧急停止程序", "警告")
        try:
            # 多次发送紧急停止命令
            for _ in range(3):
                self.fcm.emergency_stop()
                time.sleep(0.1)
            
            # 确保电机锁定
            self.fcm.motors_off()
            
            self.state = TaskState.ERROR
        except Exception as e:
            log("紧急停止失败: " + str(e), "错误")
    
    def _handle_connecting(self):
        """处理连接状态"""
        # 持续发送心跳
        log("正在连接飞控...")
        display_led_pattern("CONNECTING", 0.5)
        
        # 连接2秒后进入就绪状态
        if time.time() - self.state_start_time > 2:
            log("飞控连接建立")
            self.state = TaskState.READY
            self.state_start_time = time.time()
        
        return True
    
    def _handle_ready(self):
        """处理就绪状态"""
        log("系统就绪，准备解锁电机")
        self.state = TaskState.ARMING
        self.state_start_time = time.time()
        self.arm_attempts = 0
        return True
    
    def _handle_arming(self):
        """处理解锁状态"""
        display_led_pattern("ARMED", 0.5)
        
        # 尝试解锁
        if self.arm_attempts < ARM_RETRY_COUNT:
            if self.arm_attempts == 0 or time.time() - self.state_start_time > COMMAND_RETRY_DELAY * self.arm_attempts:
                log("尝试解锁电机... [{}]".format(self.arm_attempts + 1))
                
                # 先设置模式
                self.fcm.set_flight_mode("POSITION")
                time.sleep(0.5)
                
                # 解锁电机
                self.fcm.motors_on()
                self.arm_attempts += 1
                
                # 等待确认
                time.sleep(2)
                
                # 假设解锁成功（实际应该读取状态）
                if self.arm_attempts >= 2:
                    log("电机解锁成功")
                    self.state = TaskState.TAKEOFF
                    self.state_start_time = time.time()
        else:
            log("电机解锁失败", "错误")
            self.state = TaskState.ERROR
        
        return True
    
    def _handle_takeoff(self):
        """处理起飞任务"""
        display_led_pattern("TAKEOFF", 0.5)
        
        if time.time() - self.state_start_time < 1:
            # 刚进入起飞状态，发送起飞命令
            log("执行起飞命令 - 目标高度: {:.2f}米".format(TEST_HOVER_HEIGHT))
            
            # 发送起飞命令
            self.fcm.takeoff(TEST_HOVER_HEIGHT)
            
            # 或者使用位置控制
            time.sleep(1)
            self.fcm.set_position(0, 0, TEST_HOVER_HEIGHT)
        
        # 检查起飞超时
        if time.time() - self.state_start_time > TAKEOFF_TIMEOUT:
            log("起飞超时", "警告")
            self.state = TaskState.LANDING
            self.state_start_time = time.time()
            return True
        
        # 等待到达高度（简化处理，实际应读取高度）
        if time.time() - self.state_start_time > 8:
            log("到达目标高度，开始悬停")
            self.state = TaskState.HOVERING
            self.hover_start_time = time.time()
            display_led_pattern("FLYING", 0.5)
        
        return True
    
    def _handle_hovering(self):
        """处理悬停任务"""
        # 计算悬停时间
        hover_time = time.time() - self.hover_start_time
        
        if hover_time < TEST_HOVER_TIME:
            # 继续悬停
            remaining = TEST_HOVER_TIME - hover_time
            if int(hover_time) != int(hover_time - 0.1):  # 每秒输出一次
                log("悬停中... 剩余 {:.1f} 秒".format(remaining))
                
                # 保持位置
                self.fcm.set_position(0, 0, TEST_HOVER_HEIGHT)
            
            return True
        
        # 悬停时间到，开始降落
        log("悬停完成，开始降落")
        self.state = TaskState.LANDING
        self.state_start_time = time.time()
        display_led_pattern("LANDING", 0.5)
        
        return True
    
    def _handle_landing(self):
        """处理降落任务"""
        if time.time() - self.state_start_time < 1:
            log("执行降落命令")
            
            # 发送降落命令
            self.fcm.land()
        
        # 检查降落超时
        if time.time() - self.state_start_time > LANDING_TIMEOUT:
            log("降落超时", "警告")
            self.state = TaskState.DISARMING
            self.state_start_time = time.time()
            return True
        
        # 等待降落完成（简化处理）
        if time.time() - self.state_start_time > 8:
            log("降落完成")
            self.state = TaskState.DISARMING
            self.state_start_time = time.time()
        
        return True
    
    def _handle_disarming(self):
        """处理锁定电机状态"""
        log("锁定电机")
        
        # 发送锁定命令
        self.fcm.motors_off()
        time.sleep(1)
        
        # 再次确认
        self.fcm.motors_off()
        
        self.state = TaskState.COMPLETE
        display_led_pattern("COMPLETE")
        
        return True
    
    def _handle_complete(self):
        """处理任务完成状态"""
        if self.mission_start_time:
            total_time = time.time() - self.mission_start_time
            log("测试任务完成，总用时: {:.1f}秒".format(total_time))
            self.mission_start_time = None  # 防止重复输出
        
        return False  # 返回False结束主循环