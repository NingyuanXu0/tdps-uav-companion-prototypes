"""
任务管理器模块 - 处理无人机任务执行
实现简化的任务状态机
任务流程：起飞 → 前进 → 左转 → 后退 → 降落并通信

作者: Score99团队
日期: 2025年4月
"""
import time
import config as _board_config
import machine
from config import (
    PATH_WAYPOINTS, TASK_TRANSITION_INDICES, POSITION_THRESHOLD,
    MISSION_COMPLETE_WAIT_TIME, BATTERY_ADC_PIN,
    LOW_BATTERY_THRESHOLD, CRITICAL_BATTERY_THRESHOLD,
    TaskState  # 从config导入任务状态常量
)
from utils import log, display_led_pattern, format_mission_complete_message, is_position_reached

class TaskManager:
    """任务管理器类 - 管理无人机任务执行"""
    
    def __init__(self, fcm_interface, perception_system):
        """初始化任务管理器"""
        self.fcm = fcm_interface
        self.perception = perception_system
        
        self.state = TaskState.INITIALIZING
        self.current_waypoint_index = 0
        self.mission_start_time = None
        self.mission_complete_time = None
        
        # 电池监控
        self.LOW_BATTERY_THRESHOLD = LOW_BATTERY_THRESHOLD
        self.CRITICAL_BATTERY_THRESHOLD = CRITICAL_BATTERY_THRESHOLD
        self.battery_adc = machine.ADC(machine.Pin(BATTERY_ADC_PIN))
        self.battery_adc.atten(machine.ADC.ATTN_11DB)
        
        # 错误跟踪
        self.consecutive_errors = 0
        self.max_consecutive_errors = 3
        
        # 初始化HC12通信接口
        from communication import HC12Interface
        self.hc12 = HC12Interface()
    
    def initialize(self):
        """初始化任务管理器"""
        divider = _board_config.BATTERY_VOLTAGE_DIVIDER
        if divider is None or divider <= 0 or 3.3 * divider <= LOW_BATTERY_THRESHOLD:
            log("Configure BATTERY_VOLTAGE_DIVIDER for the actual ADC circuit", "ERROR")
            return False
        log("初始化任务管理器")
        display_led_pattern("STARTUP", 3)
        
        # 确保飞控接口已初始化
        if not self.fcm.initialized:
            log("飞控接口未初始化", "错误")
            self.state = TaskState.ERROR
            return False
            
        # 初始化传感器
        if not self.perception.initialize_sensors():
            log("传感器初始化失败", "错误")
            # 继续执行，依赖预设路径
        
        # 校准传感器
        log("校准传感器...")
        try:
            self.perception.calibrate_sensors()
            log("传感器校准完成")
        except Exception as e:
            log("传感器校准失败: " + str(e), "警告")
            # 继续执行
        
        self.state = TaskState.TAKEOFF
        log("任务管理器初始化完成")
        return True
    
    def start_mission(self):
        """开始执行任务"""
        self.mission_start_time = time.time()
        self.state = TaskState.TAKEOFF
        log("任务开始")
    
    def update(self):
        """更新任务管理器状态"""
        # 获取当前航点
        if self.current_waypoint_index < len(PATH_WAYPOINTS):
            current_waypoint = PATH_WAYPOINTS[self.current_waypoint_index]
        else:
            current_waypoint = PATH_WAYPOINTS[-1]
        
        # 更新感知系统（使用预设位置）
        try:
            self.perception.update(current_waypoint)
            current_position = self.perception.get_current_position()
            self.consecutive_errors = 0
        except Exception as e:
            log("获取位置数据错误: " + str(e), "错误")
            self.consecutive_errors += 1
            
            if self.consecutive_errors >= self.max_consecutive_errors:
                log("连续错误过多，切换到错误状态", "错误")
                self.state = TaskState.ERROR
                return False
            
            # 使用预设位置作为后备
            current_position = self.perception.get_current_position()
        
        # 检查是否到达当前航点
        waypoint_reached = is_position_reached(
            current_position, current_waypoint, POSITION_THRESHOLD
        )
        
        # 根据当前状态执行相应操作
        if self.state == TaskState.TAKEOFF:
            return self._handle_takeoff(waypoint_reached)
            
        elif self.state == TaskState.FORWARD_FLIGHT:
            return self._handle_forward_flight(waypoint_reached)
            
        elif self.state == TaskState.LEFT_TURN:
            return self._handle_left_turn(waypoint_reached)
            
        elif self.state == TaskState.BACKWARD_FLIGHT:
            return self._handle_backward_flight(waypoint_reached)
            
        elif self.state == TaskState.LANDING:
            return self._handle_landing(waypoint_reached)
            
        elif self.state == TaskState.MISSION_COMPLETE:
            return self._handle_mission_complete()
            
        elif self.state == TaskState.ERROR:
            log("系统处于错误状态", "错误")
            self.fcm.emergency_stop()
            display_led_pattern("ERROR")
            return False
        
        return True
    
    def get_battery_voltage(self):
        """获取当前电池电压"""
        raw_value = self.battery_adc.read()
        voltage = raw_value * 3.3 / 4095 * _board_config.BATTERY_VOLTAGE_DIVIDER  # 假设3:1分压比
        return voltage
    
    def emergency_landing(self):
        """紧急降落"""
        log("紧急降落启动", "警告")
        
        try:
            # 切换到降落模式
            self.fcm.land()
            
            # 等待降落
            start_time = time.time()
            while time.time() - start_time < 10:  # 最多等待10秒
                height = self.perception.get_height()
                if height < 0.3:  # 接近地面
                    break
                time.sleep(0.5)
                
            # 停止电机
            self.fcm.motors_off()
            log("紧急降落完成")
            
        except Exception as e:
            log("紧急降落期间出错: " + str(e), "错误")
            self.fcm.emergency_stop()
    
    def _move_to_next_waypoint(self):
        """前进到下一个航点"""
        if self.current_waypoint_index < len(PATH_WAYPOINTS) - 1:
            self.current_waypoint_index += 1
            next_waypoint = PATH_WAYPOINTS[self.current_waypoint_index]
            self.fcm.set_position(*next_waypoint)
            log("移动到航点 {}: {}".format(self.current_waypoint_index, next_waypoint))
            return True
        else:
            log("已到达最终航点", "警告")
            return False
    
    def _handle_takeoff(self, waypoint_reached):
        """处理起飞任务"""
        display_led_pattern("TAKEOFF", 0.5)
        
        if self.current_waypoint_index == 0:
            # 初始化飞行
            self.fcm.motors_on()
            self.fcm.set_flight_mode("POSITION")
            self._move_to_next_waypoint()
            log("开始起飞")
        
        if waypoint_reached:
            log("到达起飞航点 {}".format(self.current_waypoint_index))
            
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["TAKEOFF_COMPLETE"] - 1:
                log("起飞完成，转换到前进飞行")
                self.state = TaskState.FORWARD_FLIGHT
                display_led_pattern("FLYING", 0.5)
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_forward_flight(self, waypoint_reached):
        """处理前进飞行任务（从障碍物下方通过）"""
        if waypoint_reached:
            log("到达前进航点 {}".format(self.current_waypoint_index))
            
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["FORWARD_COMPLETE"] - 1:
                log("前进飞行完成，转换到左转")
                self.state = TaskState.LEFT_TURN
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_left_turn(self, waypoint_reached):
        """处理左转任务（穿过障碍物）"""
        if waypoint_reached:
            log("到达左转航点 {}".format(self.current_waypoint_index))
            
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["LEFT_TURN_COMPLETE"] - 1:
                log("左转完成，转换到后退飞行")
                self.state = TaskState.BACKWARD_FLIGHT
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_backward_flight(self, waypoint_reached):
        """处理后退飞行任务"""
        if waypoint_reached:
            log("到达后退航点 {}".format(self.current_waypoint_index))
            
            if self.current_waypoint_index >= TASK_TRANSITION_INDICES["BACKWARD_COMPLETE"] - 1:
                log("后退飞行完成，转换到降落")
                self.state = TaskState.LANDING
                display_led_pattern("LANDING", 0.5)
            
            self._move_to_next_waypoint()
        
        return True
    
    def _handle_landing(self, waypoint_reached):
        """处理降落任务"""
        # 切换到降落模式
        if self.fcm.flight_mode != "LAND":
            self.fcm.set_flight_mode("LAND")
        
        # 监控降落进度
        height = self.perception.get_height()
        if height < 0.2 or waypoint_reached:
            log("降落完成，当前高度: {:.2f}米".format(height))
            self.fcm.motors_off()
            self.mission_complete_time = time.time()
            self.state = TaskState.MISSION_COMPLETE
            display_led_pattern("COMPLETE")
        
        return True
    
    def _handle_mission_complete(self):
        """处理任务完成状态"""
        # 检查是否需要等待
        if not self.mission_complete_time:
            self.mission_complete_time = time.time()
            log("任务完成，等待发送通信")
            return True
        
        elapsed_time = time.time() - self.mission_complete_time
        if elapsed_time < MISSION_COMPLETE_WAIT_TIME:
            # 仍在等待
            return True
        
        # 发送任务完成消息
        message = format_mission_complete_message()
        log("任务完成消息: " + message)
        
        # 通过HC12发送消息
        if self.hc12.initialized:
            if self.hc12.send_message(message):
                log("任务完成消息已通过HC12成功发送")
            else:
                log("无法通过HC12发送任务完成消息", "错误")
        else:
            log("HC12接口未初始化，无法发送消息", "错误")
        
        log("任务完成")
        return False  # 任务完成，退出主循环