"""
任务管理器模块 - 起飞降落测试版本
仅实现起飞、悬停和降落的状态机
移除电池监控功能
"""
import time
import machine
from config import (
    TAKEOFF_HEIGHT, LANDING_HEIGHT, TEST_HOVER_TIME,
    HEIGHT_THRESHOLD
)
from utils import log, display_led_pattern

class TaskState:
    """任务状态类 - 仅包含起飞降落测试状态"""
    INITIALIZING = 0
    WAITING_FOR_START = 1
    TAKEOFF = 2
    HOVERING = 3
    LANDING = 4
    TEST_COMPLETE = 5
    ERROR = 6
    ABORT = 7
    
    # 状态名称映射（用于调试）
    _names = {
        0: "INITIALIZING",
        1: "WAITING_FOR_START",
        2: "TAKEOFF",
        3: "HOVERING",
        4: "LANDING",
        5: "TEST_COMPLETE",
        6: "ERROR",
        7: "ABORT"
    }
    
    @classmethod
    def get_name(cls, state):
        """获取状态名称"""
        return cls._names.get(state, "UNKNOWN")

class TaskManager:
    """
    任务管理器 - 起飞降落测试版本
    实现简单的起飞、悬停、降落状态机
    """
    
    def __init__(self, fcm_interface, perception_system):
        """
        初始化任务管理器
        
        参数:
            fcm_interface: FCMInterface实例，用于与飞控通信
            perception_system: PerceptionSystem实例，用于传感器数据
        """
        self.fcm = fcm_interface
        self.perception = perception_system
        
        self.state = TaskState.INITIALIZING
        self.test_start_time = None
        self.hover_start_time = None
        self.test_complete_time = None
        
        # 添加中止按钮
        self.abort_button = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
        
        # 错误跟踪
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # 测试参数
        self.target_height = TAKEOFF_HEIGHT
        self.hover_time = TEST_HOVER_TIME
        
        # 飞行数据记录
        self.flight_data = {
            "起飞开始时间": None,
            "到达目标高度时间": None,
            "降落开始时间": None,
            "降落完成时间": None,
            "最大高度": 0.0,
            "平均悬停高度": 0.0,
            "悬停高度读数": []
        }
    
    def initialize(self):
        """初始化任务管理器并准备测试"""
        log("初始化任务管理器")
        display_led_pattern("STARTUP", 3)
        
        # 确保FCM接口已初始化
        if not self.fcm.initialized:
            log("FCM接口未初始化", "ERROR")
            self.state = TaskState.ERROR
            return False
            
        # 初始化传感器
        if not self.perception.initialize_sensors():
            log("传感器初始化失败", "ERROR")
            self.state = TaskState.ERROR
            return False
        
        # 校准传感器
        log("校准传感器...")
        try:
            self.perception.calibrate_sensors()
            log("传感器校准完成")
        except Exception as e:
            log(f"传感器校准失败: {e}", "ERROR")
            self.state = TaskState.ERROR
            return False
        
        self.state = TaskState.WAITING_FOR_START
        log("任务管理器初始化完成，准备开始测试")
        return True
    
    def update(self):
        """
        更新任务管理器状态
        
        返回:
            True如果测试进行中，False如果完成或错误
        """
        # 获取当前位置
        try:
            current_position = self.perception.get_current_position()
            current_height = self.perception.get_height()
            
            # 更新最大高度记录
            if current_height and current_height > self.flight_data["最大高度"]:
                self.flight_data["最大高度"] = current_height
            
            # 重置连续错误计数
            self.consecutive_errors = 0
            
        except Exception as e:
            log(f"获取位置数据时出错: {e}", "ERROR")
            self.consecutive_errors += 1
            
            if self.consecutive_errors >= self.max_consecutive_errors:
                log("连续错误次数过多，切换到错误状态", "ERROR")
                self.state = TaskState.ERROR
                return False
            
            # 尝试使用最后已知位置
            if not hasattr(self, 'last_known_height'):
                log("无位置数据可用，无法继续", "ERROR")
                self.state = TaskState.ERROR
                return False
                
            current_height = self.last_known_height
            log("使用最后已知高度作为后备", "WARNING")
        
        # 存储当前高度供将来使用
        self.last_known_height = current_height
        
        # 打印当前状态（调试用）
        log(f"当前状态: {TaskState.get_name(self.state)}")
        
        # 根据当前状态更新
        if self.state == TaskState.WAITING_FOR_START:
            # 等待开始信号
            return True
            
        elif self.state == TaskState.TAKEOFF:
            return self._handle_takeoff(current_height)
            
        elif self.state == TaskState.HOVERING:
            return self._handle_hovering(current_height)
            
        elif self.state == TaskState.LANDING:
            return self._handle_landing(current_height)
            
        elif self.state == TaskState.TEST_COMPLETE:
            return self._handle_test_complete()
            
        elif self.state == TaskState.ERROR:
            log("系统处于错误状态", "ERROR")
            self.fcm.emergency_stop()
            display_led_pattern("ERROR")
            return False
            
        elif self.state == TaskState.ABORT:
            log("系统处于中止状态，执行紧急降落", "WARNING")
            return self._handle_abort()
        
        return True
    
    def start_test(self):
        """开始起飞降落测试"""
        if self.state != TaskState.WAITING_FOR_START:
            log("无法开始测试，当前状态不是WAITING_FOR_START", "ERROR")
            return False
        
        log("开始起飞降落测试")
        self.state = TaskState.TAKEOFF
        self.test_start_time = time.time()
        self.flight_data["起飞开始时间"] = time.time()
        
        # 初始化电机并设置飞行模式
        self.fcm.motors_on()
        time.sleep(0.5)
        self.fcm.set_flight_mode("POSITION")
        
        # 设置起飞目标
        self.fcm.takeoff(self.target_height)
        
        display_led_pattern("TAKEOFF", 0.5)
        
        log(f"起飞目标高度: {self.target_height:.2f}米")
        return True
    
    def abort_test(self):
        """中止测试并执行紧急降落"""
        log("测试中止请求", "WARNING")
        self.state = TaskState.ABORT
    
    def emergency_landing(self):
        """立即执行紧急降落"""
        log("紧急降落启动", "WARNING")
        
        try:
            # 切换到降落模式
            self.fcm.set_flight_mode("LAND")
            
            # 等待降落
            start_time = time.time()
            while time.time() - start_time < 10:  # 最多等待10秒
                try:
                    height = self.perception.get_height()
                    if height and height < 0.3:  # 足够接近地面
                        break
                    time.sleep(0.5)
                except:
                    pass
                
            # 停止电机
            self.fcm.motors_off()
            log("紧急降落完成")
            
        except Exception as e:
            log(f"紧急降落时出错: {e}", "ERROR")
            # 最后手段 - 停止电机
            self.fcm.emergency_stop()
    
    def _handle_takeoff(self, current_height):
        """处理起飞状态"""
        display_led_pattern("TAKEOFF", 0.5)
        
        if current_height is None:
            log("无法获取高度数据", "WARNING")
            return True
        
        log(f"当前高度: {current_height:.2f}米, 目标: {self.target_height:.2f}米")
        
        # 检查是否达到目标高度
        if self.perception.is_at_target_height(self.target_height):
            log(f"达到目标高度 {self.target_height:.2f}米，开始悬停")
            self.state = TaskState.HOVERING
            self.hover_start_time = time.time()
            self.flight_data["到达目标高度时间"] = time.time()
            display_led_pattern("HOVERING", 0.5)
        
        return True
    
    def _handle_hovering(self, current_height):
        """处理悬停状态"""
        display_led_pattern("HOVERING", 0.5)
        
        if current_height is not None:
            log(f"悬停高度: {current_height:.2f}米")
            self.flight_data["悬停高度读数"].append(current_height)
        
        # 检查悬停时间
        elapsed_time = time.time() - self.hover_start_time
        remaining_time = self.hover_time - elapsed_time
        
        if remaining_time > 0:
            log(f"悬停剩余时间: {remaining_time:.1f}秒")
        else:
            # 计算平均悬停高度
            if self.flight_data["悬停高度读数"]:
                avg_height = sum(self.flight_data["悬停高度读数"]) / len(self.flight_data["悬停高度读数"])
                self.flight_data["平均悬停高度"] = avg_height
                log(f"平均悬停高度: {avg_height:.2f}米")
            
            log(f"悬停时间结束，开始降落")
            self.state = TaskState.LANDING
            self.flight_data["降落开始时间"] = time.time()
            
            # 设置降落
            self.fcm.land()
            display_led_pattern("LANDING", 0.5)
        
        return True
    
    def _handle_landing(self, current_height):
        """处理降落状态"""
        display_led_pattern("LANDING", 0.5)
        
        if current_height is None:
            log("无法获取高度数据", "WARNING")
            return True
        
        log(f"降落中，当前高度: {current_height:.2f}米")
        
        # 检查是否已经着陆
        if current_height < LANDING_HEIGHT + 0.05:
            log(f"降落完成，当前高度: {current_height:.2f}米")
            self.fcm.motors_off()
            self.test_complete_time = time.time()
            self.flight_data["降落完成时间"] = time.time()
            self.state = TaskState.TEST_COMPLETE
            display_led_pattern("TEST_COMPLETE", 5)
        
        return True
    
    def _handle_test_complete(self):
        """处理测试完成状态"""
        display_led_pattern("TEST_COMPLETE")
        
        # 计算测试总时间
        if self.test_start_time and self.test_complete_time:
            test_duration = self.test_complete_time - self.test_start_time
            log(f"测试完成！总时长: {test_duration:.1f}秒")
        else:
            log("测试完成！")
        
        # 输出飞行数据总结
        log("\n=== 飞行数据总结 ===")
        log(f"起飞时间: {self.flight_data['到达目标高度时间'] - self.flight_data['起飞开始时间']:.1f}秒")
        log(f"降落时间: {self.flight_data['降落完成时间'] - self.flight_data['降落开始时间']:.1f}秒")
        log(f"最大高度: {self.flight_data['最大高度']:.2f}米")
        log(f"平均悬停高度: {self.flight_data['平均悬停高度']:.2f}米")
        log("==================\n")
        
        return False  # 测试完成，退出主循环
    
    def _handle_abort(self):
        """处理中止状态"""
        display_led_pattern("ERROR", 0.2)
        
        # 执行紧急降落
        self.emergency_landing()
        
        log("测试已中止")
        return False  # 退出主循环
