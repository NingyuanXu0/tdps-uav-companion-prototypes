"""
任务管理器模块 - 起飞降落测试版本
实现起飞、悬停和降落的状态机
包含安全检查和错误处理
"""
import time
import machine
from config import (
    TAKEOFF_HEIGHT, LANDING_HEIGHT, TEST_HOVER_TIME,
    HEIGHT_THRESHOLD, ABORT_BUTTON_PIN, MAX_ALTITUDE,
    LANDING_SPEED, HEIGHT_CHECK_STABLE_COUNT,
    EMERGENCY_LAND_HEIGHT, ERROR_CODES
)
from utils import log, display_led_pattern, format_time

class TaskState:
    """任务状态枚举类"""
    INITIALIZING = 0
    WAITING_FOR_START = 1
    TAKEOFF = 2
    HOVERING = 3
    LANDING = 4
    TEST_COMPLETE = 5
    ERROR = 6
    ABORT = 7
    EMERGENCY_LANDING = 8
    
    # 状态名称映射
    _names = {
        0: "INITIALIZING",
        1: "WAITING_FOR_START", 
        2: "TAKEOFF",
        3: "HOVERING",
        4: "LANDING",
        5: "TEST_COMPLETE",
        6: "ERROR",
        7: "ABORT",
        8: "EMERGENCY_LANDING"
    }
    
    @classmethod
    def get_name(cls, state):
        """获取状态名称"""
        return cls._names.get(state, "UNKNOWN")

class TaskManager:
    """
    任务管理器 - 起飞降落测试版本
    管理测试流程和状态转换
    """
    
    def __init__(self, fcm_interface, perception_system):
        """
        初始化任务管理器
        
        参数:
            fcm_interface: 飞控接口实例
            perception_system: 感知系统实例
        """
        self.fcm = fcm_interface
        self.perception = perception_system
        
        # 状态管理
        self.state = TaskState.INITIALIZING
        self.previous_state = None
        self.state_start_time = time.time()
        self.state_timeout = {
            TaskState.TAKEOFF: 30.0,      # 起飞超时30秒
            TaskState.HOVERING: TEST_HOVER_TIME + 5.0,  # 悬停时间+余量
            TaskState.LANDING: 20.0,      # 降落超时20秒
            TaskState.EMERGENCY_LANDING: 15.0  # 紧急降落15秒
        }
        
        # 时间记录
        self.test_start_time = None
        self.hover_start_time = None
        self.landing_start_time = None
        self.test_complete_time = None
        
        # 硬件接口
        self.abort_button = machine.Pin(ABORT_BUTTON_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
        
        # 错误管理
        self.error_code = ERROR_CODES["NONE"]
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5
        
        # 测试参数
        self.target_height = TAKEOFF_HEIGHT
        self.hover_time = TEST_HOVER_TIME
        
        # 飞行数据记录
        self.flight_data = {
            "起飞开始时间": None,
            "到达目标高度时间": None,
            "悬停开始时间": None,
            "降落开始时间": None,
            "降落完成时间": None,
            "最大高度": 0.0,
            "最小高度": float('inf'),
            "平均悬停高度": 0.0,
            "悬停高度读数": [],
            "高度稳定性": [],
            "状态转换记录": []
        }
        
        # 安全参数
        self.last_known_height = 0.0
        self.height_update_time = time.time()
        self.max_height_age = 2.0  # 高度数据最大年龄（秒）
    
    def initialize(self):
        """初始化任务管理器"""
        log("=== 初始化任务管理器 ===")
        display_led_pattern("STARTUP", 3)
        
        # 记录初始状态
        self._record_state_transition(TaskState.INITIALIZING)
        
        # 检查飞控接口
        if not self.fcm or not self.fcm.initialized:
            log("飞控接口未就绪", "ERROR")
            self.error_code = ERROR_CODES["FCM_INIT_FAILED"]
            self.state = TaskState.ERROR
            return False
        
        # 初始化感知系统
        if not self.perception.initialize_sensors():
            log("感知系统初始化失败", "ERROR")
            self.error_code = ERROR_CODES["SENSOR_INIT_FAILED"]
            self.state = TaskState.ERROR
            return False
        
        # 传感器校准
        log("执行传感器校准...")
        try:
            if self.perception.calibrate_sensors():
                log("传感器校准成功")
            else:
                log("传感器校准失败，使用默认值", "WARNING")
        except Exception as e:
            log(f"传感器校准异常: {e}", "ERROR")
            # 继续运行，使用默认值
        
        # 设置初始飞行模式
        self.fcm.set_flight_mode("STABILIZE")
        
        # 转换到等待状态
        self._transition_state(TaskState.WAITING_FOR_START)
        log("任务管理器初始化完成")
        return True
    
    def start_test(self):
        """开始测试序列"""
        if self.state != TaskState.WAITING_FOR_START:
            log(f"无法从状态 {TaskState.get_name(self.state)} 开始测试", "ERROR")
            return False
        
        log("=== 开始起飞降落测试 ===")
        self.test_start_time = time.time()
        self.flight_data["起飞开始时间"] = self.test_start_time
        
        # 解锁电机
        log("解锁电机...")
        if not self.fcm.motors_on():
            log("电机解锁失败", "ERROR")
            self.error_code = ERROR_CODES["FCM_INIT_FAILED"]
            self._transition_state(TaskState.ERROR)
            return False
        
        time.sleep(1.0)  # 等待电机稳定
        
        # 设置飞行模式
        log("设置POSITION飞行模式...")
        self.fcm.set_flight_mode("POSITION")
        time.sleep(0.5)
        
        # 执行起飞命令
        log(f"发送起飞命令，目标高度: {self.target_height:.2f}米")
        self.fcm.takeoff(self.target_height)
        
        # 转换到起飞状态
        self._transition_state(TaskState.TAKEOFF)
        display_led_pattern("TAKEOFF", 0.5)
        
        return True
    
    def update(self):
        """
        更新任务状态机
        
        返回:
            是否继续运行
        """
        # 获取当前位置和高度
        try:
            current_position = self.perception.update()
            current_height = self.perception.get_height()
            
            if current_height is not None:
                self.last_known_height = current_height
                self.height_update_time = time.time()
                
                # 更新飞行数据
                if current_height > self.flight_data["最大高度"]:
                    self.flight_data["最大高度"] = current_height
                if current_height < self.flight_data["最小高度"]:
                    self.flight_data["最小高度"] = current_height
                
                # 安全检查
                if current_height > MAX_ALTITUDE:
                    log(f"高度超限！当前: {current_height:.2f}米，最大: {MAX_ALTITUDE:.2f}米", "WARNING")
                    self._transition_state(TaskState.EMERGENCY_LANDING)
                
                # 重置错误计数
                self.consecutive_errors = 0
            else:
                # 检查高度数据年龄
                height_age = time.time() - self.height_update_time
                if height_age > self.max_height_age:
                    log(f"高度数据过旧（{height_age:.1f}秒），使用最后已知高度", "WARNING")
                
                current_height = self.last_known_height
                self.consecutive_errors += 1
                
        except Exception as e:
            log(f"位置更新异常: {e}", "ERROR")
            self.consecutive_errors += 1
            current_height = self.last_known_height
        
        # 检查连续错误
        if self.consecutive_errors >= self.max_consecutive_errors:
            log("连续错误过多，进入紧急降落", "ERROR")
            self.error_code = ERROR_CODES["HEIGHT_SENSOR_FAILED"]
            self._transition_state(TaskState.EMERGENCY_LANDING)
        
        # 检查状态超时
        self._check_state_timeout()
        
        # 状态处理
        if self.state == TaskState.WAITING_FOR_START:
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
            return self._handle_error()
            
        elif self.state == TaskState.ABORT:
            return self._handle_abort()
            
        elif self.state == TaskState.EMERGENCY_LANDING:
            return self._handle_emergency_landing(current_height)
        
        return True
    
    def abort_test(self):
        """中止测试"""
        log("=== 测试中止请求 ===", "WARNING")
        self.error_code = ERROR_CODES["EMERGENCY_STOP"]
        self._transition_state(TaskState.ABORT)
    
    def emergency_landing(self):
        """执行紧急降落"""
        log("=== 执行紧急降落 ===", "WARNING")
        self._transition_state(TaskState.EMERGENCY_LANDING)
        
        try:
            # 切换到降落模式
            self.fcm.set_flight_mode("LAND")
            display_led_pattern("ERROR", 0.2)
            
            # 等待降落完成
            start_time = time.time()
            while time.time() - start_time < 15:  # 最多等待15秒
                try:
                    height = self.perception.get_height()
                    if height and height < EMERGENCY_LAND_HEIGHT:
                        log(f"紧急降落接近地面，高度: {height:.2f}米")
                        break
                except:
                    pass
                time.sleep(0.5)
            
            # 锁定电机
            self.fcm.motors_off()
            log("紧急降落完成，电机已锁定")
            
        except Exception as e:
            log(f"紧急降落异常: {e}", "ERROR")
            # 最后的安全措施
            try:
                self.fcm.emergency_stop()
            except:
                pass
    
    def _transition_state(self, new_state):
        """状态转换"""
        if self.state != new_state:
            self.previous_state = self.state
            self.state = new_state
            self.state_start_time = time.time()
            
            log(f"状态转换: {TaskState.get_name(self.previous_state)} → {TaskState.get_name(new_state)}")
            self._record_state_transition(new_state)
            if new_state == TaskState.EMERGENCY_LANDING:
                self.fcm.land()
    
    def _record_state_transition(self, state):
        """记录状态转换"""
        self.flight_data["状态转换记录"].append({
            "状态": TaskState.get_name(state),
            "时间": time.time(),
            "时间戳": format_time()
        })
    
    def _check_state_timeout(self):
        """检查状态超时"""
        if self.state in self.state_timeout:
            elapsed = time.time() - self.state_start_time
            timeout = self.state_timeout[self.state]
            
            if elapsed > timeout:
                log(f"状态 {TaskState.get_name(self.state)} 超时（{elapsed:.1f}秒）", "ERROR")
                
                if self.state == TaskState.TAKEOFF:
                    self.error_code = ERROR_CODES["TAKEOFF_TIMEOUT"]
                elif self.state == TaskState.LANDING:
                    self.error_code = ERROR_CODES["LANDING_TIMEOUT"]
                
                self._transition_state(TaskState.EMERGENCY_LANDING)
    
    def _handle_takeoff(self, current_height):
        """处理起飞状态"""
        display_led_pattern("TAKEOFF", 0.5)
        
        if current_height is None:
            log("无法获取高度数据", "WARNING")
            return True
        
        # 记录高度数据
        elapsed = time.time() - self.state_start_time
        log(f"起飞中 - 高度: {current_height:.2f}米, 目标: {self.target_height:.2f}米, 时间: {elapsed:.1f}秒")
        
        # 检查是否达到目标高度
        if self.perception.is_at_target_height(self.target_height):
            log(f"✓ 达到目标高度 {self.target_height:.2f}米")
            self.flight_data["到达目标高度时间"] = time.time()
            
            # 转换到悬停状态
            self._transition_state(TaskState.HOVERING)
            self.hover_start_time = time.time()
            self.flight_data["悬停开始时间"] = self.hover_start_time
            display_led_pattern("HOVERING", 0.5)
        
        return True
    
    def _handle_hovering(self, current_height):
        """处理悬停状态"""
        display_led_pattern("HOVERING", 0.5)
        
        if current_height is not None:
            # 记录悬停高度
            self.flight_data["悬停高度读数"].append(current_height)
            
            # 检查高度稳定性
            height_error = abs(current_height - self.target_height)
            self.flight_data["高度稳定性"].append(height_error)
        
        # 计算悬停时间
        elapsed_time = time.time() - self.hover_start_time
        remaining_time = self.hover_time - elapsed_time
        
        if remaining_time > 0:
            if int(remaining_time) != int(remaining_time + 1):  # 每秒记录一次
                log(f"悬停中 - 高度: {current_height:.2f}米, 剩余: {remaining_time:.1f}秒")
        else:
            # 计算悬停统计
            if self.flight_data["悬停高度读数"]:
                readings = self.flight_data["悬停高度读数"]
                avg_height = sum(readings) / len(readings)
                self.flight_data["平均悬停高度"] = avg_height
                
                # 计算稳定性
                if self.flight_data["高度稳定性"]:
                    avg_error = sum(self.flight_data["高度稳定性"]) / len(self.flight_data["高度稳定性"])
                    max_error = max(self.flight_data["高度稳定性"])
                    log(f"悬停统计 - 平均高度: {avg_height:.3f}米, 平均误差: {avg_error:.3f}米, 最大误差: {max_error:.3f}米")
            
            log("悬停时间结束，开始降落")
            
            # 转换到降落状态
            self._transition_state(TaskState.LANDING)
            self.landing_start_time = time.time()
            self.flight_data["降落开始时间"] = self.landing_start_time
            
            # 发送降落命令
            self.fcm.land()
            display_led_pattern("LANDING", 0.5)
        
        return True
    
    def _handle_landing(self, current_height):
        """处理降落状态"""
        display_led_pattern("LANDING", 0.5)
        
        if current_height is None:
            log("无法获取高度数据", "WARNING")
            return True
        
        # 记录降落进度
        elapsed = time.time() - self.landing_start_time
        estimated_speed = (self.target_height - current_height) / max(elapsed, 0.1)
        
        log(f"降落中 - 高度: {current_height:.2f}米, 速度: {estimated_speed:.2f}米/秒, 时间: {elapsed:.1f}秒")
        
        # 检查是否着陆
        if current_height < LANDING_HEIGHT + 0.05:
            log(f"✓ 降落完成，高度: {current_height:.2f}米")
            
            # 锁定电机
            self.fcm.motors_off()
            
            # 记录完成时间
            self.test_complete_time = time.time()
            self.flight_data["降落完成时间"] = self.test_complete_time
            
            # 转换到完成状态
            self._transition_state(TaskState.TEST_COMPLETE)
            display_led_pattern("TEST_COMPLETE", 5)
        
        return True
    
    def _handle_test_complete(self):
        """处理测试完成状态"""
        display_led_pattern("TEST_COMPLETE")
        
        # 输出测试总结
        if hasattr(self, '_summary_printed'):
            return False
        
        self._print_flight_summary()
        self._summary_printed = True
        
        return False  # 测试完成，退出主循环
    
    def _handle_error(self):
        """处理错误状态"""
        log(f"系统错误 - 错误代码: {self.error_code}", "ERROR")
        display_led_pattern("ERROR")
        
        # 尝试安全措施
        try:
            self.fcm.motors_off()
        except:
            pass
        
        return False
    
    def _handle_abort(self):
        """处理中止状态"""
        display_led_pattern("ERROR", 0.2)
        
        # 执行紧急降落
        self.emergency_landing()
        
        log("测试已中止")
        return False
    
    def _handle_emergency_landing(self, current_height):
        """处理紧急降落状态"""
        display_led_pattern("ERROR", 0.1)
        
        if current_height is not None:
            log(f"紧急降落 - 高度: {current_height:.2f}米")
            
            if current_height < EMERGENCY_LAND_HEIGHT:
                # 锁定电机
                self.fcm.motors_off()
                log("紧急降落完成")
                self._transition_state(TaskState.ERROR)
        
        return True
    
    def _print_flight_summary(self):
        """打印飞行数据总结"""
        log("\n" + "="*50)
        log("          飞行测试总结")
        log("="*50)
        
        # 时间统计
        if self.test_start_time and self.test_complete_time:
            total_time = self.test_complete_time - self.test_start_time
            log(f"总飞行时间: {total_time:.1f}秒")
        
        if self.flight_data["起飞开始时间"] and self.flight_data["到达目标高度时间"]:
            takeoff_time = self.flight_data["到达目标高度时间"] - self.flight_data["起飞开始时间"]
            log(f"起飞时间: {takeoff_time:.1f}秒")
        
        if self.flight_data["降落开始时间"] and self.flight_data["降落完成时间"]:
            landing_time = self.flight_data["降落完成时间"] - self.flight_data["降落开始时间"]
            log(f"降落时间: {landing_time:.1f}秒")
        
        # 高度统计
        log(f"\n高度统计:")
        log(f"  最大高度: {self.flight_data['最大高度']:.3f}米")
        if self.flight_data['最小高度'] != float('inf'):
            log(f"  最小高度: {self.flight_data['最小高度']:.3f}米")
        log(f"  平均悬停高度: {self.flight_data['平均悬停高度']:.3f}米")
        log(f"  目标高度: {self.target_height:.3f}米")
        
        # 传感器状态
        sensor_status = self.perception.get_sensor_status()
        log(f"\n传感器统计:")
        log(f"  总读数: {sensor_status['total_readings']}")
        log(f"  失败读数: {sensor_status['failed_readings']}")
        log(f"  失败率: {sensor_status['failure_rate']*100:.1f}%")
        
        # 状态转换记录
        log(f"\n状态转换记录:")
        for transition in self.flight_data["状态转换记录"][-5:]:  # 显示最后5个
            log(f"  {transition['时间戳']} - {transition['状态']}")
        
        log("="*50 + "\n")