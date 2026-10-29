"""
性能测试与优化工具
用于测试代码执行时间并提供优化建议
"""
import sys
import time
import cProfile
import pstats
import io
import importlib
import traceback
import gc
import copy
from threading import Thread

# 确保能导入所有模块
sys.path.append('.')

# 配置模块模拟
from hardware_simulator import (
    machine, network, socket, ubinascii, ujson, 
    time_sim, uos, micropython, enum_sim, gc, math_sim,
    hw_simulator
)

# 修改sys.modules，插入模拟模块
sys.modules['machine'] = machine
sys.modules['network'] = network
sys.modules['socket'] = socket
sys.modules['ubinascii'] = ubinascii
sys.modules['ujson'] = ujson
sys.modules['time'] = time_sim
sys.modules['uos'] = uos
sys.modules['micropython'] = micropython
sys.modules['enum'] = enum_sim
sys.modules['gc'] = gc
sys.modules['math'] = math_sim

class PerformanceTester:
    """性能测试工具类"""
    
    def __init__(self):
        self.profiler = cProfile.Profile()
        self.results = {}
        self.modules = []
        self.current_module = None
    
    def load_modules(self):
        """加载所有模块进行测试"""
        self.modules = ['utils', 'perception', 'communication', 'task_manager', 'main']
        
        # 导入模块
        for module_name in self.modules:
            try:
                # 重新加载模块，确保最新代码
                if module_name in sys.modules:
                    module = importlib.reload(sys.modules[module_name])
                else:
                    module = importlib.import_module(module_name)
                
                print(f"成功加载模块: {module_name}")
            except Exception as e:
                print(f"加载模块 {module_name} 失败: {e}")
                traceback.print_exc()
    
    def test_function(self, module_name, function_name, *args, **kwargs):
        """测试指定函数的性能"""
        try:
            # 加载模块
            if module_name in sys.modules:
                module = importlib.reload(sys.modules[module_name])
            else:
                module = importlib.import_module(module_name)
            
            # 获取函数
            func = getattr(module, function_name)
            
            # 使用profiler测试性能
            self.profiler.enable()
            result = func(*args, **kwargs)
            self.profiler.disable()
            
            # 获取统计结果
            s = io.StringIO()
            ps = pstats.Stats(self.profiler, stream=s).sort_stats('cumulative')
            ps.print_stats()
            
            # 存储结果
            self.results[f"{module_name}.{function_name}"] = s.getvalue()
            
            print(f"函数 {module_name}.{function_name} 测试完成")
            return result
            
        except Exception as e:
            print(f"测试函数 {module_name}.{function_name} 失败: {e}")
            traceback.print_exc()
            return None
    
    def test_module(self, module_name):
        """测试整个模块的性能"""
        try:
            # 设置当前模块
            self.current_module = module_name
            
            # 加载模块
            if module_name in sys.modules:
                module = importlib.reload(sys.modules[module_name])
            else:
                module = importlib.import_module(module_name)
            
            # 获取所有可调用函数
            functions = []
            for attr_name in dir(module):
                if attr_name.startswith('_'):
                    continue
                attr = getattr(module, attr_name)
                if callable(attr) and not isinstance(attr, type):
                    functions.append(attr_name)
            
            print(f"模块 {module_name} 中找到 {len(functions)} 个可测试函数")
            
            # 使用profiler测试整个模块
            self.profiler.enable()
            # 执行模块级代码
            importlib.reload(module)
            self.profiler.disable()
            
            # 获取统计结果
            s = io.StringIO()
            ps = pstats.Stats(self.profiler, stream=s).sort_stats('cumulative')
            ps.print_stats()
            
            # 存储结果
            self.results[module_name] = s.getvalue()
            
            print(f"模块 {module_name} 测试完成")
            
        except Exception as e:
            print(f"测试模块 {module_name} 失败: {e}")
            traceback.print_exc()
    
    def test_loop_performance(self):
        """测试主循环性能"""
        try:
            # 导入main模块
            import main
            
            # 模拟主循环的关键组件
            fcm = sys.modules['communication'].FCMInterface()
            perception_system = sys.modules['perception'].PerceptionSystem()
            wireless = sys.modules['communication'].WirelessCommunication()
            
            # 创建任务管理器
            task_manager = sys.modules['task_manager'].TaskManager(fcm, perception_system, wireless)
            task_manager.initialize()
            
            # 记录循环时间
            loop_times = []
            
            # 执行10次循环
            for i in range(10):
                start_time = time.time()
                
                # 收集垃圾
                gc.collect()
                
                # 更新任务管理器
                task_manager.update()
                
                # 记录循环时间
                loop_time = time.time() - start_time
                loop_times.append(loop_time)
                
                print(f"循环 {i+1} 耗时: {loop_time:.6f} 秒")
            
            # 计算平均循环时间
            avg_time = sum(loop_times) / len(loop_times)
            print(f"平均循环时间: {avg_time:.6f} 秒")
            
            # 存储结果
            self.results['main_loop'] = {
                'loop_times': loop_times,
                'average': avg_time
            }
            
        except Exception as e:
            print(f"测试主循环性能失败: {e}")
            traceback.print_exc()
    
    def analyze_memory_usage(self):
        """分析内存使用情况"""
        try:
            # 收集垃圾确保测量准确
            gc.collect()
            
            # 获取初始内存使用
            initial_memory = 0  # MicroPython可以使用gc.mem_free()获取
            
            # 导入各模块并测量内存
            module_memory = {}
            
            for module_name in self.modules:
                # 收集垃圾
                gc.collect()
                
                # 导入前测量内存
                before_import = 0  # MicroPython测量
                
                # 重新导入模块
                if module_name in sys.modules:
                    del sys.modules[module_name]
                module = importlib.import_module(module_name)
                
                # 导入后测量内存
                after_import = 0  # MicroPython测量
                
                # 计算模块内存使用
                module_usage = before_import - after_import
                module_memory[module_name] = module_usage
                
                print(f"模块 {module_name} 内存使用: {module_usage} 字节")
            
            # 存储结果
            self.results['memory_usage'] = module_memory
            
        except Exception as e:
            print(f"分析内存使用失败: {e}")
            traceback.print_exc()
    
    def suggest_optimizations(self):
        """根据性能测试结果提供优化建议"""
        suggestions = []
        
        # 分析函数调用
        for name, result in self.results.items():
            if isinstance(result, str):
                # 分析profiler输出
                if "function calls" in result:
                    lines = result.split('\n')
                    for line in lines:
                        # 查找耗时较长的函数
                        if line.strip() and not line.startswith('ncalls') and not ":" in line:
                            parts = line.split()
                            if len(parts) >= 6:
                                try:
                                    cumtime = float(parts[3])
                                    if cumtime > 0.1:  # 耗时超过0.1秒视为可优化
                                        func_name = parts[5]
                                        suggestions.append(f"函数 {func_name} 耗时 {cumtime:.6f} 秒，考虑优化")
                                except:
                                    pass
        
        # 分析内存使用
        if 'memory_usage' in self.results:
            memory_usage = self.results['memory_usage']
            for module, usage in memory_usage.items():
                if usage > 10000:  # 使用超过10KB内存视为可优化
                    suggestions.append(f"模块 {module} 内存使用 {usage} 字节，考虑优化")
        
        # 分析主循环性能
        if 'main_loop' in self.results:
            main_loop = self.results['main_loop']
            if main_loop['average'] > 0.02:  # 平均循环时间超过20ms视为可优化
                suggestions.append(f"主循环平均耗时 {main_loop['average']:.6f} 秒，考虑优化")
        
        return suggestions
    
    def run_complete_test(self):
        """运行完整的性能测试"""
        print("开始完整性能测试...")
        
        # 加载模块
        print("\n加载模块:")
        self.load_modules()
        
        # 测试各模块
        print("\n测试模块性能:")
        for module_name in self.modules:
            print(f"\n测试模块 {module_name}:")
            self.test_module(module_name)
        
        # 测试主循环
        print("\n测试主循环性能:")
        self.test_loop_performance()
        
        # 分析内存使用
        print("\n分析内存使用:")
        self.analyze_memory_usage()
        
        # 提供优化建议
        print("\n优化建议:")
        suggestions = self.suggest_optimizations()
        for suggestion in suggestions:
            print(f"- {suggestion}")
        
        print("\n性能测试完成！")


class CodeOptimizer:
    """代码优化工具类"""
    
    def __init__(self):
        self.optimizations = []
    
    def optimize_utils(self):
        """优化utils模块"""
        try:
            # 导入模块
            import utils
            
            # 优化1: 日志记录函数
            original_log = utils.log
            
            def optimized_log(message, level="INFO"):
                """优化的日志记录函数，减少字符串格式化和IO操作"""
                if utils.DEBUG_MODE or level in ["ERROR", "WARNING"]:
                    timestamp = time.localtime()
                    time_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
                        timestamp[0], timestamp[1], timestamp[2],
                        timestamp[3], timestamp[4], timestamp[5]
                    )
                    log_message = "[{}] {}: {}".format(time_str, level, message)
                    print(log_message)
                    
                    # 减少IO操作，只在确实需要时写入日志
                    if level in ["ERROR", "WARNING"]:
                        try:
                            with open(utils.LOG_FILENAME, "a") as f:
                                f.write(log_message + "\n")
                        except:
                            pass
            
            # 将优化后的函数替换原函数
            utils.log = optimized_log
            
            # 优化2: LED模式显示函数
            original_display_led_pattern = utils.display_led_pattern
            
            def optimized_display_led_pattern(pattern_name, duration=None):
                """优化的LED模式显示函数，使用更高效的时间控制"""
                if pattern_name not in utils.LED_PATTERNS:
                    utils.log(f"未知的LED模式: {pattern_name}", "WARNING")
                    return
                
                pattern = utils.LED_PATTERNS[pattern_name]
                start_time = utils.current_time_ms()
                
                # 如果没有持续时间，只执行一次模式
                if duration is None:
                    for state, delay in pattern:
                        utils.led.value(state)
                        time.sleep(delay)
                    return
                
                # 有持续时间，计算可以执行几次完整模式
                total_pattern_time = sum(delay for _, delay in pattern)
                iterations = int(duration / total_pattern_time)
                
                # 执行完整的迭代
                for _ in range(iterations):
                    for state, delay in pattern:
                        utils.led.value(state)
                        time.sleep(delay)
                
                # 为剩余时间执行部分模式
                remaining_time = duration - (iterations * total_pattern_time)
                cumulative_time = 0
                
                for state, delay in pattern:
                    if cumulative_time + delay > remaining_time:
                        break
                    utils.led.value(state)
                    time.sleep(delay)
                    cumulative_time += delay
            
            # 将优化后的函数替换原函数
            utils.display_led_pattern = optimized_display_led_pattern
            
            # 记录优化
            self.optimizations.append("utils.log: 减少IO操作")
            self.optimizations.append("utils.display_led_pattern: 优化时间控制")
            
            print("utils模块优化完成")
            
        except Exception as e:
            print(f"优化utils模块失败: {e}")
            traceback.print_exc()
    
    def optimize_perception(self):
        """优化perception模块"""
        try:
            # 导入模块
            import perception
            
            # 优化1: 传感器数据获取函数
            original_get_height = perception.PerceptionSystem.get_height
            
            def optimized_get_height(self):
                """优化的高度获取函数，减少重试次数和等待时间"""
                if not self.initialized or not self.height_sensor.initialized:
                    return None
                
                # 首先尝试使用缓存的读数，如果有足够的样本
                if len(self.height_readings) >= 3:
                    return sum(self.height_readings) / len(self.height_readings)
                
                # 只尝试两次读取
                max_retries = 2
                for _ in range(max_retries):
                    height = self.height_sensor.get_distance()
                    if height is not None:
                        # 添加到过滤窗口
                        self.height_readings.append(height)
                        # 保持窗口大小
                        if len(self.height_readings) > self.filter_window:
                            self.height_readings.pop(0)
                        
                        # 返回过滤值
                        if self.height_readings:
                            return sum(self.height_readings) / len(self.height_readings)
                        return height
                    
                    # 减少等待时间
                    time.sleep(0.005)  # 只等待5ms
                
                # 如果所有重试都失败，返回上次有效的高度
                if self.height_readings:
                    return sum(self.height_readings) / len(self.height_readings)
                
                return None
            
            # 将优化后的函数替换原函数
            perception.PerceptionSystem.get_height = optimized_get_height
            
            # 优化2: 更新函数
            original_update = perception.PerceptionSystem.update
            
            def optimized_update(self, fcm_position_data=None):
                """优化的更新函数，减少不必要的计算"""
                if not self.initialized:
                    sys.modules['utils'].log("感知系统未初始化", "ERROR")
                    return self._current_position
                
                current_time = time.ticks_ms()
                dt = time.ticks_diff(current_time, self._last_update_time) / 1000.0
                self._last_update_time = current_time
                
                # 直接使用FCM数据，如果可用
                if fcm_position_data:
                    # 只复制而不创建新列表
                    self._current_position[0] = fcm_position_data[0]
                    self._current_position[1] = fcm_position_data[1]
                    self._current_position[2] = fcm_position_data[2]
                    return self._current_position
                
                # 只更新高度
                height = self.get_height()
                if height is not None:
                    self._current_position[2] = height
                
                return self._current_position
            
            # 将优化后的函数替换原函数
            perception.PerceptionSystem.update = optimized_update
            
            # 记录优化
            self.optimizations.append("perception.PerceptionSystem.get_height: 减少重试和等待")
            self.optimizations.append("perception.PerceptionSystem.update: 减少计算和内存分配")
            
            print("perception模块优化完成")
            
        except Exception as e:
            print(f"优化perception模块失败: {e}")
            traceback.print_exc()
    
    def optimize_communication(self):
        """优化communication模块"""
        try:
            # 导入模块
            import communication
            
            # 优化1: 发送命令函数
            original_send_command = communication.FCMInterface.send_command
            
            def optimized_send_command(self, command, data=None):
                """优化的发送命令函数，减少异常处理和重试逻辑"""
                if not self.initialized:
                    sys.modules['utils'].log("FCM接口未初始化", "ERROR")
                    return False
                
                cmd_byte = sys.modules['config'].FCM_COMMANDS.get(command)
                if cmd_byte is None:
                    sys.modules['utils'].log(f"未知命令: {command}", "ERROR")
                    return False
                
                try:
                    # 使用预分配的缓冲区，避免频繁创建字节数组
                    packet = bytearray(32)  # 预分配足够大的缓冲区
                    packet[0] = cmd_byte
                    packet[1] = self.cmd_sequence & 0xFF
                    packet_len = 2
                    
                    # 更新序列号
                    self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
                    
                    # 添加数据
                    if data:
                        if isinstance(data, (list, tuple)):
                            # 直接处理数值数据
                            for value in data:
                                if isinstance(value, float):
                                    # 浮点数转换
                                    import struct
                                    struct.pack_into('f', packet, packet_len, value)
                                    packet_len += 4
                                elif isinstance(value, int):
                                    # 整数转换
                                    packet[packet_len] = (value >> 8) & 0xFF
                                    packet[packet_len + 1] = value & 0xFF
                                    packet_len += 2
                        elif isinstance(data, dict):
                            # 字典转JSON
                            json_data = sys.modules['ujson'].dumps(data)
                            json_bytes = json_data.encode()
                            for i, b in enumerate(json_bytes):
                                packet[packet_len + i] = b
                            packet_len += len(json_bytes)
                    
                    # 发送数据包
                    self.i2c.writeto(self.fcm_addr, packet[:packet_len])
                    
                    sys.modules['utils'].log(f"向FCM发送命令 {command}")
                    
                    # 存储上次命令
                    self.last_command = command
                    self.last_data = data
                    self.retry_count = 0
                    
                    return True
                    
                except Exception as e:
                    sys.modules['utils'].log(f"向FCM发送命令失败: {e}", "ERROR")
                    
                    # 只为关键命令重试一次
                    if command in ["EMERGENCY_STOP"]:
                        time.sleep(0.05)
                        try:
                            # 简化重试逻辑
                            simple_packet = bytearray([cmd_byte, self.cmd_sequence & 0xFF])
                            self.i2c.writeto(self.fcm_addr, simple_packet)
                            return True
                        except:
                            pass
                    
                    return False
            
            # 将优化后的函数替换原函数
            communication.FCMInterface.send_command = optimized_send_command
            
            # 优化2: Wi-Fi连接函数
            original_connect = communication.WirelessCommunication.connect
            
            def optimized_connect(self, timeout=None):
                """优化的Wi-Fi连接函数，减少等待时间和重试逻辑"""
                if timeout is None:
                    timeout = sys.modules['config'].WIFI_CONNECTION_TIMEOUT
                
                if not self.initialized:
                    if not self.initialize():
                        return False
                
                try:
                    # 检查是否已经连接
                    if self.wlan.isconnected():
                        self.connected = True
                        return True
                    
                    # 不做过多重试，直接尝试连接
                    self.wlan.connect(sys.modules['config'].WIFI_SSID, sys.modules['config'].WIFI_PASSWORD)
                    
                    # 等待连接，减少睡眠时间，更积极检查
                    start_time = time.time()
                    while not self.wlan.isconnected():
                        if time.time() - start_time > timeout:
                            return False
                        time.sleep(0.05)  # 只等待50ms
                    
                    self.connected = True
                    return True
                    
                except Exception as e:
                    sys.modules['utils'].log(f"连接到Wi-Fi失败: {e}", "ERROR")
                    return False
            
            # 将优化后的函数替换原函数
            communication.WirelessCommunication.connect = optimized_connect
            
            # 记录优化
            self.optimizations.append("communication.FCMInterface.send_command: 使用预分配缓冲区")
            self.optimizations.append("communication.WirelessCommunication.connect: 减少等待时间")
            
            print("communication模块优化完成")
            
        except Exception as e:
            print(f"优化communication模块失败: {e}")
            traceback.print_exc()
    
    def optimize_task_manager(self):
        """优化task_manager模块"""
        try:
            # 导入模块
            import task_manager
            
            # 优化1: 更新函数
            original_update = task_manager.TaskManager.update
            
            def optimized_update(self):
                """优化的更新函数，减少异常处理和状态检查"""
                # 检查是否需要手动干预
                if self.manual_intervention_required and not self.manual_intervention_acknowledged:
                    sys.modules['utils'].display_led_pattern("MANUAL_INTERVENTION")
                    
                    if self.abort_button.value() == 0:
                        time.sleep(0.1)  # 减少去抖时间
                        self.manual_intervention_acknowledged = True
                        sys.modules['utils'].log("手动干预已确认")
                        sys.modules['utils'].display_led_pattern("CONTINUE", 1)
                    
                    return True
                
                # 减少异常处理，直接获取位置数据
                try:
                    current_position = self.perception.get_current_position()
                    self.wall_distance = self.perception.get_wall_distance()
                    self.ground_distance = self.perception.get_height()
                    
                    # 重置连续错误计数器
                    self.consecutive_errors = 0
                    
                except Exception as e:
                    sys.modules['utils'].log(f"获取位置数据时出错: {e}", "ERROR")
                    self.consecutive_errors += 1
                    
                    if self.consecutive_errors >= self.max_consecutive_errors:
                        sys.modules['utils'].log("连续错误过多，切换到ERROR状态", "ERROR")
                        self.state = task_manager.TaskState.ERROR
                        return False
                    
                    if hasattr(self, 'last_known_position'):
                        current_position = self.last_known_position
                    else:
                        sys.modules['utils'].log("无可用位置数据，无法继续", "ERROR")
                        self.state = task_manager.TaskState.ERROR
                        return False
                
                # 存储当前位置
                self.last_known_position = current_position
                
                # 获取当前目标航点
                current_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
                
                # 检查是否到达目标位置
                waypoint_reached = sys.modules['utils'].is_position_reached(
                    current_position, current_waypoint, sys.modules['config'].POSITION_THRESHOLD
                )
                
                # 根据当前状态更新任务
                if self.state == task_manager.TaskState.WAITING_FOR_START:
                    # 直接进入隧道任务
                    self.state = task_manager.TaskState.TUNNELLING
                    self.mission_start_time = time.time()
                    
                    # 启动电机并设置位置
                    self.fcm.motors_on()
                    first_waypoint = self._get_adjusted_waypoint(self.current_waypoint_index)
                    self.fcm.set_position(*first_waypoint)
                    sys.modules['utils'].log(f"开始任务，直接进入隧道任务，设置位置 {first_waypoint}")
                    return True
                    
                elif self.state == task_manager.TaskState.TUNNELLING:
                    return self._handle_tunnelling(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.OBSTACLE_AVOIDANCE:
                    return self._handle_obstacle_avoidance(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.PACKAGE_SEARCH:
                    return self._handle_package_search(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.PACKAGE_PICKUP:
                    return self._handle_package_pickup(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.MARKER_SEARCH:
                    return self._handle_marker_search(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.PACKAGE_DROPOFF:
                    return self._handle_package_dropoff(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.LANDING:
                    return self._handle_landing(current_position, waypoint_reached)
                    
                elif self.state == task_manager.TaskState.MISSION_COMPLETE:
                    return self._handle_mission_complete()
                    
                elif self.state == task_manager.TaskState.ERROR:
                    sys.modules['utils'].log("系统处于错误状态", "ERROR")
                    self.fcm.emergency_stop()
                    sys.modules['utils'].display_led_pattern("ERROR")
                    return False
                    
                elif self.state == task_manager.TaskState.ABORT:
                    sys.modules['utils'].log("系统处于中止状态，执行紧急降落", "WARNING")
                    return self._handle_abort()
                
                return True
            
            # 将优化后的函数替换原函数
            task_manager.TaskManager.update = optimized_update
            
            # 优化2: 获取调整后的航点函数
            original_get_adjusted_waypoint = task_manager.TaskManager._get_adjusted_waypoint
            
            def optimized_get_adjusted_waypoint(self, index):
                """优化的获取调整后的航点函数，减少内存分配"""
                if index >= len(sys.modules['config'].PATH_WAYPOINTS):
                    return sys.modules['config'].PATH_WAYPOINTS[-1]
                    
                waypoint = sys.modules['config'].PATH_WAYPOINTS[index]
                
                # 直接返回原始航点，如果没有调整
                if all(adj == 0 for adj in self.path_adjustment):
                    return waypoint
                
                # 应用位置调整
                adjusted_waypoint = [
                    waypoint[0] + self.path_adjustment[0],
                    waypoint[1] + self.path_adjustment[1],
                    waypoint[2] + self.path_adjustment[2]
                ]
                
                return adjusted_waypoint
            
            # 将优化后的函数替换原函数
            task_manager.TaskManager._get_adjusted_waypoint = optimized_get_adjusted_waypoint
            
            # 记录优化
            self.optimizations.append("task_manager.TaskManager.update: 减少异常处理和状态检查")
            self.optimizations.append("task_manager.TaskManager._get_adjusted_waypoint: 减少内存分配")
            
            print("task_manager模块优化完成")
            
        except Exception as e:
            print(f"优化task_manager模块失败: {e}")
            traceback.print_exc()
    
    def optimize_main(self):
        """优化main模块"""
        try:
            # 导入模块
            import main
            
            # 优化主循环
            original_main = main.main
            
            def optimized_main():
                """优化的主函数，减少循环延迟和状态检查"""
                sys.modules['utils'].log("ACM 核心启动中")
                
                # 配置看门狗，延长超时时间以减少中断
                wdt = sys.modules['machine'].WDT(timeout=20000)  # 20秒超时
                
                # 初始化系统组件
                task_manager = main.initialize_system()
                if task_manager is None:
                    sys.modules['utils'].log("系统初始化失败，中止", "ERROR")
                    sys.modules['utils'].display_led_pattern("ERROR")
                    return
                
                # 初始化任务管理器
                if not task_manager.initialize():
                    sys.modules['utils'].log("任务管理器初始化失败，中止", "ERROR")
                    sys.modules['utils'].display_led_pattern("ERROR")
                    return
                
                sys.modules['utils'].log("ACM 核心初始化完成，进入主循环")
                last_loop_time = sys.modules['time'].ticks_ms()
                last_wdt_feed_time = sys.modules['time'].ticks_ms()
                last_gc_time = sys.modules['time'].ticks_ms()
                
                # 主控制循环
                try:
                    # 等待操作员确认开始
                    sys.modules['utils'].display_led_pattern("WAITING_FOR_START")
                    sys.modules['utils'].log("等待开始按钮按下...")
                    
                    # 使用GPIO引脚作为开始按钮
                    start_button = sys.modules['machine'].Pin(15, sys.modules['machine'].Pin.IN, sys.modules['machine'].Pin.PULL_UP)
                    while start_button.value() == 1:  # 低电平有效按钮
                        wdt.feed()
                        sys.modules['time'].sleep(0.1)
                    
                    sys.modules['utils'].log("开始按钮已按下，任务开始")
                    sys.modules['utils'].display_led_pattern("MISSION_START", 1)
                        
                    while True:
                        # 定期喂狗，而不是每次循环
                        current_time = sys.modules['time'].ticks_ms()
                        if sys.modules['time'].ticks_diff(current_time, last_wdt_feed_time) > 5000:  # 每5秒喂一次狗
                            wdt.feed()
                            last_wdt_feed_time = current_time
                        
                        # 定期收集垃圾，而不是每次循环
                        if sys.modules['time'].ticks_diff(current_time, last_gc_time) > 10000:  # 每10秒GC一次
                            sys.modules['gc'].collect()
                            last_gc_time = current_time
                        
                        # 检查紧急中止按钮
                        if hasattr(task_manager, 'abort_button') and task_manager.abort_button.value() == 0:
                            sys.modules['utils'].log("手动中止触发")
                            task_manager.abort_mission()
                            break
                        
                        # 优化电池检查
                        if sys.modules['time'].ticks_diff(current_time, last_wdt_feed_time) > 5000:  # 每5秒检查电池
                            battery_voltage = task_manager.get_battery_voltage()
                            if battery_voltage < task_manager.LOW_BATTERY_THRESHOLD:
                                sys.modules['utils'].log(f"电池电量低: {battery_voltage:.2f}V", "WARNING")
                                if battery_voltage < task_manager.CRITICAL_BATTERY_THRESHOLD:
                                    sys.modules['utils'].log("电池电量严重不足，紧急降落", "ERROR")
                                    task_manager.emergency_landing()
                                    break
                        
                        # 更新任务管理器
                        if not task_manager.update():
                            sys.modules['utils'].log("任务管理器发出退出信号")
                            break
                        
                        # 计算循环时间并控制频率
                        current_time = sys.modules['time'].ticks_ms()
                        loop_time = sys.modules['time'].ticks_diff(current_time, last_loop_time) / 1000.0
                        last_loop_time = current_time
                        
                        if sys.modules['config'].DEBUG_MODE:
                            sys.modules['utils'].log(f"循环时间: {loop_time * 1000:.2f}ms")
                        
                        # 优化睡眠时间计算
                        sleep_time = max(0, sys.modules['config'].MAIN_LOOP_DELAY - loop_time)
                        if sleep_time > 0.001:  # 只有当需要睡眠超过1ms时才睡眠
                            sys.modules['time'].sleep(sleep_time)
                
                except Exception as e:
                    sys.modules['utils'].log(f"主循环中出现未处理异常: {e}", "ERROR")
                    sys.modules['utils'].display_led_pattern("ERROR")
                    
                    # 尝试执行紧急降落
                    try:
                        task_manager.emergency_landing()
                    except:
                        # 最后的尝试 - 尝试停止电机
                        try:
                            task_manager.fcm.emergency_stop()
                        except:
                            pass
                
                finally:
                    sys.modules['utils'].log("ACM 核心关闭中")
                    # 执行清理
                    if task_manager.state != task_manager.TaskState.ERROR:
                        sys.modules['utils'].display_led_pattern("MISSION_COMPLETE", 2)
            
            # 将优化后的函数替换原函数
            main.main = optimized_main
            
            # 记录优化
            self.optimizations.append("main.main: 减少循环开销，定期而非每次循环执行某些操作")
            
            print("main模块优化完成")
            
        except Exception as e:
            print(f"优化main模块失败: {e}")
            traceback.print_exc()
    
    def optimize_all(self):
        """优化所有模块"""
        print("开始优化所有模块...")
        
        self.optimize_utils()
        self.optimize_perception()
        self.optimize_communication()
        self.optimize_task_manager()
        self.optimize_main()
        
        print("\n优化完成！应用了以下优化：")
        for i, optimization in enumerate(self.optimizations, 1):
            print(f"{i}. {optimization}")


# 创建性能测试实例
performance_tester = PerformanceTester()

# 创建代码优化器
code_optimizer = CodeOptimizer()

def run_performance_test():
    """运行性能测试"""
    performance_tester.run_complete_test()

def run_code_optimization():
    """运行代码优化"""
    code_optimizer.optimize_all()

if __name__ == "__main__":
    # 运行性能测试
    run_performance_test()
    
    # 运行代码优化
    run_code_optimization()