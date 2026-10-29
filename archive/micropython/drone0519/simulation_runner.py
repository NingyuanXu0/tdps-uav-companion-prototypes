"""
模拟器主程序 - 在计算机上模拟整个无人机系统
"""
import os
import sys
import time
import json
import threading
import importlib
import traceback
import argparse
from mock_fcm import MockFCM
from mock_sensors import MockVL53L0XSensor, MockHCSR04Sensor
from mock_wireless import MockWirelessCommunication

# 模拟环境类
class DroneSimulationEnvironment:
    """无人机模拟环境"""
    
    def __init__(self):
        """初始化模拟环境"""
        self.fcm = None
        self.height_sensor = None
        self.wall_sensor = None
        self.wireless = None
        
        # 环境状态
        self.drone_position = [0, 0, 0]  # x, y, z
        self.wall_distance = 2.0  # 初始墙壁距离（米）
        
        # 状态记录
        self.state_history = []
        self.record_interval = 0.1  # 记录间隔（秒）
        self.last_record_time = 0
        
        # 模拟线程
        self.running = False
        self.simulation_thread = None
        
        print("无人机模拟环境已创建")
    
    def initialize(self):
        """初始化模拟环境组件"""
        self.fcm = MockFCM()
        self.height_sensor = MockVL53L0XSensor(initial_height=0.1)  # 初始高度0.1米
        self.wall_sensor = MockHCSR04Sensor(initial_distance=2.0)  # 初始墙距2米
        self.wireless = MockWirelessCommunication()
        
        print("无人机模拟环境已初始化")
        return True
    
    def start_simulation(self):
        """启动模拟"""
        if self.running:
            print("模拟已在运行中")
            return
        
        self.running = True
        self.simulation_thread = threading.Thread(target=self._simulation_loop)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()
        print("无人机模拟已启动")
    
    def stop_simulation(self):
        """停止模拟"""
        self.running = False
        if self.simulation_thread and self.simulation_thread.is_alive():
            self.simulation_thread.join(timeout=1.0)
        
        # 保存模拟历史
        self.save_state_history()
        
        # 如果有FCM，保存其命令历史
        if self.fcm:
            self.fcm.save_command_history()
        
        # 如果有无线模块，保存其传输日志
        if self.wireless:
            self.wireless.save_transmission_log()
        
        print("无人机模拟已停止")
    
    def _simulation_loop(self):
        """模拟主循环"""
        last_time = time.time()
        self.last_record_time = last_time
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # 更新环境状态
            self._update_environment(dt)
            
            # 更新传感器读数
            self._update_sensors()
            
            # 记录状态
            if current_time - self.last_record_time >= self.record_interval:
                self._record_state()
                self.last_record_time = current_time
            
            # 保持模拟频率（100Hz）
            time.sleep(0.01)
    
    def _update_environment(self, dt):
        """更新环境状态"""
        # 获取FCM状态
        fcm_state = self.fcm.get_current_state()
        
        # 更新无人机位置（简化模型）
        self.drone_position = fcm_state['actual_position']
        
        # 任务相关的墙壁距离变化模拟
        # 在实际任务中，墙壁距离会随着无人机在环境中移动而变化
        # 此处简化为基于x坐标的函数
        x_position = self.drone_position[0]
        
        # 模拟不同区域的墙壁距离
        if 2 <= x_position <= 8:  # 穿越隧道阶段
            # 根据位置变化墙壁距离
            self.wall_distance = max(0.3, 2.0 - abs(x_position - 5) * 0.2)
        else:
            # 其他区域墙壁较远
            self.wall_distance = 2.0 + (x_position * 0.1)
    
    def _update_sensors(self):
        """更新传感器读数"""
        # 更新高度传感器
        if self.height_sensor:
            self.height_sensor.set_current_height(self.drone_position[2])
        
        # 更新墙壁距离传感器
        if self.wall_sensor:
            self.wall_sensor.set_current_distance(self.wall_distance)
    
    def _record_state(self):
        """记录当前状态"""
        state = {
            'time': time.time(),
            'drone_position': list(self.drone_position),
            'wall_distance': self.wall_distance
        }
        
        # 记录FCM状态（如果可用）
        if self.fcm:
            state['fcm_state'] = self.fcm.get_current_state()
        
        # 记录高度传感器读数（如果可用）
        if self.height_sensor:
            state['height_sensor'] = self.height_sensor.get_distance()
        
        # 记录墙壁传感器读数（如果可用）
        if self.wall_sensor:
            state['wall_sensor'] = self.wall_sensor.get_distance()
        
        # 记录无线连接状态（如果可用）
        if self.wireless:
            state['wireless_connected'] = self.wireless.connected
        
        self.state_history.append(state)
    
    def save_state_history(self, filename="simulation_history.json"):
        """保存状态历史到文件"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.state_history, f, indent=2)
            print(f"模拟历史已保存到 {filename}")
        except Exception as e:
            print(f"保存模拟历史出错: {e}")
    
    def load_acm_module(self, main_module_path="./main.py"):
        """
        加载ACM主模块并连接到模拟环境
        
        Args:
            main_module_path: ACM主模块路径
        
        Returns:
            加载成功返回True，否则返回False
        """
        try:
            # 确保目录在Python路径中
            dir_path = os.path.dirname(os.path.abspath(main_module_path))
            if dir_path not in sys.path:
                sys.path.insert(0, dir_path)
            
            # 尝试导入我们需要模拟的模块
            try:
                import config
            except ImportError:
                # 如果模块不存在，我们创建一个假的模块
                import types
                sys.modules['config'] = types.ModuleType('config')
                import config
                
            try:
                import perception
            except ImportError:
                sys.modules['perception'] = types.ModuleType('perception')
                import perception
                
            try:
                import communication
            except ImportError:
                sys.modules['communication'] = types.ModuleType('communication')
                import communication
            
            # 替换模块中的类为我们的模拟类
            perception.VL53L0XSensor = MockVL53L0XSensor
            perception.HCSR04Sensor = MockHCSR04Sensor
            communication.FCMInterface = MockFCM
            communication.WirelessCommunication = MockWirelessCommunication
            
            # 重新加载主模块
            try:
                import main
                importlib.reload(main)
            except ImportError:
                print("警告：找不到main模块，将使用默认行为")
            
            print("ACM模块已加载并连接到模拟环境")
            return True
            
        except Exception as e:
            print(f"加载ACM模块时出错: {e}")
            traceback.print_exc()
            return False
    
    def run_acm_test(self, duration=60):
        """
        运行ACM测试指定时长
        
        Args:
            duration: 测试持续时间（秒）
        """
        try:
            print(f"开始运行ACM测试，持续{duration}秒...")
            
            # 启动模拟
            self.start_simulation()
            
            # 加载并运行ACM
            if not self.load_acm_module():
                self.stop_simulation()
                return
            
            # 等待指定时长
            time.sleep(duration)
            
            # 停止模拟
            self.stop_simulation()
            
            print("ACM测试完成")
            
        except Exception as e:
            print(f"运行ACM测试时出错: {e}")
            traceback.print_exc()
            self.stop_simulation()

# 主函数
def main():
    """模拟器主函数"""
    parser = argparse.ArgumentParser(description="无人机控制系统模拟器")
    parser.add_argument("--duration", type=int, default=60, help="模拟持续时间（秒）")
    args = parser.parse_args()
    
    print("启动无人机控制系统模拟器...")
    
    # 创建模拟环境
    env = DroneSimulationEnvironment()
    
    # 初始化模拟环境
    if not env.initialize():
        print("初始化模拟环境失败")
        return
    
    # 运行ACM测试
    env.run_acm_test(duration=args.duration)
    
    print("模拟器已退出")

if __name__ == "__main__":
    main()