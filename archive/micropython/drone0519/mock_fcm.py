"""
FCM模拟器 - 模拟飞行控制模块的行为
用于在计算机上测试ACM代码
"""
import time
import math
import threading
import json

class MockFCM:
    """模拟飞行控制模块"""
    
    def __init__(self, i2c_address=0x08):
        """初始化模拟FCM"""
        self.i2c_address = i2c_address
        self.motors_active = False
        self.position = [0, 0, 0]  # x, y, z
        self._current_x = 0
        self._current_y = 0
        self._current_z = 0
        self.velocity = [0, 0, 0]  # vx, vy, vz
        self.attitude = [0, 0, 0]  # roll, pitch, yaw
        self.altitude = 0
        
        # 为动态更新创建线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        # 最后接收到的命令
        self.last_command = None
        self.last_data = None
        
        # 命令历史记录
        self.command_history = []
        
        print("模拟FCM已初始化，地址：0x{:02X}".format(i2c_address))
    
    def receive_command(self, command, data=None):
        """
        接收并处理来自ACM的命令
        
        Args:
            command: 命令代码
            data: 命令数据（如有）
        
        Returns:
            成功处理返回True，否则返回False
        """
        self.last_command = command
        self.last_data = data
        
        # 记录命令
        self.command_history.append({
            'time': time.time(),
            'command': command,
            'data': data
        })
        
        # 处理各种命令
        if command == "MOTORS_ON":
            self.motors_active = True
            print("FCM模拟：电机已打开")
            return True
            
        elif command == "MOTORS_OFF":
            self.motors_active = False
            print("FCM模拟：电机已关闭")
            return True
            
        elif command == "EMERGENCY_STOP":
            self.motors_active = False
            self.velocity = [0, 0, 0]
            print("FCM模拟：紧急停止！")
            return True
            
        elif command == "SET_POSITION":
            if isinstance(data, tuple) and len(data) == 3:
                self.position = [data[0], data[1], data[2]]
                print(f"FCM模拟：设置位置 -> {self.position}")
                return True
            return False
            
        elif command == "SET_VELOCITY":
            if isinstance(data, tuple) and len(data) == 3:
                self.velocity = [data[0], data[1], data[2]]
                print(f"FCM模拟：设置速度 -> {self.velocity}")
                return True
            return False
            
        elif command == "SET_ATTITUDE":
            if isinstance(data, tuple) and len(data) == 3:
                self.attitude = [data[0], data[1], data[2]]
                print(f"FCM模拟：设置姿态 -> {self.attitude}")
                return True
            return False
            
        elif command == "SET_ALTITUDE":
            if isinstance(data, tuple) and len(data) == 1:
                self.altitude = data[0]
                print(f"FCM模拟：设置高度 -> {self.altitude}")
                return True
            return False
            
        else:
            print(f"FCM模拟：未知命令 -> {command}")
            return False
    
    def _update_loop(self):
        """更新线程，模拟飞行控制动态"""
        last_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # 如果电机激活且有目标位置，更新模拟飞行动态
            if self.motors_active:
                # 简单物理模拟 - 逐渐接近目标位置
                self._update_position(dt)
            
            time.sleep(0.01)  # 100Hz更新频率
    
    def _update_position(self, dt):
        """更新位置基于简单的物理模型"""
        # 简化模型 - 考虑目标位置和当前位置的差异
        # 在实际环境中，这会更复杂，包括物理约束、加速度和动态特性
        
        # 计算当前位置与目标位置之间的差异
        error_x = self.position[0] - self._current_x
        error_y = self.position[1] - self._current_y
        error_z = self.position[2] - self._current_z
        
        # 简单模型，当前位置朝目标位置移动
        speed = 0.5  # 米/秒
        max_move = speed * dt
        
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        if distance > 0:
            scale = min(max_move / distance, 1.0)
            
            self._current_x += error_x * scale
            self._current_y += error_y * scale
            self._current_z += error_z * scale
    
    def get_current_state(self):
        """获取FCM当前状态"""
        return {
            'motors_active': self.motors_active,
            'position': self.position,
            'actual_position': [self._current_x, self._current_y, self._current_z],
            'velocity': self.velocity,
            'attitude': self.attitude,
            'altitude': self.altitude
        }
    
    def save_command_history(self, filename="fcm_command_history.json"):
        """保存命令历史到文件"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.command_history, f, indent=2)
            print(f"命令历史已保存到 {filename}")
        except Exception as e:
            print(f"保存命令历史时出错: {e}")
    
    def stop(self):
        """停止模拟器"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        print("FCM模拟器已停止")