"""
FCM模拟器与ACM集成测试脚本
用于模拟FCM并与ACM进行集成测试
"""
import sys
import time
import threading
import socket
import json
import math
import random
import importlib
import traceback
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# 确保能导入所有模块
sys.path.append('.')

# 导入硬件模拟器
from hardware_simulator import hw_simulator

class FCMSimulator:
    """FCM模拟器类，模拟飞行控制模块的功能"""
    
    def __init__(self, gui=None):
        # 飞行状态
        self.motors_on = False
        self.position = [0, 0, 0]  # x, y, z (米)
        self.velocity = [0, 0, 0]  # vx, vy, vz (米/秒)
        self.attitude = [0, 0, 0]  # roll, pitch, yaw (度)
        self.altitude = 0  # 高度 (米)
        
        # 目标状态
        self.target_position = [0, 0, 0]
        self.target_velocity = [0, 0, 0]
        self.target_attitude = [0, 0, 0]
        self.target_altitude = 0
        
        # 物理参数
        self.max_velocity = 2.0  # 最大速度 (米/秒)
        self.max_acceleration = 1.0  # 最大加速度 (米/秒²)
        self.max_angular_velocity = 45.0  # 最大角速度 (度/秒)
        self.gravity = 9.81  # 重力加速度 (米/秒²)
        
        # I2C通信缓冲区
        self.command_buffer = []
        
        # 状态变化回调
        self.on_state_change = None
        
        # 上次更新时间
        self.last_update_time = time.time()
        
        # 模拟噪声
        self.position_noise = 0.02  # 位置噪声 (米)
        self.attitude_noise = 0.5  # 姿态噪声 (度)
        
        # GUI引用
        self.gui = gui
        
        # 运行标志
        self.running = False
        self.thread = None
        
        # 日志缓冲区
        self.log_buffer = []
    
    def log(self, message):
        """记录日志"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[FCM {timestamp}] {message}"
        self.log_buffer.append(log_entry)
        
        # 如果有GUI，也更新GUI日志
        if self.gui and hasattr(self.gui, 'log'):
            self.gui.log(log_entry)
        else:
            print(log_entry)
    
    def process_command(self, command, data=None):
        """处理来自ACM的命令"""
        if command == "MOTORS_ON":
            self.motors_on = True
            self.log("电机启动")
        
        elif command == "MOTORS_OFF":
            self.motors_on = False
            self.log("电机关闭")
        
        elif command == "SET_POSITION":
            if data and len(data) == 3:
                self.target_position = list(data)
                self.log(f"设置目标位置: ({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f})")
        
        elif command == "SET_VELOCITY":
            if data and len(data) == 3:
                self.target_velocity = list(data)
                self.log(f"设置目标速度: ({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f})")
        
        elif command == "SET_ATTITUDE":
            if data and len(data) == 3:
                self.target_attitude = list(data)
                self.log(f"设置目标姿态: ({data[0]:.2f}, {data[1]:.2f}, {data[2]:.2f})")
        
        elif command == "SET_ALTITUDE":
            if data and len(data) == 1:
                self.target_altitude = data[0]
                self.target_position[2] = data[0]  # 同时更新目标位置的高度
                self.log(f"设置目标高度: {data[0]:.2f}")
        
        elif command == "EMERGENCY_STOP":
            self.motors_on = False
            self.velocity = [0, 0, 0]
            self.log("紧急停止")
    
    def update(self, dt):
        """更新飞行状态"""
        if not self.motors_on:
            # 模拟重力作用
            if self.position[2] > 0:
                self.velocity[2] -= self.gravity * dt
            else:
                self.position[2] = 0
                self.velocity = [0, 0, 0]
            
            # 更新位置
            for i in range(3):
                self.position[i] += self.velocity[i] * dt
            
            # 确保不会低于地面
            if self.position[2] < 0:
                self.position[2] = 0
                self.velocity[2] = 0
            
            return
        
        # 计算目标位置与当前位置的差距
        position_error = [
            self.target_position[i] - self.position[i]
            for i in range(3)
        ]
        
        # 计算目标速度
        # 使用P控制器计算速度，与位置误差成正比
        desired_velocity = [
            min(self.max_velocity, max(-self.max_velocity, position_error[i] * 2.0))
            for i in range(3)
        ]
        
        # 计算加速度
        # 使用P控制器计算加速度，与速度误差成正比
        velocity_error = [
            desired_velocity[i] - self.velocity[i]
            for i in range(3)
        ]
        
        acceleration = [
            min(self.max_acceleration, max(-self.max_acceleration, velocity_error[i] * 5.0))
            for i in range(3)
        ]
        
        # 更新速度
        for i in range(3):
            self.velocity[i] += acceleration[i] * dt
            # 限制速度
            self.velocity[i] = min(self.max_velocity, max(-self.max_velocity, self.velocity[i]))
        
        # 更新位置
        for i in range(3):
            self.position[i] += self.velocity[i] * dt
            # 添加随机噪声
            self.position[i] += random.uniform(-self.position_noise, self.position_noise)
        
        # 确保不会低于地面
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = 0
        
        # 更新姿态（简化模型）
        # 姿态变化与速度相关
        self.attitude[0] = -self.velocity[1] * 5.0  # roll与y方向速度相关
        self.attitude[1] = self.velocity[0] * 5.0   # pitch与x方向速度相关
        
        # 添加姿态噪声
        for i in range(3):
            self.attitude[i] += random.uniform(-self.attitude_noise, self.attitude_noise)
        
        # 更新高度
        self.altitude = self.position[2]
        
        # 触发状态变化回调
        if self.on_state_change:
            self.on_state_change(self.position, self.velocity, self.attitude, self.altitude)
    
    def run_update_loop(self):
        """运行更新循环"""
        while self.running:
            # 计算时间差
            current_time = time.time()
            dt = current_time - self.last_update_time
            self.last_update_time = current_time
            
            # 更新状态
            self.update(dt)
            
            # 控制更新频率
            time.sleep(0.02)  # 50Hz
    
    def start(self):
        """启动FCM模拟器"""
        if self.thread is not None and self.thread.is_alive():
            return
        
        self.running = True
        self.last_update_time = time.time()
        self.thread = threading.Thread(target=self.run_update_loop)
        self.thread.daemon = True
        self.thread.start()
        self.log("FCM模拟器启动")
    
    def stop(self):
        """停止FCM模拟器"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.log("FCM模拟器停止")


class FCMAcmIntegrationGUI:
    """FCM与ACM集成测试的图形界面"""
    
    def __init__(self, root=None):
        # 创建主窗口
        if root is None:
            self.root = tk.Tk()
            self.root.title("FCM与ACM集成测试")
            self.root.geometry("1200x800")
        else:
            self.root = root
        
        # 创建FCM模拟器
        self.fcm = FCMSimulator(gui=self)
        
        # 创建选项卡
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True)
        
        # 创建控制面板选项卡
        self.control_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.control_frame, text="控制面板")
        
        # 创建飞行状态选项卡
        self.status_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.status_frame, text="飞行状态")
        
        # 创建I2C通信选项卡
        self.comm_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.comm_frame, text="I2C通信")
        
        # 创建日志选项卡
        self.log_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.log_frame, text="日志")
        
        # 初始化各选项卡
        self._init_control_panel()
        self._init_status_panel()
        self._init_comm_panel()
        self._init_log_panel()
        
        # 设置状态变化回调
        self.fcm.on_state_change = self._update_status_display
        
        # 时间和高度历史数据
        self.time_data = []
        self.altitude_data = []
        self.start_time = time.time()
        
        # 通信历史
        self.command_history = []
    
    def _init_control_panel(self):
        """初始化控制面板"""
        # 控制按钮
        button_frame = ttk.Frame(self.control_frame)
        button_frame.pack(pady=10, padx=10, fill="x")
        
        self.start_button = ttk.Button(button_frame, text="启动FCM", command=self.fcm.start)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="停止FCM", command=self.fcm.stop)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        self.emergency_button = ttk.Button(button_frame, text="紧急停止", 
                                        command=lambda: self.fcm.process_command("EMERGENCY_STOP"))
        self.emergency_button.pack(side=tk.LEFT, padx=5)
        
        # 创建集成测试按钮
        self.integration_test_button = ttk.Button(button_frame, text="运行集成测试", 
                                               command=self.run_integration_test)
        self.integration_test_button.pack(side=tk.LEFT, padx=5)
        
        # 模拟通信按钮
        self.mock_comm_button = ttk.Button(button_frame, text="模拟I2C通信", 
                                        command=self.mock_i2c_communication)
        self.mock_comm_button.pack(side=tk.LEFT, padx=5)
        
        # 位置输入
        position_frame = ttk.LabelFrame(self.control_frame, text="目标位置")
        position_frame.pack(pady=10, padx=10, fill="x")
        
        ttk.Label(position_frame, text="X (米):").grid(row=0, column=0, padx=5, pady=5)
        self.x_entry = ttk.Entry(position_frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)
        self.x_entry.insert(0, "0.0")
        
        ttk.Label(position_frame, text="Y (米):").grid(row=0, column=2, padx=5, pady=5)
        self.y_entry = ttk.Entry(position_frame, width=10)
        self.y_entry.grid(row=0, column=3, padx=5, pady=5)
        self.y_entry.insert(0, "0.0")
        
        ttk.Label(position_frame, text="Z (米):").grid(row=0, column=4, padx=5, pady=5)
        self.z_entry = ttk.Entry(position_frame, width=10)
        self.z_entry.grid(row=0, column=5, padx=5, pady=5)
        self.z_entry.insert(0, "0.0")
        
        self.set_position_button = ttk.Button(position_frame, text="设置位置", 
                                           command=self.set_position)
        self.set_position_button.grid(row=0, column=6, padx=5, pady=5)
        
        # 3D轨迹图
        trajectory_frame = ttk.LabelFrame(self.control_frame, text="飞行轨迹")
        trajectory_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        # 创建图形
        self.fig = Figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (米)')
        self.ax.set_ylabel('Y (米)')
        self.ax.set_zlabel('Z (米)')
        self.ax.set_title('无人机轨迹')
        
        # 设置坐标范围
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.ax.set_zlim(0, 3)
        
        # 添加路径线
        x = [0, 2, 4, 6, 8, 8, 8, 8, 6, 6, 6, 4, 4, 4, 4]
        y = [0, 0, 0, 0, 0, 2, 4, 6, 6, 6, 6, 8, 8, 8, 8]
        z = [2.8, 2.8, 2.8, 2.8, 2.8, 2.6, 2.6, 2.6, 2.7, 0.2, 2.7, 2.7, 0.2, 2.7, 0.1]
        
        # 添加任务点
        self.ax.scatter(x, y, z, c='r', marker='o')
        self.ax.plot(x, y, z, 'r--', alpha=0.5)
        
        # 初始无人机位置
        self.drone_point = self.ax.scatter([0], [0], [0], c='b', marker='^', s=100)
        
        # 轨迹历史
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_z = []
        self.trajectory_line, = self.ax.plot([], [], [], 'b-', alpha=0.5)
        
        # 设置画布
        self.canvas = FigureCanvasTkAgg(self.fig, trajectory_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def _init_status_panel(self):
        """初始化飞行状态面板"""
        # 飞行状态表格
        status_frame = ttk.LabelFrame(self.status_frame, text="当前飞行状态")
        status_frame.pack(pady=10, padx=10, fill="x")
        
        # 当前位置
        ttk.Label(status_frame, text="当前位置:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.position_label = ttk.Label(status_frame, text="X: 0.00 m, Y: 0.00 m, Z: 0.00 m")
        self.position_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # 当前速度
        ttk.Label(status_frame, text="当前速度:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.velocity_label = ttk.Label(status_frame, text="X: 0.00 m/s, Y: 0.00 m/s, Z: 0.00 m/s")
        self.velocity_label.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        # 当前姿态
        ttk.Label(status_frame, text="当前姿态:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.attitude_label = ttk.Label(status_frame, text="Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°")
        self.attitude_label.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        
        # 当前高度
        ttk.Label(status_frame, text="当前高度:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.altitude_label = ttk.Label(status_frame, text="0.00 m")
        self.altitude_label.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        
        # 电机状态
        ttk.Label(status_frame, text="电机状态:").grid(row=4, column=0, padx=5, pady=5, sticky="w")
        self.motors_label = ttk.Label(status_frame, text="关闭")
        self.motors_label.grid(row=4, column=1, padx=5, pady=5, sticky="w")
        
        # 目标状态表格
        target_frame = ttk.LabelFrame(self.status_frame, text="目标状态")
        target_frame.pack(pady=10, padx=10, fill="x")
        
        # 目标位置
        ttk.Label(target_frame, text="目标位置:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.target_position_label = ttk.Label(target_frame, text="X: 0.00 m, Y: 0.00 m, Z: 0.00 m")
        self.target_position_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        # 目标速度
        ttk.Label(target_frame, text="目标速度:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.target_velocity_label = ttk.Label(target_frame, text="X: 0.00 m/s, Y: 0.00 m/s, Z: 0.00 m/s")
        self.target_velocity_label.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        # 目标姿态
        ttk.Label(target_frame, text="目标姿态:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.target_attitude_label = ttk.Label(target_frame, text="Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°")
        self.target_attitude_label.grid(row=2, column=1, padx=5, pady=5, sticky="w")
        
        # 目标高度
        ttk.Label(target_frame, text="目标高度:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.target_altitude_label = ttk.Label(target_frame, text="0.00 m")
        self.target_altitude_label.grid(row=3, column=1, padx=5, pady=5, sticky="w")
        
        # 状态历史图表
        hist_frame = ttk.LabelFrame(self.status_frame, text="高度历史")
        hist_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        # 创建图形
        self.hist_fig = Figure(figsize=(8, 4))
        self.hist_ax = self.hist_fig.add_subplot(111)
        self.hist_ax.set_xlabel('时间 (秒)')
        self.hist_ax.set_ylabel('高度 (米)')
        self.hist_ax.set_title('高度历史')
        self.hist_ax.set_ylim(0, 3)
        
        # 创建线条
        self.hist_line, = self.hist_ax.plot([], [], 'b-')
        
        # 设置画布
        self.hist_canvas = FigureCanvasTkAgg(self.hist_fig, hist_frame)
        self.hist_canvas.draw()
        self.hist_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def _init_comm_panel(self):
        """初始化通信面板"""
        # I2C命令记录
        command_frame = ttk.LabelFrame(self.comm_frame, text="I2C命令记录")
        command_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        # 命令列表
        ttk.Label(command_frame, text="接收到的命令:").pack(anchor="w", padx=5, pady=5)
        
        # 创建列表框和滚动条
        command_scroll = ttk.Scrollbar(command_frame)
        command_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.command_listbox = tk.Listbox(command_frame, yscrollcommand=command_scroll.set, height=15)
        self.command_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        command_scroll.config(command=self.command_listbox.yview)
        
        # 手动发送命令
        send_frame = ttk.LabelFrame(self.comm_frame, text="发送模拟命令")
        send_frame.pack(pady=10, padx=10, fill="x")
        
        # 命令选择
        ttk.Label(send_frame, text="命令:").grid(row=0, column=0, padx=5, pady=5)
        self.command_combo = ttk.Combobox(send_frame, values=[
            "MOTORS_ON", "MOTORS_OFF", "SET_POSITION", "SET_VELOCITY", 
            "SET_ATTITUDE", "SET_ALTITUDE", "EMERGENCY_STOP"
        ])
        self.command_combo.grid(row=0, column=1, padx=5, pady=5)
        self.command_combo.current(0)
        
        # 数据输入
        ttk.Label(send_frame, text="数据 (JSON):").grid(row=1, column=0, padx=5, pady=5)
        self.data_entry = ttk.Entry(send_frame, width=40)
        self.data_entry.grid(row=1, column=1, padx=5, pady=5)
        self.data_entry.insert(0, "[0, 0, 0]")
        
        # 发送按钮
        self.send_button = ttk.Button(send_frame, text="发送命令", command=self.send_manual_command)
        self.send_button.grid(row=1, column=2, padx=5, pady=5)
    
    def _init_log_panel(self):
        """初始化日志面板"""
        # 创建日志文本框和滚动条
        log_scroll = ttk.Scrollbar(self.log_frame)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.log_text = tk.Text(self.log_frame, yscrollcommand=log_scroll.set)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        log_scroll.config(command=self.log_text.yview)
        
        # 清除日志按钮
        clear_button = ttk.Button(self.log_frame, text="清除日志", command=self.clear_log)
        clear_button.pack(pady=5)
    
    def log(self, message):
        """记录日志"""
        # 在UI线程中更新日志
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)  # 自动滚动到最新日志
    
    def clear_log(self):
        """清除日志"""
        self.log_text.delete(1.0, tk.END)
    
    def set_position(self):
        """设置目标位置"""
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            
            # 处理命令
            self.fcm.process_command("SET_POSITION", (x, y, z))
            
            self.log(f"手动设置目标位置: ({x:.2f}, {y:.2f}, {z:.2f})")
        except ValueError:
            self.log("错误：请输入有效的数值")
    
    def send_manual_command(self):
        """发送手动命令"""
        try:
            command = self.command_combo.get()
            data_str = self.data_entry.get().strip()
            
            # 解析数据
            data = None
            if data_str:
                data = json.loads(data_str)
            
            # 处理命令
            self.fcm.process_command(command, data)
            
            self.log(f"手动发送命令: {command}, 数据: {data}")
        except Exception as e:
            self.log(f"发送命令错误: {e}")
    
    def _update_status_display(self, position, velocity, attitude, altitude):
        """更新状态显示"""
        # 更新位置标签
        self.position_label.config(text=f"X: {position[0]:.2f} m, Y: {position[1]:.2f} m, Z: {position[2]:.2f} m")
        
        # 更新速度标签
        self.velocity_label.config(text=f"X: {velocity[0]:.2f} m/s, Y: {velocity[1]:.2f} m/s, Z: {velocity[2]:.2f} m/s")
        
        # 更新姿态标签
        self.attitude_label.config(text=f"Roll: {attitude[0]:.2f}°, Pitch: {attitude[1]:.2f}°, Yaw: {attitude[2]:.2f}°")
        
        # 更新高度标签
        self.altitude_label.config(text=f"{altitude:.2f} m")
        
        # 更新电机状态标签
        self.motors_label.config(text="开启" if self.fcm.motors_on else "关闭")
        
        # 更新目标状态标签
        self.target_position_label.config(
            text=f"X: {self.fcm.target_position[0]:.2f} m, Y: {self.fcm.target_position[1]:.2f} m, Z: {self.fcm.target_position[2]:.2f} m"
        )
        self.target_velocity_label.config(
            text=f"X: {self.fcm.target_velocity[0]:.2f} m/s, Y: {self.fcm.target_velocity[1]:.2f} m/s, Z: {self.fcm.target_velocity[2]:.2f} m/s"
        )
        self.target_attitude_label.config(
            text=f"Roll: {self.fcm.target_attitude[0]:.2f}°, Pitch: {self.fcm.target_attitude[1]:.2f}°, Yaw: {self.fcm.target_attitude[2]:.2f}°"
        )
        self.target_altitude_label.config(text=f"{self.fcm.target_altitude:.2f} m")
        
        # 更新轨迹图
        self.trajectory_x.append(position[0])
        self.trajectory_y.append(position[1])
        self.trajectory_z.append(position[2])
        
        # 限制轨迹点数量，避免内存问题
        max_points = 1000
        if len(self.trajectory_x) > max_points:
            self.trajectory_x = self.trajectory_x[-max_points:]
            self.trajectory_y = self.trajectory_y[-max_points:]
            self.trajectory_z = self.trajectory_z[-max_points:]
        
        # 更新轨迹线
        self.trajectory_line.set_data(self.trajectory_x, self.trajectory_y)
        self.trajectory_line.set_3d_properties(self.trajectory_z)
        
        # 更新无人机位置点
        self.drone_point.remove()
        self.drone_point = self.ax.scatter([position[0]], [position[1]], [position[2]], c='b', marker='^', s=100)
        
        # 重新绘制图形
        self.canvas.draw_idle()
        
        # 更新高度历史图表
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.altitude_data.append(altitude)
        
        # 限制数据点数量
        max_hist_points = 200
        if len(self.time_data) > max_hist_points:
            self.time_data = self.time_data[-max_hist_points:]
            self.altitude_data = self.altitude_data[-max_hist_points:]
        
        # 更新图表
        self.hist_line.set_data(self.time_data, self.altitude_data)
        self.hist_ax.set_xlim(max(0, current_time - 30), current_time + 1)  # 显示最近30秒
        self.hist_canvas.draw_idle()
    
    def mock_i2c_communication(self):
        """模拟I2C通信，用于与ACM集成测试"""
        self.log("模拟I2C通信...")
        
        try:
            # 模拟ACM发送命令到FCM
            # 这里我们假设ACM每次更新时都会发送位置命令
            command = "SET_POSITION"
            data = (4.0, 6.0, 2.5)  # 例如，飞到第三个任务点
            
            # 处理命令
            self.fcm.process_command(command, data)
            
            # 记录命令
            command_entry = f"命令: {command}, 数据: {data}"
            self.command_listbox.insert(tk.END, command_entry)
            self.command_listbox.see(tk.END)
            self.command_history.append(command_entry)
            
            self.log(f"模拟I2C命令: {command}, 数据: {data}")
            
        except Exception as e:
            self.log(f"模拟I2C通信错误: {e}")
    
    def run_integration_test(self):
        """运行FCM与ACM集成测试"""
        self.log("开始FCM与ACM集成测试...")
        
        # 启动FCM模拟器
        self.fcm.start()
        
        # 创建测试线程
        test_thread = threading.Thread(target=self._run_test_sequence)
        test_thread.daemon = True
        test_thread.start()
    
    def _run_test_sequence(self):
        """运行测试序列"""
        try:
            # 导入ACM模块
            self.log("导入ACM模块...")
            
            # 重定向I2C通信
            # 这里需要修改hardware_simulator.py，使其调用FCM模拟器的process_command方法
            # 而不是使用默认的模拟I2C通信
            hw_simulator.machine.I2C.writeto = self._mock_i2c_writeto
            
            # 创建测试场景
            self.log("创建测试场景...")
            
            # 任务1：起飞
            self.log("任务1：起飞...")
            time.sleep(1)
            
            # 处理电机开启命令
            self.fcm.process_command("MOTORS_ON")
            
            # 处理起飞命令
            self.fcm.process_command("SET_POSITION", (0, 0, 2.8))
            time.sleep(2)
            
            # 任务2：穿越隧道
            self.log("任务2：穿越隧道...")
            
            # 第一个隧道
            self.fcm.process_command("SET_POSITION", (2, 0, 2.8))
            time.sleep(2)
            
            # 第二个隧道
            self.fcm.process_command("SET_POSITION", (4, 0, 2.8))
            time.sleep(2)
            
            # 第三个隧道
            self.fcm.process_command("SET_POSITION", (6, 0, 2.8))
            time.sleep(2)
            
            # 任务3：障碍物避开
            self.log("任务3：障碍物避开...")
            
            # 右转
            self.fcm.process_command("SET_POSITION", (8, 2, 2.6))
            time.sleep(2)
            
            # 第一个横梁
            self.fcm.process_command("SET_POSITION", (8, 4, 2.6))
            time.sleep(2)
            
            # 第二个横梁
            self.fcm.process_command("SET_POSITION", (8, 6, 2.6))
            time.sleep(2)
            
            # 任务4：包裹搜索和拾取
            self.log("任务4：包裹搜索和拾取...")
            
            # 接近红色区域
            self.fcm.process_command("SET_POSITION", (6, 6, 2.7))
            time.sleep(2)
            
            # 降低高度进行拾取
            self.fcm.process_command("SET_POSITION", (6, 6, 0.2))
            time.sleep(2)
            
            # 拾取后上升
            self.fcm.process_command("SET_POSITION", (6, 6, 2.7))
            time.sleep(2)
            
            # 任务5：标记搜索和包裹放置
            self.log("任务5：标记搜索和包裹放置...")
            
            # 接近绿色区域
            self.fcm.process_command("SET_POSITION", (4, 8, 2.7))
            time.sleep(2)
            
            # 降低高度进行放置
            self.fcm.process_command("SET_POSITION", (4, 8, 0.2))
            time.sleep(2)
            
            # 放置后上升
            self.fcm.process_command("SET_POSITION", (4, 8, 2.7))
            time.sleep(2)
            
            # 任务6：降落和通信
            self.log("任务6：降落和通信...")
            
            # 降落
            self.fcm.process_command("SET_POSITION", (4, 8, 0.1))
            time.sleep(2)
            
            # 关闭电机
            self.fcm.process_command("MOTORS_OFF")
            
            self.log("集成测试完成！")
            
        except Exception as e:
            self.log(f"集成测试错误: {e}")
            traceback.print_exc()
    
    def _mock_i2c_writeto(self, addr, data):
        """模拟I2C写入，重定向到FCM模拟器"""
        if addr == 0x08:  # FCM地址
            # 解析命令和数据
            if len(data) > 0:
                cmd = data[0]
                # 将命令代码转换为命令名称
                cmd_names = {
                    0x00: "MOTORS_OFF",
                    0x01: "MOTORS_ON",
                    0x02: "SET_POSITION",
                    0x03: "SET_VELOCITY",
                    0x04: "SET_ATTITUDE",
                    0x05: "SET_ALTITUDE",
                    0xFF: "EMERGENCY_STOP"
                }
                cmd_name = cmd_names.get(cmd, f"UNKNOWN_{cmd}")
                
                # 处理数据部分
                cmd_data = None
                if len(data) > 2:  # 有数据部分
                    if cmd == 0x02:  # SET_POSITION
                        # 解析三个浮点数(x, y, z)
                        import struct
                        if len(data) >= 14:  # 1(cmd) + 1(seq) + 4*3(float)
                            try:
                                x = struct.unpack('f', data[2:6])[0]
                                y = struct.unpack('f', data[6:10])[0]
                                z = struct.unpack('f', data[10:14])[0]
                                cmd_data = (x, y, z)
                            except:
                                pass
                    elif cmd == 0x05:  # SET_ALTITUDE
                        # 解析一个浮点数(altitude)
                        import struct
                        if len(data) >= 6:  # 1(cmd) + 1(seq) + 4(float)
                            try:
                                altitude = struct.unpack('f', data[2:6])[0]
                                cmd_data = (altitude,)
                            except:
                                pass
                
                # 将命令转发到FCM模拟器
                self.fcm.process_command(cmd_name, cmd_data)
                
                # 记录命令
                command_entry = f"命令: {cmd_name}, 数据: {cmd_data}"
                self.command_listbox.insert(tk.END, command_entry)
                self.command_listbox.see(tk.END)
                self.command_history.append(command_entry)
    
    def run(self):
        """运行图形界面"""
        self.root.mainloop()


def create_complete_simulation():
    """创建完整的FCM模拟环境和ACM集成测试"""
    # 创建GUI
    gui = FCMAcmIntegrationGUI()
    
    # 运行GUI
    gui.run()


if __name__ == "__main__":
    create_complete_simulation()