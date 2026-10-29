"""
硬件模拟器模块
用于模拟ESP32和传感器硬件，以便在PC上测试代码
"""

import time
import threading
import json
from collections import deque
import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

class HardwareSimulator:
    """
    硬件模拟器类，模拟ESP32和传感器硬件
    """
    def __init__(self):
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("无人机硬件模拟器")
        self.root.geometry("1200x800")
        
        # 创建多个选项卡
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True)
        
        # 创建控制面板选项卡
        self.control_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.control_frame, text="控制面板")
        
        # 创建传感器数据选项卡
        self.sensor_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.sensor_frame, text="传感器数据")
        
        # 创建无线通信选项卡
        self.comm_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.comm_frame, text="无线通信")
        
        # 创建日志选项卡
        self.log_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.log_frame, text="日志")
        
        # 初始化各选项卡
        self._init_control_panel()
        self._init_sensor_panel()
        self._init_comm_panel()
        self._init_log_panel()
        
        # 初始化模拟状态
        self.running = False
        self.vl53l0x_distance = 2.8  # 默认高度2.8米
        self.hcsr04_distance = 1.0   # 默认墙壁距离1.0米
        self.battery_voltage = 15.0  # 默认电池电压15V
        self.drone_position = [0, 0, 0]  # 默认位置
        self.led_state = 0  # LED状态，0=关，1=开
        self.fcm_commands = deque(maxlen=100)  # 记录发送到FCM的命令
        self.mission_complete_msg = ""  # 任务完成消息
        
        # 创建锁，防止多线程访问冲突
        self.lock = threading.Lock()
        
        # 记录日志
        self.log("硬件模拟器初始化完成")
    
    def _init_control_panel(self):
        """初始化控制面板"""
        # 创建上半部分：无人机位置显示
        position_frame = ttk.LabelFrame(self.control_frame, text="无人机位置")
        position_frame.pack(pady=10, padx=10, fill="x")
        
        # 创建绘图区域
        self.fig = Figure(figsize=(10, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (米)')
        self.ax.set_ylabel('Y (米)')
        self.ax.set_zlabel('Z (米)')
        self.ax.set_title('无人机位置轨迹')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(0, 10)
        self.ax.set_zlim(0, 3)
        
        # 添加任务点
        x = [0, 2, 4, 6, 8, 8, 8, 8, 6, 6, 6, 4, 4, 4, 4]
        y = [0, 0, 0, 0, 0, 2, 4, 6, 6, 6, 6, 8, 8, 8, 8]
        z = [2.8, 2.8, 2.8, 2.8, 2.8, 2.6, 2.6, 2.6, 2.7, 0.2, 2.7, 2.7, 0.2, 2.7, 0.1]
        
        self.ax.scatter(x, y, z, c='r', marker='o')
        
        # 添加路径线
        self.ax.plot(x, y, z, 'r--', alpha=0.5)
        
        # 初始无人机位置
        self.drone_point = self.ax.scatter([0], [0], [0], c='b', marker='^', s=100)
        
        # 设置画布
        self.canvas = FigureCanvasTkAgg(self.fig, position_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 创建下半部分：控制按钮
        button_frame = ttk.Frame(self.control_frame)
        button_frame.pack(pady=10, padx=10, fill="x")
        
        # 启动按钮
        self.start_button = ttk.Button(button_frame, text="启动模拟器", command=self.start_simulation)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        # 停止按钮
        self.stop_button = ttk.Button(button_frame, text="停止模拟器", command=self.stop_simulation)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        self.stop_button.config(state=tk.DISABLED)
        
        # 按钮按下按钮
        self.button_press = ttk.Button(button_frame, text="按下按钮", command=self.press_button)
        self.button_press.pack(side=tk.LEFT, padx=5)
        
        # 紧急降落按钮
        self.emergency_button = ttk.Button(button_frame, text="紧急降落", command=self.emergency_landing)
        self.emergency_button.pack(side=tk.LEFT, padx=5)
        
        # 电池电压调节
        battery_frame = ttk.LabelFrame(self.control_frame, text="电池电压")
        battery_frame.pack(pady=10, padx=10, fill="x")
        
        self.battery_slider = ttk.Scale(battery_frame, from_=10.0, to=16.0, orient=tk.HORIZONTAL, 
                                      command=self.update_battery_voltage)
        self.battery_slider.set(15.0)
        self.battery_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.battery_label = ttk.Label(battery_frame, text="15.0V")
        self.battery_label.pack(side=tk.LEFT, padx=5)
    
    def _init_sensor_panel(self):
        """初始化传感器数据面板"""
        # 高度传感器数据
        height_frame = ttk.LabelFrame(self.sensor_frame, text="高度传感器 (VL53L0X)")
        height_frame.pack(pady=10, padx=10, fill="x")
        
        ttk.Label(height_frame, text="模拟高度 (米):").grid(row=0, column=0, padx=5, pady=5)
        self.height_slider = ttk.Scale(height_frame, from_=0.0, to=3.0, orient=tk.HORIZONTAL, 
                                     command=self.update_height)
        self.height_slider.set(2.8)
        self.height_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.height_label = ttk.Label(height_frame, text="2.8米")
        self.height_label.grid(row=0, column=2, padx=5, pady=5)
        
        # 墙壁传感器数据
        wall_frame = ttk.LabelFrame(self.sensor_frame, text="墙壁传感器 (HC-SR04)")
        wall_frame.pack(pady=10, padx=10, fill="x")
        
        ttk.Label(wall_frame, text="模拟距离 (米):").grid(row=0, column=0, padx=5, pady=5)
        self.wall_slider = ttk.Scale(wall_frame, from_=0.0, to=5.0, orient=tk.HORIZONTAL, 
                                   command=self.update_wall_distance)
        self.wall_slider.set(1.0)
        self.wall_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.wall_label = ttk.Label(wall_frame, text="1.0米")
        self.wall_label.grid(row=0, column=2, padx=5, pady=5)
        
        # 历史数据图表
        hist_frame = ttk.LabelFrame(self.sensor_frame, text="传感器历史数据")
        hist_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        self.sensor_fig = Figure(figsize=(10, 4))
        self.sensor_ax = self.sensor_fig.add_subplot(111)
        self.sensor_ax.set_xlabel('时间 (秒)')
        self.sensor_ax.set_ylabel('距离 (米)')
        self.sensor_ax.set_title('传感器历史数据')
        
        # 初始数据
        self.time_data = np.arange(0, 100)
        self.height_data = np.ones(100) * 2.8
        self.wall_data = np.ones(100) * 1.0
        
        # 创建线条
        self.height_line, = self.sensor_ax.plot(self.time_data, self.height_data, 'b-', label='高度')
        self.wall_line, = self.sensor_ax.plot(self.time_data, self.wall_data, 'r-', label='墙壁距离')
        self.sensor_ax.legend()
        
        # 设置画布
        self.sensor_canvas = FigureCanvasTkAgg(self.sensor_fig, hist_frame)
        self.sensor_canvas.draw()
        self.sensor_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def _init_comm_panel(self):
        """初始化通信面板"""
        # FCM命令列表
        fcm_frame = ttk.LabelFrame(self.comm_frame, text="FCM通信")
        fcm_frame.pack(pady=10, padx=10, fill="both", expand=True)
        
        # 命令列表
        ttk.Label(fcm_frame, text="发送到FCM的命令:").pack(anchor="w", padx=5, pady=5)
        
        # 创建列表框和滚动条
        fcm_scroll = ttk.Scrollbar(fcm_frame)
        fcm_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.fcm_listbox = tk.Listbox(fcm_frame, yscrollcommand=fcm_scroll.set, height=10)
        self.fcm_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        fcm_scroll.config(command=self.fcm_listbox.yview)
        
        # 无线通信
        wireless_frame = ttk.LabelFrame(self.comm_frame, text="无线通信")
        wireless_frame.pack(pady=10, padx=10, fill="x")
        
        ttk.Label(wireless_frame, text="Wi-Fi状态:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.wifi_status = ttk.Label(wireless_frame, text="已连接")
        self.wifi_status.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(wireless_frame, text="信号强度:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.wifi_signal = ttk.Label(wireless_frame, text="-45 dBm")
        self.wifi_signal.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(wireless_frame, text="任务完成消息:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.mission_msg = ttk.Label(wireless_frame, text="等待接收...")
        self.mission_msg.grid(row=2, column=1, padx=5, pady=5, sticky="w")
    
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
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        log_entry = f"[{timestamp}] {message}\n"
        
        # 在UI线程中更新日志
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)  # 自动滚动到最新日志
    
    def clear_log(self):
        """清除日志"""
        self.log_text.delete(1.0, tk.END)
    
    def update_height(self, value):
        """更新高度传感器数据"""
        value = float(value)
        with self.lock:
            self.vl53l0x_distance = value
        self.height_label.config(text=f"{value:.1f}米")
        
        # 更新图表数据
        self.height_data = np.roll(self.height_data, -1)
        self.height_data[-1] = value
        self.height_line.set_ydata(self.height_data)
        self.sensor_canvas.draw()
    
    def update_wall_distance(self, value):
        """更新墙壁传感器数据"""
        value = float(value)
        with self.lock:
            self.hcsr04_distance = value
        self.wall_label.config(text=f"{value:.1f}米")
        
        # 更新图表数据
        self.wall_data = np.roll(self.wall_data, -1)
        self.wall_data[-1] = value
        self.wall_line.set_ydata(self.wall_data)
        self.sensor_canvas.draw()
    
    def update_battery_voltage(self, value):
        """更新电池电压"""
        value = float(value)
        with self.lock:
            self.battery_voltage = value
        self.battery_label.config(text=f"{value:.1f}V")
    
    def update_drone_position(self, x, y, z):
        """更新无人机位置"""
        with self.lock:
            self.drone_position = [x, y, z]
        
        # 更新3D图表
        self.drone_point.remove()
        self.drone_point = self.ax.scatter([x], [y], [z], c='b', marker='^', s=100)
        self.canvas.draw()
    
    def register_fcm_command(self, command, data=None):
        """记录发送到FCM的命令"""
        cmd_str = f"命令: {command}"
        if data:
            if isinstance(data, (tuple, list)):
                # 格式化数值数据
                data_str = ", ".join([f"{val:.2f}" if isinstance(val, float) else str(val) for val in data])
                cmd_str += f", 数据: ({data_str})"
            elif isinstance(data, dict):
                # 格式化字典数据
                cmd_str += f", 数据: {json.dumps(data, ensure_ascii=False)}"
        
        # 添加到命令队列
        self.fcm_commands.append(cmd_str)
        
        # 更新界面
        self.fcm_listbox.insert(tk.END, cmd_str)
        self.fcm_listbox.see(tk.END)  # 自动滚动到最新命令
        
        # 记录日志
        self.log(f"FCM命令: {cmd_str}")
        
        # 处理位置命令
        if command == "SET_POSITION" and isinstance(data, (tuple, list)) and len(data) == 3:
            x, y, z = data
            self.update_drone_position(x, y, z)
    
    def register_mission_complete(self, message):
        """注册任务完成消息"""
        with self.lock:
            self.mission_complete_msg = message
        
        # 更新界面
        self.mission_msg.config(text=message)
        
        # 记录日志
        self.log(f"任务完成消息: {message}")
    
    def press_button(self):
        """模拟按下按钮"""
        self.log("按钮被按下")
        # 这里可以添加按钮事件处理逻辑
    
    def emergency_landing(self):
        """模拟紧急降落"""
        self.log("紧急降落触发")
        # 添加降落处理逻辑
        # 将高度设置为接近地面
        self.height_slider.set(0.1)
        # 添加紧急命令
        self.register_fcm_command("EMERGENCY_STOP")
    
    def start_simulation(self):
        """启动模拟器"""
        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.log("模拟器启动")
        
        # 这里可以启动模拟线程
    
    def stop_simulation(self):
        """停止模拟器"""
        self.running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.log("模拟器停止")
        
        # 这里可以停止模拟线程
    
    def run(self):
        """运行模拟器"""
        # 进入主循环
        self.root.mainloop()


class MachineSimulator:
    """模拟MicroPython的machine模块"""
    class Pin:
        """模拟Pin类"""
        IN = 0
        OUT = 1
        PULL_UP = 2
        
        def __init__(self, pin_id, mode=IN, pull=None):
            self.pin_id = pin_id
            self.mode = mode
            self.pull = pull
            self._value = 0
            # 获取硬件模拟器引用
            self.simulator = hw_simulator
            self.simulator.log(f"创建Pin {pin_id}, 模式={mode}")
        
        def value(self, val=None):
            if val is not None:
                self._value = val
                # 处理LED状态
                if self.pin_id == 2:  # 假设引脚2是LED
                    self.simulator.led_state = val
                return None
            return self._value
    
    class I2C:
        """模拟I2C接口"""
        def __init__(self, id, scl=None, sda=None, freq=100000):
            self.id = id
            self.scl = scl
            self.sda = sda
            self.freq = freq
            # 获取硬件模拟器引用
            self.simulator = hw_simulator
            self.simulator.log(f"创建I2C {id}, 频率={freq}")
        
        def scan(self):
            # 模拟设备扫描，返回FCM和VL53L0X的地址
            return [0x08, 0x29]
        
        def readfrom_mem(self, addr, memaddr, nbytes):
            # 根据不同地址返回模拟数据
            if addr == 0x29:  # VL53L0X
                if memaddr == 0xC0:  # ID寄存器
                    return bytes([0xEE, 0xAA])
                elif memaddr == 0x14:  # 测量状态寄存器
                    return bytes([0x01])
                elif memaddr == 0x14 + 10:  # 距离寄存器
                    # 将距离值转换为毫米并返回16位值
                    with self.simulator.lock:
                        mm = int(self.simulator.vl53l0x_distance * 1000)
                    return bytes([(mm >> 8) & 0xFF, mm & 0xFF])
            
            # 默认返回0
            return bytes([0] * nbytes)
        
        def writeto_mem(self, addr, memaddr, data):
            # 模拟写入寄存器
            pass
        
        def writeto(self, addr, data):
            # 模拟向设备写入数据
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
                    
                    # 注册命令
                    self.simulator.register_fcm_command(cmd_name, cmd_data)
    
    class ADC:
        """模拟ADC"""
        ATTN_0DB = 0
        ATTN_2_5DB = 1
        ATTN_6DB = 2
        ATTN_11DB = 3
        
        def __init__(self, pin):
            self.pin = pin
            self.attenuation = self.ATTN_0DB
            # 获取硬件模拟器引用
            self.simulator = hw_simulator
            self.simulator.log(f"创建ADC，引脚={pin.pin_id}")
        
        def atten(self, attenuation):
            self.attenuation = attenuation
        
        def read(self):
            # 模拟ADC读数，根据电池电压返回对应的数值
            # 假设满量程16V对应4095
            with self.simulator.lock:
                voltage = self.simulator.battery_voltage
            
            # 根据衰减计算ADC读数
            max_voltage = 1.0  # 默认1V满量程
            if self.attenuation == self.ATTN_2_5DB:
                max_voltage = 1.34
            elif self.attenuation == self.ATTN_6DB:
                max_voltage = 2.0
            elif self.attenuation == self.ATTN_11DB:
                max_voltage = 3.6
            
            # 计算ADC原始值（假设电压被分压器除以5）
            voltage_divided = voltage / 5.0
            adc_value = int((voltage_divided / max_voltage) * 4095)
            
            # 确保在范围内
            adc_value = max(0, min(adc_value, 4095))
            
            return adc_value


class NetworkSimulator:
    """模拟MicroPython的network模块"""
    STA_IF = 0
    AP_IF = 1
    
    class WLAN:
        """模拟WLAN类"""
        def __init__(self, interface):
            self.interface = interface
            self._active = False
            self._connected = False
            # 获取硬件模拟器引用
            self.simulator = hw_simulator
            self.simulator.log(f"创建WLAN，接口类型={interface}")
        
        def active(self, is_active=None):
            if is_active is not None:
                self._active = is_active
                return None
            return self._active
        
        def connect(self, ssid, password):
            # 模拟Wi-Fi连接
            self._connected = True
            self.simulator.log(f"连接到Wi-Fi: {ssid}")
            self.simulator.wifi_status.config(text=f"已连接到 {ssid}")
        
        def isconnected(self):
            return self._connected
        
        def ifconfig(self):
            # 返回IP配置
            return ("192.168.4.2", "255.255.255.0", "192.168.4.1", "8.8.8.8")
        
        def disconnect(self):
            self._connected = False
            self.simulator.log("从Wi-Fi断开连接")
            self.simulator.wifi_status.config(text="已断开")
        
        def status(self, param=None):
            if param == 'rssi':
                return -45  # 模拟信号强度


class SocketSimulator:
    """模拟MicroPython的socket模块"""
    AF_INET = 0
    SOCK_STREAM = 0
    
    class socket:
        """模拟socket类"""
        def __init__(self, family=0, type=0):
            self.family = family
            self.type = type
            self._connected = False
            self._closed = False
            # 获取硬件模拟器引用
            self.simulator = hw_simulator
            self.simulator.log("创建socket")
        
        def connect(self, address):
            # 模拟连接到服务器
            host, port = address
            self._connected = True
            self.simulator.log(f"连接到服务器 {host}:{port}")
        
        def send(self, data):
            # 模拟发送数据
            message = data.decode('utf-8')
            self.simulator.register_mission_complete(message)
            self.simulator.log(f"发送数据: {message}")
            return len(data)
        
        def recv(self, bufsize):
            # 模拟接收数据
            return b"OK"
        
        def settimeout(self, timeout):
            # 模拟设置超时
            pass
        
        def close(self):
            # 模拟关闭连接
            self._closed = True
            self._connected = False
            self.simulator.log("关闭socket连接")


class UbinasciiSimulator:
    """模拟MicroPython的ubinascii模块"""
    @staticmethod
    def hexlify(data):
        # 将二进制数据转换为十六进制字符串
        return bytes("".join([f"{b:02x}" for b in data]), 'ascii')


class UjsonSimulator:
    """模拟MicroPython的ujson模块"""
    @staticmethod
    def dumps(obj):
        # 将对象转换为JSON字符串
        return json.dumps(obj, ensure_ascii=False)
    
    @staticmethod
    def loads(s):
        # 将JSON字符串转换为对象
        return json.loads(s)


class TimeSimulator:
    """模拟MicroPython的time模块"""
    def __init__(self):
        self.start_time = time.time()
        self.ticks_ms_counter = 0
        self.ticks_us_counter = 0
    
    def sleep(self, seconds):
        # 不实际睡眠，只记录
        hw_simulator.log(f"sleep({seconds})")
    
    def sleep_us(self, us):
        # 不实际睡眠，只记录
        pass
    
    def ticks_ms(self):
        # 返回毫秒计数器
        self.ticks_ms_counter += 10
        return self.ticks_ms_counter
    
    def ticks_us(self):
        # 返回微秒计数器
        self.ticks_us_counter += 100
        return self.ticks_us_counter
    
    def ticks_diff(self, end, start):
        # 计算两个时间点之间的差
        return end - start
    
    def localtime(self):
        # 返回当前时间的元组
        t = time.localtime()
        return t


class UosSimulator:
    """模拟MicroPython的uos模块"""
    @staticmethod
    def listdir():
        # 模拟文件列表
        return ["drone_log.txt"]


class MicropythonSimulator:
    """模拟MicroPython的micropython模块"""
    @staticmethod
    def const(value):
        # 模拟const函数
        return value


class EnumSimulator:
    """模拟Python标准库的enum模块"""
    class Enum:
        """模拟Enum类"""
        pass


class GcSimulator:
    """模拟MicroPython的gc模块"""
    @staticmethod
    def collect():
        # 模拟内存垃圾回收
        hw_simulator.log("执行内存垃圾回收")


class MathSimulator:
    """模拟math模块"""
    @staticmethod
    def isclose(a, b, *, rel_tol=1e-9, abs_tol=0.0):
        return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)


# 创建硬件模拟器实例
hw_simulator = HardwareSimulator()

# 定义模拟模块
machine = MachineSimulator()
network = NetworkSimulator()
socket = SocketSimulator()
ubinascii = UbinasciiSimulator()
ujson = UjsonSimulator()
time_sim = TimeSimulator()
uos = UosSimulator()
micropython = MicropythonSimulator()
enum_sim = EnumSimulator()
gc = GcSimulator()
math_sim = MathSimulator()

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

def run_simulator():
    """运行硬件模拟器"""
    hw_simulator.run()

if __name__ == "__main__":
    run_simulator()