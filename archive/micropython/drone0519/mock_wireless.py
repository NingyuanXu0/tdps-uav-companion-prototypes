"""
无线通信模拟器 - 模拟Wi-Fi通信功能
用于在计算机上测试ACM代码
"""
import time
import random
import json

class MockWirelessCommunication:
    """模拟无线通信模块"""
    
    def __init__(self, success_rate=0.95):
        """
        初始化模拟无线通信
        
        Args:
            success_rate: 连接成功率 (0-1)
        """
        self.initialized = False
        self.connected = False
        self.success_rate = success_rate
        self.connection_attempts = 0
        self.max_connection_attempts = 5
        self.transmitted_messages = []
        print("模拟无线通信模块已创建")
    
    def initialize(self):
        """初始化无线硬件"""
        # 模拟90%初始化成功率
        self.initialized = random.random() < 0.9
        if self.initialized:
            print("无线通信模块初始化成功")
        else:
            print("无线通信模块初始化失败")
        return self.initialized
    
    def connect(self, timeout=10):
        """
        连接到Wi-Fi网络
        
        Args:
            timeout: 连接超时时间（秒）
        
        Returns:
            连接成功返回True，否则返回False
        """
        if not self.initialized:
            if not self.initialize():
                return False
        
        # 如果已连接，直接返回成功
        if self.connected:
            print("已经连接到Wi-Fi网络")
            return True
        
        # 增加连接尝试次数
        self.connection_attempts += 1
        if self.connection_attempts > self.max_connection_attempts:
            print(f"达到最大连接尝试次数 ({self.max_connection_attempts})")
            return False
        
        # 模拟连接过程
        print(f"尝试连接到Wi-Fi网络 (尝试 {self.connection_attempts}/{self.max_connection_attempts})")
        
        # 模拟连接延迟和成功率
        time.sleep(random.uniform(0.5, 2.0))  # 模拟连接延迟
        
        # 根据成功率决定连接是否成功
        connection_success = random.random() < self.success_rate
        
        if connection_success:
            self.connected = True
            print("成功连接到Wi-Fi网络，IP地址: 192.168.1.100")
            self.connection_attempts = 0  # 重置连接尝试计数
            return True
        else:
            print("连接到Wi-Fi网络失败")
            return False
    
    def send_mission_complete(self, message):
        """
        发送任务完成消息
        
        Args:
            message: 发送的消息内容
        
        Returns:
            发送成功返回True，否则返回False
        """
        if not self.connected:
            if not self.connect():
                return False
        
        # 模拟消息发送
        print(f"正在发送消息: {message}")
        
        # 记录发送的消息
        self.transmitted_messages.append({
            'time': time.time(),
            'message': message
        })
        
        # 模拟通信延迟
        time.sleep(random.uniform(0.2, 1.0))
        
        # 90%发送成功率
        send_success = random.random() < 0.9
        
        if send_success:
            print("消息发送成功，服务器响应: ACK")
            return True
        else:
            print("消息发送失败")
            return False
    
    def disconnect(self):
        """断开Wi-Fi连接"""
        if self.connected:
            print("从Wi-Fi网络断开")
            self.connected = False
        else:
            print("当前未连接")
    
    def get_signal_strength(self):
        """获取当前Wi-Fi信号强度（RSSI）"""
        if self.connected:
            # 模拟-30 到 -80 dBm的信号强度
            return -30 - random.randint(0, 50)
        return None
    
    def save_transmission_log(self, filename="wireless_transmission_log.json"):
        """保存传输日志到文件"""
        try:
            with open(filename, 'w') as f:
                json.dump(self.transmitted_messages, f, indent=2)
            print(f"传输日志已保存到 {filename}")
        except Exception as e:
            print(f"保存传输日志出错: {e}")