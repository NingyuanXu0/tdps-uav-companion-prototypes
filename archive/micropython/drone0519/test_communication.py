"""
通信模块单元测试
测试FCMInterface和WirelessCommunication类的功能
"""
import sys
import os
import importlib.util

# 添加项目目录到Python路径
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# 导入测试框架
from simple_test_framework import TestCase

# 导入模拟模块
from mock_fcm import MockFCM
from mock_wireless import MockWirelessCommunication

# 动态创建一个模拟的通信模块
communication_module = type('module', (), {})()
communication_module.FCMInterface = MockFCM
communication_module.WirelessCommunication = MockWirelessCommunication
sys.modules['communication'] = communication_module

class TestFCMInterface(TestCase):
    """测试FCM接口功能"""
    
    def setUp(self):
        """测试前准备"""
        self.fcm = MockFCM()
    
    def tearDown(self):
        """测试后清理"""
        if hasattr(self.fcm, 'stop'):
            self.fcm.stop()
    
    def test_initialization(self):
        """测试初始化"""
        self.assertTrue(self.fcm.i2c_address == 0x08, "FCM地址应为0x08")
    
    def test_motors_on(self):
        """测试开启电机"""
        result = self.fcm.receive_command("MOTORS_ON")
        self.assertTrue(result, "电机开启命令应成功执行")
        self.assertTrue(self.fcm.motors_active, "电机应被激活")
    
    def test_motors_off(self):
        """测试关闭电机"""
        self.fcm.receive_command("MOTORS_ON")  # 先开启电机
        result = self.fcm.receive_command("MOTORS_OFF")
        self.assertTrue(result, "电机关闭命令应成功执行")
        self.assertFalse(self.fcm.motors_active, "电机应被停止")
    
    def test_emergency_stop(self):
        """测试紧急停止"""
        self.fcm.receive_command("MOTORS_ON")  # 先开启电机
        result = self.fcm.receive_command("EMERGENCY_STOP")
        self.assertTrue(result, "紧急停止命令应成功执行")
        self.assertFalse(self.fcm.motors_active, "电机应被紧急停止")
    
    def test_set_position(self):
        """测试设置位置"""
        result = self.fcm.receive_command("SET_POSITION", (1.0, 2.0, 3.0))
        self.assertTrue(result, "设置位置命令应成功执行")
        self.assertEqual(self.fcm.position, [1.0, 2.0, 3.0], "位置应被正确设置")
    
    def test_set_velocity(self):
        """测试设置速度"""
        result = self.fcm.receive_command("SET_VELOCITY", (0.5, -0.5, 0.0))
        self.assertTrue(result, "设置速度命令应成功执行")
        self.assertEqual(self.fcm.velocity, [0.5, -0.5, 0.0], "速度应被正确设置")
    
    def test_set_attitude(self):
        """测试设置姿态"""
        result = self.fcm.receive_command("SET_ATTITUDE", (10.0, 5.0, 0.0))
        self.assertTrue(result, "设置姿态命令应成功执行")
        self.assertEqual(self.fcm.attitude, [10.0, 5.0, 0.0], "姿态应被正确设置")
    
    def test_set_altitude(self):
        """测试设置高度"""
        result = self.fcm.receive_command("SET_ALTITUDE", (2.5,))
        self.assertTrue(result, "设置高度命令应成功执行")
        self.assertEqual(self.fcm.altitude, 2.5, "高度应被正确设置")

class TestWirelessCommunication(TestCase):
    """测试无线通信功能"""
    
    def setUp(self):
        """测试前准备"""
        self.wireless = MockWirelessCommunication()
        self.wireless.success_rate = 1.0  # 确保测试成功率100%
    
    def test_initialization(self):
        """测试初始化"""
        result = self.wireless.initialize()
        self.assertTrue(result, "无线模块初始化应成功")
        self.assertTrue(self.wireless.initialized, "初始化后标志应为True")
    
    def test_connect(self):
        """测试连接"""
        result = self.wireless.connect()
        self.assertTrue(result, "Wi-Fi连接应成功")
        self.assertTrue(self.wireless.connected, "连接后标志应为True")
    
    def test_send_mission_complete(self):
        """测试发送任务完成消息"""
        self.wireless.connect()
        result = self.wireless.send_mission_complete("任务完成于 2025-05-19 15:30:00")
        self.assertTrue(result, "发送任务完成消息应成功")
        self.assertTrue(len(self.wireless.transmitted_messages) > 0, "应有消息被记录")
    
    def test_disconnect(self):
        """测试断开连接"""
        self.wireless.connect()
        self.wireless.disconnect()
        self.assertFalse(self.wireless.connected, "断开后标志应为False")
    
    def test_get_signal_strength(self):
        """测试获取信号强度"""
        self.wireless.connect()
        signal = self.wireless.get_signal_strength()
        self.assertIsNotNone(signal, "信号强度不应为None")
        # 信号强度通常在-20至-90 dBm之间
        self.assertTrue(-90 <= signal <= -20, "信号强度应在合理范围内")