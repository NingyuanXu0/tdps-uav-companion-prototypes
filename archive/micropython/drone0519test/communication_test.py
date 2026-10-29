"""
communication.py 的单元测试
测试FCM接口和无线通信功能
"""
import unittest
from unittest.mock import patch, MagicMock, call
import time
import sys
import json

# 模拟依赖
sys.modules['machine'] = MagicMock()
sys.modules['network'] = MagicMock()
sys.modules['socket'] = MagicMock()
sys.modules['ubinascii'] = MagicMock()
sys.modules['ujson'] = MagicMock()
sys.modules['config'] = MagicMock()
sys.modules['utils'] = MagicMock()
sys.modules['utils'].log = MagicMock()

# 配置模拟
sys.modules['config'].I2C_SCL_PIN = 22
sys.modules['config'].I2C_SDA_PIN = 21
sys.modules['config'].I2C_FREQUENCY = 400000
sys.modules['config'].FCM_I2C_ADDRESS = 0x08
sys.modules['config'].FCM_COMMANDS = {
    "MOTORS_ON": 0x01,
    "MOTORS_OFF": 0x00,
    "SET_POSITION": 0x02,
    "SET_VELOCITY": 0x03,
    "SET_ALTITUDE": 0x05,
    "SET_ATTITUDE": 0x04,
    "EMERGENCY_STOP": 0xFF
}
sys.modules['config'].WIFI_SSID = "TEST_SSID"
sys.modules['config'].WIFI_PASSWORD = "TEST_PASSWORD"
sys.modules['config'].SERVER_IP = "192.168.4.1"
sys.modules['config'].SERVER_PORT = 8080
sys.modules['config'].WIFI_CONNECTION_TIMEOUT = 10
sys.modules['config'].SOCKET_TIMEOUT = 5

sys.modules['ujson'].dumps = json.dumps

# 导入被测试模块
import communication

class TestFCMInterface(unittest.TestCase):
    """测试FCM接口类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟I2C总线和设备扫描结果
        self.mock_i2c = MagicMock()
        self.mock_i2c.scan.return_value = [0x08]  # FCM地址
        
        # 打补丁
        patch('machine.I2C', return_value=self.mock_i2c).start()
        patch('machine.Pin').start()
        
        # 创建FCM接口对象
        self.fcm = communication.FCMInterface()
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialization(self):
        """测试初始化"""
        self.assertTrue(self.fcm.initialized)
        self.assertEqual(self.fcm.cmd_sequence, 0)
        self.assertEqual(self.fcm.retry_count, 0)
        self.assertEqual(self.fcm.max_retries, 3)
    
    def test_send_command_with_no_data(self):
        """测试发送无数据命令"""
        # 发送电机开启命令
        result = self.fcm.send_command("MOTORS_ON")
        
        # 验证命令发送
        self.assertTrue(result)
        self.mock_i2c.writeto.assert_called_once()
        
        # 验证命令序列增加
        self.assertEqual(self.fcm.cmd_sequence, 1)
    
    def test_send_command_with_numerical_data(self):
        """测试发送带数值数据的命令"""
        # 测试设置位置命令
        result = self.fcm.send_command("SET_POSITION", (1.0, 2.0, 3.0))
        
        # 验证命令发送
        self.assertTrue(result)
        self.mock_i2c.writeto.assert_called_once()
    
    def test_send_command_with_dict_data(self):
        """测试发送带字典数据的命令"""
        # 测试发送JSON数据
        test_data = {"mode": "auto", "speed": 50}
        result = self.fcm.send_command("SET_VELOCITY", test_data)
        
        # 验证命令发送
        self.assertTrue(result)
        self.mock_i2c.writeto.assert_called_once()
    
    def test_motors_on(self):
        """测试开启电机"""
        self.fcm.send_command = MagicMock(return_value=True)
        result = self.fcm.motors_on()
        self.assertTrue(result)
        self.fcm.send_command.assert_called_with("MOTORS_ON")
    
    def test_motors_off(self):
        """测试关闭电机"""
        self.fcm.send_command = MagicMock(return_value=True)
        result = self.fcm.motors_off()
        self.assertTrue(result)
        self.fcm.send_command.assert_called_with("MOTORS_OFF")
    
    def test_emergency_stop(self):
        """测试紧急停止"""
        self.fcm.send_command = MagicMock(return_value=True)
        result = self.fcm.emergency_stop()
        self.assertTrue(result)
        # 应该调用两次紧急停止命令
        self.assertEqual(self.fcm.send_command.call_count, 2)
    
    def test_set_position(self):
        """测试设置位置"""
        self.fcm.send_command = MagicMock(return_value=True)
        result = self.fcm.set_position(1, 2, 3)
        self.assertTrue(result)
        self.fcm.send_command.assert_called_with("SET_POSITION", (1, 2, 3))
    
    def test_set_velocity(self):
        """测试设置速度"""
        self.fcm.send_command = MagicMock(return_value=True)
        result = self.fcm.set_velocity(1, 2, 3)
        self.assertTrue(result)
        self.fcm.send_command.assert_called_with("SET_VELOCITY", (1, 2, 3))


class TestWirelessCommunication(unittest.TestCase):
    """测试无线通信类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟WLAN接口
        self.mock_wlan = MagicMock()
        self.mock_wlan.active.return_value = True
        self.mock_wlan.isconnected.return_value = False
        
        # 模拟socket
        self.mock_socket = MagicMock()
        self.mock_socket_instance = MagicMock()
        self.mock_socket.return_value = self.mock_socket_instance
        self.mock_socket_instance.recv.return_value = b"OK"
        
        # 模拟machine.unique_id
        self.mock_unique_id = MagicMock(return_value=b'\x01\x02\x03\x04')
        
        # 打补丁
        patch('network.WLAN', return_value=self.mock_wlan).start()
        patch('network.STA_IF', 0).start()
        patch('socket.socket', self.mock_socket).start()
        patch('socket.AF_INET', 0).start()
        patch('socket.SOCK_STREAM', 0).start()
        patch('machine.unique_id', self.mock_unique_id).start()
        patch('ubinascii.hexlify', return_value=b'01020304').start()
        patch('time.time', side_effect=[0, 5, 10]).start()
        
        # 创建无线通信对象
        self.wireless = communication.WirelessCommunication()
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialization(self):
        """测试初始化"""
        self.assertFalse(self.wireless.initialized)
        self.assertFalse(self.wireless.connected)
        self.assertEqual(self.wireless.connection_attempts, 0)
        self.assertEqual(self.wireless.max_connection_attempts, 5)
    
    def test_initialize(self):
        """测试初始化无线模块"""
        result = self.wireless.initialize()
        self.assertTrue(result)
        self.assertTrue(self.wireless.initialized)
        self.mock_wlan.active.assert_called_with(True)
    
    def test_connect_already_connected(self):
        """测试连接Wi-Fi（已连接情况）"""
        # 设置已连接状态
        self.wireless.initialized = True
        self.mock_wlan.isconnected.return_value = True
        
        result = self.wireless.connect()
        
        self.assertTrue(result)
        self.assertTrue(self.wireless.connected)
    
    def test_connect_success(self):
        """测试连接Wi-Fi（成功连接）"""
        # 设置未连接但能成功连接
        self.wireless.initialized = True
        self.mock_wlan.isconnected.side_effect = [False, True]
        self.mock_wlan.ifconfig.return_value = ["192.168.4.2", "255.255.255.0", "192.168.4.1", "8.8.8.8"]
        
        result = self.wireless.connect()
        
        self.assertTrue(result)
        self.assertTrue(self.wireless.connected)
        self.mock_wlan.connect.assert_called_once_with("TEST_SSID", "TEST_PASSWORD")
    
    def test_send_mission_complete(self):
        """测试发送任务完成消息"""
        # 设置已连接状态
        self.wireless.connected = True
        
        result = self.wireless.send_mission_complete("任务完成")
        
        self.assertTrue(result)
        # 检查socket连接和发送
        self.mock_socket.assert_called_once()
        self.mock_socket_instance.connect.assert_called_once_with(("192.168.4.1", 8080))
        self.mock_socket_instance.send.assert_called_once()
        self.mock_socket_instance.recv.assert_called_once()
        self.mock_socket_instance.close.assert_called_once()
    
    def test_disconnect(self):
        """测试断开Wi-Fi连接"""
        # 设置已连接状态
        self.wireless.connected = True
        
        self.wireless.disconnect()
        
        self.assertFalse(self.wireless.connected)
        self.mock_wlan.disconnect.assert_called_once()


if __name__ == '__main__':
    unittest.main()