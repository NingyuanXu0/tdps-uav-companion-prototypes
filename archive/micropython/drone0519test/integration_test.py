"""
集成测试模块
测试ACM系统各模块之间的集成
"""
import enum
import unittest
from unittest.mock import patch, MagicMock
import time
import sys
import io

# 确保能导入所有模块
sys.path.append('.')

# 注册必要的模拟模块
sys.modules['machine'] = MagicMock()
sys.modules['network'] = MagicMock()
sys.modules['socket'] = MagicMock()
sys.modules['ubinascii'] = MagicMock()
sys.modules['ujson'] = MagicMock()
sys.modules['uos'] = MagicMock()
sys.modules['micropython'] = MagicMock()
sys.modules['enum'] = MagicMock()
sys.modules['gc'] = MagicMock()
sys.modules['math'] = MagicMock()

# 导入真实模块
import config
import utils
import communication
import perception
import task_manager

# 为枚举类做准备
class TaskState(enum.Enum):
    INITIALIZING = 0
    WAITING_FOR_START = 1
    TUNNELLING = 2
    OBSTACLE_AVOIDANCE = 3
    PACKAGE_SEARCH = 4
    PACKAGE_PICKUP = 5
    MARKER_SEARCH = 6
    PACKAGE_DROPOFF = 7
    LANDING = 8
    MISSION_COMPLETE = 9
    ERROR = 10
    ABORT = 11

# 覆盖任务状态枚举
task_manager.TaskState = TaskState

class TestACMIntegration(unittest.TestCase):
    """ACM系统集成测试"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟硬件和依赖
        self.mock_i2c = MagicMock()
        self.mock_i2c.scan.return_value = [0x08, 0x29]  # FCM和VL53L0X地址
        self.mock_i2c.readfrom_mem.return_value = bytes([0xEE, 0xAA])  # 有效传感器ID
        
        self.mock_pin = MagicMock()
        self.mock_adc = MagicMock()
        self.mock_adc.read.return_value = 3000  # 模拟ADC读数
        
        self.mock_wlan = MagicMock()
        self.mock_wlan.active.return_value = True
        self.mock_wlan.isconnected.return_value = True
        self.mock_wlan.ifconfig.return_value = ["192.168.4.2", "255.255.255.0", "192.168.4.1", "8.8.8.8"]
        
        self.mock_socket = MagicMock()
        self.mock_socket_instance = MagicMock()
        self.mock_socket.return_value = self.mock_socket_instance
        self.mock_socket_instance.recv.return_value = b"OK"
        
        # 打补丁
        patch('machine.I2C', return_value=self.mock_i2c).start()
        patch('machine.Pin', return_value=self.mock_pin).start()
        patch('machine.ADC', return_value=self.mock_adc).start()
        patch('network.WLAN', return_value=self.mock_wlan).start()
        patch('network.STA_IF', 0).start()
        patch('socket.socket', self.mock_socket).start()
        patch('socket.AF_INET', 0).start()
        patch('socket.SOCK_STREAM', 0).start()
        patch('time.sleep').start()
        patch('time.sleep_us').start()
        patch('time.ticks_ms', side_effect=range(0, 10000, 100)).start()
        patch('time.ticks_us', side_effect=range(0, 100000, 10)).start()
        patch('time.ticks_diff', return_value=10).start()
        patch('time.time', side_effect=range(0, 100, 1)).start()
        
        # 捕获输出
        self.held_output = io.StringIO()
        sys.stdout = self.held_output
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
        sys.stdout = sys.__stdout__
    
    def test_perception_communication_integration(self):
        """测试感知系统和通信模块的集成"""
        # 创建真实的感知系统和通信模块实例
        percept_system = perception.PerceptionSystem()
        fcm_interface = communication.FCMInterface()
        
        # 确保初始化成功
        self.assertTrue(fcm_interface.initialized)
        
        # 模拟感知系统获取高度
        percept_system.height_sensor = MagicMock()
        percept_system.height_sensor.initialized = True
        percept_system.height_sensor.get_distance.return_value = 2.5
        percept_system.initialized = True
        
        # 从感知系统获取高度并通过FCM接口发送命令
        height = percept_system.get_height()
        self.assertEqual(height, 2.5)
        
        # 尝试发送高度控制命令
        result = fcm_interface.set_altitude(height)
        self.assertTrue(result)
        
        # 验证I2C写入调用
        self.mock_i2c.writeto.assert_called()
    
    def test_task_manager_perception_integration(self):
        """测试任务管理器和感知系统的集成"""
        # 创建感知系统和FCM接口实例
        percept_system = perception.PerceptionSystem()
        fcm_interface = communication.FCMInterface()
        wireless_comm = communication.WirelessCommunication()
        
        # 模拟感知系统
        percept_system.initialized = True
        percept_system.initialize_sensors = MagicMock(return_value=True)
        percept_system.calibrate_sensors = MagicMock(return_value=True)
        percept_system.get_current_position = MagicMock(return_value=[0, 0, 2.8])
        percept_system.get_height = MagicMock(return_value=2.8)
        percept_system.get_wall_distance = MagicMock(return_value=1.0)
        
        # 创建任务管理器
        tm = task_manager.TaskManager(fcm_interface, percept_system, wireless_comm)
        
        # 初始化任务管理器
        result = tm.initialize()
        self.assertTrue(result)
        
        # 验证状态和初始化调用
        self.assertEqual(tm.state, TaskState.WAITING_FOR_START)
        percept_system.initialize_sensors.assert_called_once()
        percept_system.calibrate_sensors.assert_called_once()
        
        # 模拟任务更新
        utils.is_position_reached = MagicMock(return_value=False)
        tm.state = TaskState.TUNNELLING
        
        # 更新任务状态
        result = tm.update()
        
        # 验证更新过程
        self.assertTrue(result)
        percept_system.get_current_position.assert_called()
        percept_system.get_height.assert_called()
    
    def test_wireless_communication_integration(self):
        """测试无线通信功能集成"""
        # 创建无线通信实例
        wireless_comm = communication.WirelessCommunication()
        
        # 初始化并连接
        wireless_comm.initialize()
        self.assertTrue(wireless_comm.initialized)
        
        result = wireless_comm.connect()
        self.assertTrue(result)
        self.assertTrue(wireless_comm.connected)
        
        # 发送任务完成消息
        result = wireless_comm.send_mission_complete("任务完成测试")
        self.assertTrue(result)
        
        # 验证socket连接和发送
        self.mock_socket.assert_called_once()
        self.mock_socket_instance.connect.assert_called_once()
        self.mock_socket_instance.send.assert_called_once()
        
        # 断开连接
        wireless_comm.disconnect()
        self.assertFalse(wireless_comm.connected)
    
    def test_task_manager_full_workflow(self):
        """测试任务管理器完整工作流程"""
        # 创建相关组件
        fcm_interface = communication.FCMInterface()
        percept_system = perception.PerceptionSystem()
        wireless_comm = communication.WirelessCommunication()
        
        # 配置模拟
        percept_system.initialized = True
        percept_system.initialize_sensors = MagicMock(return_value=True)
        percept_system.calibrate_sensors = MagicMock(return_value=True)
        percept_system.get_current_position = MagicMock(return_value=[0, 0, 2.8])
        percept_system.get_height = MagicMock(return_value=2.8)
        percept_system.get_wall_distance = MagicMock(return_value=1.0)
        
        utils.is_position_reached = MagicMock(return_value=True)
        
        # 创建任务管理器
        tm = task_manager.TaskManager(fcm_interface, percept_system, wireless_comm)
        
        # 初始化
        tm.initialize()
        self.assertEqual(tm.state, TaskState.WAITING_FOR_START)
        
        # 测试任务转换序列
        # 1. 等待开始 -> 隧道穿越
        tm.update()
        self.assertEqual(tm.state, TaskState.TUNNELLING)
        
        # 2. 穿越隧道
        # 模拟已完成所有隧道
        tm.current_waypoint_index = config.TASK_TRANSITION_INDICES["TUNNELLING_COMPLETE"]
        tm.update()
        self.assertEqual(tm.state, TaskState.OBSTACLE_AVOIDANCE)
        
        # 3. 障碍物避开
        # 模拟已完成所有障碍物
        tm.current_waypoint_index = config.TASK_TRANSITION_INDICES["OBSTACLE_AVOIDANCE_COMPLETE"]
        tm.update()
        self.assertEqual(tm.state, TaskState.PACKAGE_SEARCH)
        
        # 4. 包裹搜索 -> 包裹拾取
        tm.update()
        self.assertEqual(tm.state, TaskState.PACKAGE_PICKUP)
        
        # 5. 包裹拾取
        tm.update()
        self.assertTrue(tm.package_picked_up)
        
        # 模拟包裹拾取完成
        tm.current_waypoint_index = config.TASK_TRANSITION_INDICES["PICKUP_COMPLETE"]
        tm.update()
        self.assertEqual(tm.state, TaskState.MARKER_SEARCH)
        
        # 6. 标记搜索 -> 包裹放置
        tm.update()
        self.assertEqual(tm.state, TaskState.PACKAGE_DROPOFF)
        
        # 7. 包裹放置
        tm.update()
        self.assertTrue(tm.package_dropped_off)
        
        # 模拟包裹放置完成
        tm.current_waypoint_index = config.TASK_TRANSITION_INDICES["DROPOFF_COMPLETE"]
        tm.update()
        self.assertEqual(tm.state, TaskState.LANDING)
        
        # 8. 降落
        # 模拟已接近地面
        percept_system.get_height.return_value = 0.05
        tm.update()
        self.assertEqual(tm.state, TaskState.MISSION_COMPLETE)
        
        # 9. 任务完成
        # 配置无线通信
        wireless_comm.connect = MagicMock(return_value=True)
        wireless_comm.send_mission_complete = MagicMock(return_value=True)
        
        # 模拟足够的等待时间
        tm.mission_complete_time = 0  # 已经足够长时间
        
        # 更新
        result = tm.update()
        
        # 验证结果
        self.assertFalse(result)  # 任务完成，返回False
        wireless_comm.connect.assert_called_once()
        wireless_comm.send_mission_complete.assert_called_once()


if __name__ == '__main__':
    unittest.main()