"""
task_manager.py 的单元测试
测试任务管理器和状态机功能
"""
import unittest
from unittest.mock import patch, MagicMock, call
import time
import sys
import enum

# 模拟依赖
sys.modules['machine'] = MagicMock()
sys.modules['enum'] = enum
sys.modules['config'] = MagicMock()
sys.modules['utils'] = MagicMock()
sys.modules['utils'].log = MagicMock()
sys.modules['utils'].display_led_pattern = MagicMock()
sys.modules['utils'].format_mission_complete_message = MagicMock(return_value="任务完成于2024-05-19 12:00:00")
sys.modules['utils'].is_position_reached = MagicMock()
sys.modules['utils'].vector_distance = MagicMock()
sys.modules['math'] = MagicMock()

# 配置模拟
sys.modules['config'].PATH_WAYPOINTS = [
    (0, 0, 2.8),  # 起点，高度为2.8米
    (2, 0, 2.8),  # 第一个灯穿越
    (4, 0, 2.8),  # 第二个灯穿越
    (6, 0, 2.8),  # 第三个灯穿越
    (8, 0, 2.8),  # 从第三个灯出来
    (8, 2, 2.6),  # 右转
    (8, 4, 2.6),  # 通过第一个横梁
    (8, 6, 2.6),  # 通过第二个横梁
    (6, 6, 2.7),  # 接近红色区域
    (6, 6, 0.2),  # 降低高度进行包裹拾取
    (6, 6, 2.7),  # 拾取后上升
    (4, 8, 2.7),  # 接近绿色区域
    (4, 8, 0.2),  # 降低高度进行包裹放置
    (4, 8, 2.7),  # 放置后上升
    (4, 8, 0.1),  # 最终降落
]

sys.modules['config'].TASK_TRANSITION_INDICES = {
    "TAKEOFF_COMPLETE": 1,
    "TUNNELLING_COMPLETE": 4,
    "OBSTACLE_AVOIDANCE_COMPLETE": 8,
    "PICKUP_COMPLETE": 11,
    "DROPOFF_COMPLETE": 14,
    "LANDING_COMPLETE": 15,
}

sys.modules['config'].POSITION_THRESHOLD = 0.2
sys.modules['config'].TAKEOFF_HEIGHT = 2.8
sys.modules['config'].CRUISING_HEIGHT = 2.7
sys.modules['config'].OBSTACLE_AVOIDANCE_HEIGHT = 2.6
sys.modules['config'].PICKUP_HEIGHT = 0.2
sys.modules['config'].DROPOFF_HEIGHT = 0.2
sys.modules['config'].LANDING_HEIGHT = 0.1
sys.modules['config'].MISSION_COMPLETE_WAIT_TIME = 15
sys.modules['config'].BATTERY_ADC_PIN = 34
sys.modules['config'].LOW_BATTERY_THRESHOLD = 14.0
sys.modules['config'].CRITICAL_BATTERY_THRESHOLD = 13.5
sys.modules['config'].BATTERY_VOLTAGE_DIVIDER = 6.0  # Simulated ADC scale only

# 创建Enum类型
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

# 导入并模拟TaskState
sys.modules.pop('task_manager', None)

# 导入被测试模块（需要在其他补丁之后）
import task_manager
TaskState = task_manager.TaskState

class TestTaskManager(unittest.TestCase):
    """测试任务管理器类"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟依赖对象
        self.mock_fcm = MagicMock()
        self.mock_fcm.initialized = True
        
        self.mock_perception = MagicMock()
        self.mock_perception.get_current_position.return_value = [0, 0, 0]
        self.mock_perception.get_wall_distance.return_value = 1.0
        self.mock_perception.get_height.return_value = 2.5
        
        self.mock_wireless = MagicMock()
        
        # 模拟电池ADC
        self.mock_adc = MagicMock()
        self.mock_adc.read.return_value = 3000  # 模拟ADC读数
        
        # 模拟按钮
        self.mock_button = MagicMock()
        
        # 打补丁
        patch('machine.ADC', return_value=self.mock_adc).start()
        patch('machine.Pin', return_value=self.mock_button).start()
        
        # 恢复task_manager模块
        task_manager.TaskState = TaskState
        
        # 创建任务管理器
        self.task_manager = task_manager.TaskManager(
            self.mock_fcm, 
            self.mock_perception, 
            self.mock_wireless
        )
        
        # 配置任务状态
        self.task_manager.task_state = TaskState
        self.task_manager.state = TaskState.INITIALIZING
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialization(self):
        """测试任务管理器初始化"""
        self.assertEqual(self.task_manager.state, TaskState.INITIALIZING)
        self.assertEqual(self.task_manager.current_waypoint_index, 
                         sys.modules['config'].TASK_TRANSITION_INDICES["TAKEOFF_COMPLETE"])
        self.assertIsNone(self.task_manager.mission_start_time)
        self.assertIsNone(self.task_manager.mission_complete_time)
        self.assertFalse(self.task_manager.package_picked_up)
        self.assertFalse(self.task_manager.package_dropped_off)
    
    def test_initialize_method(self):
        """测试初始化方法"""
        # 模拟成功初始化传感器
        self.mock_perception.initialize_sensors.return_value = True
        self.mock_perception.calibrate_sensors.return_value = True
        
        result = self.task_manager.initialize()
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.WAITING_FOR_START)
        self.mock_perception.initialize_sensors.assert_called_once()
        self.mock_perception.calibrate_sensors.assert_called_once()
    
    def test_get_battery_voltage(self):
        """测试获取电池电压"""
        voltage = self.task_manager.get_battery_voltage()
        self.assertIsInstance(voltage, float)
        self.mock_adc.read.assert_called_once()
    
    def test_abort_mission(self):
        """测试中止任务"""
        self.task_manager.abort_mission()
        self.assertEqual(self.task_manager.state, TaskState.ABORT)
    
    def test_emergency_landing(self):
        """测试紧急降落"""
        # 模拟位置获取
        self.mock_perception.get_current_position.return_value = [1, 2, 3]
        
        self.task_manager.emergency_landing()
        
        # 验证FCM命令
        self.mock_fcm.set_position.assert_called_once_with(1, 2, 0.1)
        self.mock_fcm.motors_off.assert_called_once()
    
    def test_handle_tunnelling(self):
        """测试隧道穿越任务处理"""
        # 设置状态
        self.task_manager.state = TaskState.TUNNELLING
        self.task_manager.current_waypoint_index = 2  # 第二个隧道位置
        
        # 模拟未到达目标位置
        sys.modules['utils'].is_position_reached.return_value = False
        
        # 调用处理函数
        result = self.task_manager._handle_tunnelling([0, 0, 2.8], False)
        
        # 验证结果
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.TUNNELLING)
        
        # 模拟到达目标位置，但未完成所有隧道
        sys.modules['utils'].is_position_reached.return_value = True
        
        result = self.task_manager._handle_tunnelling([2, 0, 2.8], True)
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.TUNNELLING)
        self.mock_fcm.set_position.assert_called()
        
        # 模拟到达最后一个隧道位置
        self.task_manager.current_waypoint_index = 4  # 隧道完成索引
        
        result = self.task_manager._handle_tunnelling([6, 0, 2.8], True)
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.OBSTACLE_AVOIDANCE)
    
    def test_handle_obstacle_avoidance(self):
        """测试障碍物避开任务处理"""
        # 设置状态
        self.task_manager.state = TaskState.OBSTACLE_AVOIDANCE
        self.task_manager.current_waypoint_index = 5  # 第一个障碍物位置
        
        # 模拟到达目标位置
        sys.modules['utils'].is_position_reached.return_value = True
        
        # 调用处理函数
        result = self.task_manager._handle_obstacle_avoidance([8, 2, 2.6], True)
        
        # 验证结果
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.OBSTACLE_AVOIDANCE)
        self.mock_fcm.set_position.assert_called()
        
        # 模拟到达最后一个障碍物位置
        self.task_manager.current_waypoint_index = 8  # 障碍物避开完成索引
        
        result = self.task_manager._handle_obstacle_avoidance([8, 6, 2.6], True)
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.PACKAGE_SEARCH)
    
    def test_handle_package_pickup(self):
        """测试包裹拾取任务处理"""
        # 设置状态
        self.task_manager.state = TaskState.PACKAGE_PICKUP
        self.task_manager.current_waypoint_index = 9  # 拾取位置
        
        # 模拟到达目标位置
        sys.modules['utils'].is_position_reached.return_value = True
        
        # 调用处理函数（第一次调用，拾取包裹）
        result = self.task_manager._handle_package_pickup([6, 6, 0.2], True)
        
        # 验证结果
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.PACKAGE_PICKUP)
        self.assertTrue(self.task_manager.package_picked_up)
        
        # 模拟包裹已拾取，继续上升
        self.task_manager.package_picked_up = True
        self.task_manager.current_waypoint_index = 10  # 拾取后上升
        
        result = self.task_manager._handle_package_pickup([6, 6, 2.7], True)
        
        self.assertTrue(result)
        self.mock_fcm.set_position.assert_called()
        
        # 模拟到达拾取完成位置
        self.task_manager.current_waypoint_index = 11  # 拾取完成索引
        
        result = self.task_manager._handle_package_pickup([6, 6, 2.7], True)
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.MARKER_SEARCH)
    
    def test_handle_package_dropoff(self):
        """测试包裹放置任务处理"""
        # 设置状态
        self.task_manager.state = TaskState.PACKAGE_DROPOFF
        self.task_manager.current_waypoint_index = 12  # 放置位置
        self.task_manager.package_picked_up = True  # 已经拾取包裹
        
        # 模拟到达目标位置
        sys.modules['utils'].is_position_reached.return_value = True
        
        # 调用处理函数（第一次调用，放置包裹）
        result = self.task_manager._handle_package_dropoff([4, 8, 0.2], True)
        
        # 验证结果
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.PACKAGE_DROPOFF)
        self.assertTrue(self.task_manager.package_dropped_off)
        
        # 模拟包裹已放置，继续上升
        self.task_manager.package_dropped_off = True
        self.task_manager.current_waypoint_index = 13  # 放置后上升
        
        result = self.task_manager._handle_package_dropoff([4, 8, 2.7], True)
        
        self.assertTrue(result)
        self.mock_fcm.set_position.assert_called()
        
        # 模拟到达放置完成位置
        self.task_manager.current_waypoint_index = 14  # 放置完成索引
        
        result = self.task_manager._handle_package_dropoff([4, 8, 2.7], True)
        
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.LANDING)
    
    def test_handle_landing(self):
        """测试降落任务处理"""
        # 设置状态
        self.task_manager.state = TaskState.LANDING
        
        # 模拟高度传感器读数（仍在空中）
        self.mock_perception.get_height.return_value = 0.5
        
        # 调用处理函数
        result = self.task_manager._handle_landing([4, 8, 0.5], True)
        
        # 验证结果（仍在降落过程中）
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.LANDING)
        self.mock_fcm.set_position.assert_called()
        
        # 模拟已降落到地面
        self.mock_perception.get_height.return_value = 0.05
        
        result = self.task_manager._handle_landing([4, 8, 0.05], True)
        
        # 验证结果（降落完成）
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.MISSION_COMPLETE)
        self.mock_fcm.motors_off.assert_called_once()
        self.assertIsNotNone(self.task_manager.mission_complete_time)
    
    def test_handle_mission_complete(self):
        """测试任务完成状态处理"""
        # 设置状态
        self.task_manager.state = TaskState.MISSION_COMPLETE
        self.task_manager.mission_complete_time = time.time() - 20  # 设置为20秒前
        
        # 模拟Wi-Fi连接
        self.mock_wireless.connect.return_value = True
        self.mock_wireless.send_mission_complete.return_value = True
        
        # 调用处理函数
        result = self.task_manager._handle_mission_complete()
        
        # 验证结果
        self.assertFalse(result)  # 应该返回False表示主循环结束
        self.mock_wireless.connect.assert_called_once()
        self.mock_wireless.send_mission_complete.assert_called_once()
        self.mock_wireless.disconnect.assert_called_once()
    
    def test_update(self):
        """测试整体状态更新"""
        # 设置正常状态下的update
        self.task_manager.state = TaskState.WAITING_FOR_START
        self.mock_perception.get_current_position.return_value = [0, 0, 0]
        
        # 模拟各状态处理函数
        self.task_manager._handle_tunnelling = MagicMock(return_value=True)
        self.task_manager._handle_obstacle_avoidance = MagicMock(return_value=True)
        self.task_manager._handle_package_search = MagicMock(return_value=True)
        self.task_manager._handle_package_pickup = MagicMock(return_value=True)
        self.task_manager._handle_marker_search = MagicMock(return_value=True)
        self.task_manager._handle_package_dropoff = MagicMock(return_value=True)
        self.task_manager._handle_landing = MagicMock(return_value=True)
        self.task_manager._handle_mission_complete = MagicMock(return_value=True)
        self.task_manager._handle_abort = MagicMock(return_value=False)
        
        # 初始状态下的更新
        result = self.task_manager.update()
        
        # 验证结果和状态转换
        self.assertTrue(result)
        self.assertEqual(self.task_manager.state, TaskState.TUNNELLING)
        
        # 转换为隧道穿越状态后的更新
        self.task_manager.state = TaskState.TUNNELLING
        
        result = self.task_manager.update()
        
        self.assertTrue(result)
        self.task_manager._handle_tunnelling.assert_called_once()


if __name__ == '__main__':
    unittest.main()