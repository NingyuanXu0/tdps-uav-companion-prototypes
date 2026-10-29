"""
main.py 的单元测试
测试主程序流程和系统初始化
"""
import unittest
from unittest.mock import patch, MagicMock, call
import time
import sys
import gc

# 模拟依赖
sys.modules['time'] = MagicMock()
sys.modules['gc'] = MagicMock()
sys.modules['machine'] = MagicMock()
sys.modules['config'] = MagicMock()
sys.modules['utils'] = MagicMock()
sys.modules['perception'] = MagicMock()
sys.modules['communication'] = MagicMock()
sys.modules['task_manager'] = MagicMock()

# 配置模拟
sys.modules['config'].MAIN_LOOP_DELAY = 0.02
sys.modules['config'].DEBUG_MODE = True
sys.modules['time'].ticks_ms = MagicMock(side_effect=[0, 10, 20, 30])
sys.modules['time'].ticks_diff = MagicMock(return_value=10)
sys.modules['utils'].log = MagicMock()
sys.modules['utils'].display_led_pattern = MagicMock()

# 导入被测试模块
import main

class TestMain(unittest.TestCase):
    """测试主程序功能"""
    
    def setUp(self):
        """测试前的准备工作"""
        # 模拟系统组件
        self.mock_fcm = MagicMock()
        self.mock_fcm.initialized = True
        
        self.mock_perception = MagicMock()
        self.mock_perception.initialized = True
        
        self.mock_wireless = MagicMock()
        
        self.mock_task_manager = MagicMock()
        self.mock_task_manager.initialize.return_value = True
        self.mock_task_manager.update.return_value = True
        self.mock_task_manager.LOW_BATTERY_THRESHOLD = 14.0
        self.mock_task_manager.CRITICAL_BATTERY_THRESHOLD = 13.5
        self.mock_task_manager.get_battery_voltage.return_value = 15.0
        
        # 模拟按钮
        self.mock_button = MagicMock()
        self.mock_button.value.side_effect = [1, 1, 0]  # 第三次按下按钮
        
        # 模拟看门狗
        self.mock_wdt = MagicMock()
        
        # 打补丁
        patch('communication.FCMInterface', return_value=self.mock_fcm).start()
        patch('perception.PerceptionSystem', return_value=self.mock_perception).start()
        patch('communication.WirelessCommunication', return_value=self.mock_wireless).start()
        patch('task_manager.TaskManager', return_value=self.mock_task_manager).start()
        patch('machine.Pin', return_value=self.mock_button).start()
        patch('machine.WDT', return_value=self.mock_wdt).start()
    
    def tearDown(self):
        """测试后的清理工作"""
        patch.stopall()
    
    def test_initialize_system(self):
        """测试系统初始化函数"""
        task_manager = main.initialize_system()
        
        self.assertEqual(task_manager, self.mock_task_manager)
        
        # 检查是否正确创建了所有系统组件
        sys.modules['communication'].FCMInterface.assert_called_once()
        sys.modules['perception'].PerceptionSystem.assert_called_once()
        sys.modules['communication'].WirelessCommunication.assert_called_once()
        sys.modules['task_manager'].TaskManager.assert_called_once_with(
            self.mock_fcm, self.mock_perception, self.mock_wireless
        )
    
    def test_initialize_system_failure(self):
        """测试系统初始化失败的情况"""
        # 模拟FCM初始化失败
        self.mock_fcm.initialized = False
        
        task_manager = main.initialize_system()
        
        self.assertIsNone(task_manager)
        sys.modules['utils'].log.assert_any_call("FCM 接口初始化失败", "ERROR")
    
    @patch('main.initialize_system')
    def test_main_success(self, mock_init_system):
        """测试主函数成功执行"""
        # 模拟成功初始化
        mock_init_system.return_value = self.mock_task_manager
        
        # 修改update返回值，使主循环退出
        self.mock_task_manager.update.side_effect = [True, False]
        
        # 运行主函数
        main.main()
        
        # 验证执行流程
        mock_init_system.assert_called_once()
        self.mock_task_manager.initialize.assert_called_once()
        self.mock_wdt.feed.assert_called()
        self.mock_task_manager.update.assert_called()
        sys.modules['gc'].collect.assert_called()
        
        # 验证LED模式显示
        sys.modules['utils'].display_led_pattern.assert_any_call("WAITING_FOR_START")
        sys.modules['utils'].display_led_pattern.assert_any_call("MISSION_START", 2)
    
    @patch('main.initialize_system')
    def test_main_initialization_failure(self, mock_init_system):
        """测试主函数初始化失败的情况"""
        # 模拟初始化失败
        mock_init_system.return_value = None
        
        # 运行主函数
        main.main()
        
        # 验证错误处理
        sys.modules['utils'].log.assert_any_call("系统初始化失败，中止", "ERROR")
        sys.modules['utils'].display_led_pattern.assert_any_call("ERROR")
    
    @patch('main.initialize_system')
    def test_main_task_manager_initialization_failure(self, mock_init_system):
        """测试任务管理器初始化失败的情况"""
        # 模拟成功获取任务管理器但初始化失败
        mock_init_system.return_value = self.mock_task_manager
        self.mock_task_manager.initialize.return_value = False
        
        # 运行主函数
        main.main()
        
        # 验证错误处理
        sys.modules['utils'].log.assert_any_call("任务管理器初始化失败，中止", "ERROR")
        sys.modules['utils'].display_led_pattern.assert_any_call("ERROR")
    
    @patch('main.initialize_system')
    def test_main_with_exception(self, mock_init_system):
        """测试主函数执行过程中出现异常的情况"""
        # 模拟成功初始化但更新时抛出异常
        mock_init_system.return_value = self.mock_task_manager
        self.mock_task_manager.update.side_effect = Exception("测试异常")
        
        # 运行主函数
        main.main()
        
        # 验证异常处理
        sys.modules['utils'].log.assert_any_call("主循环中出现未处理异常: 测试异常", "ERROR")
        sys.modules['utils'].display_led_pattern.assert_any_call("ERROR")
        self.mock_task_manager.emergency_landing.assert_called_once()
    
    @patch('main.initialize_system')
    def test_main_with_low_battery(self, mock_init_system):
        """测试主函数在低电量情况下的行为"""
        # 模拟成功初始化但电池电量低
        mock_init_system.return_value = self.mock_task_manager
        self.mock_task_manager.get_battery_voltage.return_value = 13.8  # 低于阈值但高于临界值
        
        # 修改update返回值，使主循环退出
        self.mock_task_manager.update.side_effect = [True, False]
        
        # 运行主函数
        main.main()
        
        # 验证低电量处理
        sys.modules['utils'].log.assert_any_call(f"电池电量低: 13.80V", "WARNING")
    
    @patch('main.initialize_system')
    def test_main_with_critical_battery(self, mock_init_system):
        """测试主函数在电量严重不足情况下的行为"""
        # 模拟成功初始化但电池电量严重不足
        mock_init_system.return_value = self.mock_task_manager
        self.mock_task_manager.get_battery_voltage.return_value = 13.0  # 低于临界值
        
        # 运行主函数
        main.main()
        
        # 验证紧急降落处理
        sys.modules['utils'].log.assert_any_call("电池电量严重不足，紧急降落", "ERROR")
        self.mock_task_manager.emergency_landing.assert_called_once()


if __name__ == '__main__':
    unittest.main()