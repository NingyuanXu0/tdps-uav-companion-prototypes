"""
完整系统集成测试脚本
在模拟环境中运行完整的ACM系统测试
"""
import os
import sys
import importlib
import time
import threading
import tkinter as tk
import traceback

# 确保能导入所有模块
sys.path.append('.')

# 导入硬件模拟器
from hardware_simulator import hw_simulator

def simulate_button_press():
    """模拟按下启动按钮"""
    time.sleep(2)  # 等待2秒
    hw_simulator.press_button()
    hw_simulator.log("模拟按下启动按钮")

def run_integration_test():
    """运行集成测试"""
    try:
        # 导入主模块（确保使用重新加载以获取最新代码）
        hw_simulator.log("导入主模块")
        if 'main' in sys.modules:
            importlib.reload(sys.modules['main'])
        else:
            import main
        
        # 启动模拟器
        hw_simulator.start_simulation()
        
        # 创建线程模拟按下按钮
        button_thread = threading.Thread(target=simulate_button_press)
        button_thread.daemon = True
        button_thread.start()
        
        # 运行主程序
        hw_simulator.log("开始运行主程序")
        try:
            main.main()
            hw_simulator.log("主程序正常退出")
        except Exception as e:
            hw_simulator.log(f"主程序异常: {e}")
            hw_simulator.log(traceback.format_exc())
        
        # 停止模拟器
        hw_simulator.stop_simulation()
        
    except Exception as e:
        hw_simulator.log(f"集成测试异常: {e}")
        hw_simulator.log(traceback.format_exc())

def auto_test_sequence():
    """自动测试序列，模拟整个无人机任务流程"""
    try:
        # 先延迟启动，让UI完全加载
        time.sleep(1)
        
        hw_simulator.log("开始自动测试序列")
        
        # 导入所需模块
        import config
        import communication
        import perception
        import task_manager
        
        # 确保枚举类型正确
        task_manager.TaskState = sys.modules['task_manager'].TaskState
        
        # 创建核心组件
        hw_simulator.log("创建核心组件")
        fcm = communication.FCMInterface()
        percept = perception.PerceptionSystem()
        wireless = communication.WirelessCommunication()
        
        # 设置感知系统状态
        percept.initialized = True
        percept.initialize_sensors = lambda: True
        percept.calibrate_sensors = lambda: True
        
        # 创建任务管理器
        hw_simulator.log("创建任务管理器")
        tm = task_manager.TaskManager(fcm, percept, wireless)
        
        # 初始化任务管理器
        hw_simulator.log("初始化任务管理器")
        tm.initialize()
        
        # 模拟按下开始按钮
        hw_simulator.press_button()
        hw_simulator.log("模拟按下开始按钮")
        
        # 模拟任务1：起飞
        hw_simulator.log("模拟任务1：起飞")
        hw_simulator.update_height(2.8)  # 设置高度为2.8米
        tm.state = task_manager.TaskState.TUNNELLING
        tm.update()
        
        # 模拟任务2：穿越隧道
        hw_simulator.log("模拟任务2：穿越隧道")
        
        # 第一个隧道
        hw_simulator.update_drone_position(0, 0, 2.8)
        tm.current_waypoint_index = 1
        tm.update()
        time.sleep(1)
        
        # 第二个隧道
        hw_simulator.update_drone_position(2, 0, 2.8)
        tm.current_waypoint_index = 2
        tm.update()
        time.sleep(1)
        
        # 第三个隧道
        hw_simulator.update_drone_position(4, 0, 2.8)
        tm.current_waypoint_index = 3
        tm.update()
        time.sleep(1)
        
        # 结束隧道
        hw_simulator.update_drone_position(6, 0, 2.8)
        tm.current_waypoint_index = 4
        tm.update()
        time.sleep(1)
        
        # 模拟任务3：障碍物避开
        hw_simulator.log("模拟任务3：障碍物避开")
        tm.state = task_manager.TaskState.OBSTACLE_AVOIDANCE
        
        # 第一个障碍物
        hw_simulator.update_drone_position(8, 2, 2.6)
        hw_simulator.update_height(2.6)  # 更新高度
        tm.current_waypoint_index = 5
        tm.update()
        time.sleep(1)
        
        # 第二个障碍物
        hw_simulator.update_drone_position(8, 4, 2.6)
        tm.current_waypoint_index = 6
        tm.update()
        time.sleep(1)
        
        # 第三个障碍物
        hw_simulator.update_drone_position(8, 6, 2.6)
        tm.current_waypoint_index = 7
        tm.update()
        time.sleep(1)
        
        # 模拟任务4：包裹搜索和拾取
        hw_simulator.log("模拟任务4：包裹搜索和拾取")
        tm.state = task_manager.TaskState.PACKAGE_SEARCH
        
        # 接近红色区域
        hw_simulator.update_drone_position(6, 6, 2.7)
        hw_simulator.update_height(2.7)  # 更新高度
        tm.current_waypoint_index = 8
        tm.update()
        time.sleep(1)
        
        # 进入包裹拾取状态
        tm.state = task_manager.TaskState.PACKAGE_PICKUP
        
        # 降低高度进行拾取
        hw_simulator.update_drone_position(6, 6, 0.2)
        hw_simulator.update_height(0.2)  # 更新高度
        tm.current_waypoint_index = 9
        tm.update()
        time.sleep(1)
        
        # 拾取后上升
        hw_simulator.update_drone_position(6, 6, 2.7)
        hw_simulator.update_height(2.7)  # 更新高度
        tm.package_picked_up = True
        tm.current_waypoint_index = 10
        tm.update()
        time.sleep(1)
        
        # 模拟任务5：标记搜索和包裹放置
        hw_simulator.log("模拟任务5：标记搜索和包裹放置")
        tm.state = task_manager.TaskState.MARKER_SEARCH
        
        # 接近绿色区域
        hw_simulator.update_drone_position(4, 8, 2.7)
        tm.current_waypoint_index = 11
        tm.update()
        time.sleep(1)
        
        # 进入包裹放置状态
        tm.state = task_manager.TaskState.PACKAGE_DROPOFF
        
        # 降低高度进行放置
        hw_simulator.update_drone_position(4, 8, 0.2)
        hw_simulator.update_height(0.2)  # 更新高度
        tm.current_waypoint_index = 12
        tm.update()
        time.sleep(1)
        
        # 放置后上升
        hw_simulator.update_drone_position(4, 8, 2.7)
        hw_simulator.update_height(2.7)  # 更新高度
        tm.package_dropped_off = True
        tm.current_waypoint_index = 13
        tm.update()
        time.sleep(1)
        
        # 模拟任务6：降落和通信
        hw_simulator.log("模拟任务6：降落和通信")
        tm.state = task_manager.TaskState.LANDING
        
        # 降落
        hw_simulator.update_drone_position(4, 8, 0.1)
        hw_simulator.update_height(0.1)  # 接近地面
        tm.current_waypoint_index = 14
        tm.update()
        time.sleep(1)
        
        # 任务完成
        hw_simulator.log("任务完成，等待发送消息")
        tm.mission_complete_time = time.time() - 20  # 设置为20秒前
        tm.update()
        
        hw_simulator.log("自动测试序列完成")
        
    except Exception as e:
        hw_simulator.log(f"自动测试序列异常: {e}")
        hw_simulator.log(traceback.format_exc())

def run_auto_test():
    """启动自动测试线程"""
    auto_thread = threading.Thread(target=auto_test_sequence)
    auto_thread.daemon = True
    auto_thread.start()

if __name__ == "__main__":
    # 创建自动测试按钮
    auto_test_button = tk.Button(hw_simulator.control_frame, text="运行自动测试", command=run_auto_test)
    auto_test_button.pack(side=tk.LEFT, padx=5)
    
    # 创建集成测试按钮
    integration_test_button = tk.Button(hw_simulator.control_frame, text="运行集成测试", command=run_integration_test)
    integration_test_button.pack(side=tk.LEFT, padx=5)
    
    # 运行硬件模拟器
    hw_simulator.run()