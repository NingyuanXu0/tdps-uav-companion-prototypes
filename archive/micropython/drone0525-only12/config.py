"""
无人机系统配置参数 - 起飞降落测试版本
仅包含测试所需的基本配置

作者: Score99团队
日期: 2025年4月
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 50  # Hz
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY
AUTO_TAKEOFF_DELAY = 5  # 缩短到5秒用于测试

# UART通信参数（与Pixhawk4通信）
UART_TX_PIN = 17  # GPIO17 - 连接到Pixhawk4的RX
UART_RX_PIN = 16  # GPIO16 - 连接到Pixhawk4的TX
UART_BAUD_RATE = 115200

# 测试参数
TEST_HOVER_HEIGHT = 0.5  # 测试悬停高度（米）
TEST_HOVER_TIME = 3.0    # 悬停时间（秒）

# LED指示灯闪烁模式
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # 快速闪烁3次
    "WAITING": [(1, 0.5), (0, 0.5)],      # 慢速闪烁
    "TAKEOFF": [(1, 0.2), (0, 0.2)],      # 中速闪烁
    "FLYING": [(1, 1), (0, 0.1)],         # 长亮短灭
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,  # 快速闪烁5次
    "COMPLETE": [(1, 2)],                  # 持续亮
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,   # 快速闪烁10次
}

# 飞控命令代码
FCM_COMMANDS = {
    "MOTORS_OFF": 0x00,
    "MOTORS_ON": 0x01,
    "SET_POSITION": 0x02,
    "SET_MODE": 0x07,
    "TAKEOFF": 0x08,
    "LAND": 0x09,
    "EMERGENCY_STOP": 0xFF,
}

# 任务状态常量（简化版本）
class TaskState:
    INITIALIZING = 0
    READY = 1
    TAKEOFF = 2
    HOVERING = 3
    LANDING = 4
    COMPLETE = 5
    ERROR = 6