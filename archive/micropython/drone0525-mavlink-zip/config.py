"""
无人机系统配置参数 - MAVLink版本
适配Pixhawk4飞控的配置

作者: Score99团队
日期: 2025年4月
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 20  # Hz - 降低频率以适应MAVLink通信
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY
AUTO_TAKEOFF_DELAY = 10  # 增加到10秒，给系统更多初始化时间

# UART通信参数（Pixhawk4 TELEM端口）
UART_TX_PIN = 17  # GPIO17 - 连接到Pixhawk4 TELEM RX
UART_RX_PIN = 16  # GPIO16 - 连接到Pixhawk4 TELEM TX
MAVLINK_BAUD_RATE = 57600  # Pixhawk4 TELEM端口标准波特率

# 测试参数
TEST_HOVER_HEIGHT = 1.0  # 提高到1米，更安全
TEST_HOVER_TIME = 5.0    # 延长到5秒，便于观察

# 安全参数
ARM_RETRY_COUNT = 3      # 解锁重试次数
COMMAND_RETRY_DELAY = 0.5  # 命令重试延迟（秒）
TAKEOFF_TIMEOUT = 15.0   # 起飞超时时间（秒）
LANDING_TIMEOUT = 15.0   # 降落超时时间（秒）

# LED指示灯闪烁模式
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # 快速闪烁3次
    "WAITING": [(1, 0.5), (0, 0.5)],      # 慢速闪烁
    "CONNECTING": [(1, 0.2), (0, 0.2)],   # 中速闪烁 - 连接飞控
    "ARMED": [(1, 0.1), (0, 0.1)] * 2,    # 双闪 - 已解锁
    "TAKEOFF": [(1, 0.3), (0, 0.1)],      # 长短闪烁
    "FLYING": [(1, 1), (0, 0.1)],         # 长亮短灭
    "LANDING": [(1, 0.1), (0, 0.3)],      # 短长闪烁
    "COMPLETE": [(1, 2)],                  # 持续亮
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,   # 快速闪烁10次
}

# 任务状态常量
class TaskState:
    INITIALIZING = 0
    CONNECTING = 1      # 连接飞控
    READY = 2
    ARMING = 3         # 解锁电机
    TAKEOFF = 4
    HOVERING = 5
    LANDING = 6
    DISARMING = 7      # 锁定电机
    COMPLETE = 8
    ERROR = 9