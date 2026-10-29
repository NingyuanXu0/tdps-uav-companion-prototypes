"""
无人机系统配置参数
包含预设路径、时序参数和通信设置
针对简化任务进行了优化

任务流程：起飞 → 前进（从障碍物下方通过） → 左转（穿过障碍物） → 后退 → 降落并通信
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 50  # Hz
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY
AUTO_TAKEOFF_DELAY = 15  # 上电后15秒自动起飞

# UART通信参数（替代原I2C）
UART_TX_PIN = 17  # GPIO17 - 连接到Pixhawk4的RX
UART_RX_PIN = 16  # GPIO16 - 连接到Pixhawk4的TX
UART_BAUD_RATE = 115200

# HC12通信参数
HC12_TX_PIN = 43  # GPIO43 - 连接到HC12的RX
HC12_RX_PIN = 44  # GPIO44 - 连接到HC12的TX
HC12_SET_PIN = 21 # GPIO21 - 连接到HC12的SET引脚
HC12_BAUD_RATE = 9600
HC12_CHANNEL = 1

# 传感器参数
VL53L0X_SCL_PIN = 5  # GPIO5 - I2C时钟线
VL53L0X_SDA_PIN = 4  # GPIO4 - I2C数据线
SENSOR_READ_RETRIES = 3

# 电池监控
BATTERY_ADC_PIN = 1  # GPIO1 (ADC1_1) - 电池电压检测
LOW_BATTERY_THRESHOLD = 14.0  # 伏特
CRITICAL_BATTERY_THRESHOLD = 13.5  # 伏特

# 任务参数
TAKEOFF_HEIGHT = 2.5  # 米（起飞高度）
CRUISING_HEIGHT = 2.5  # 米（巡航高度）
OBSTACLE_PASS_HEIGHT = 2.2  # 米（从障碍物下方通过的高度）
LANDING_HEIGHT = 0.1  # 米
MISSION_COMPLETE_WAIT_TIME = 15  # 秒（降落后等待时间）

# 简化的预设路径航点 (x, y, z)，从起点开始的相对坐标
# 根据实际场地调整（场地与PDF中的镜像对称）
PATH_WAYPOINTS = [
    # 任务1：起飞
    (0, 0, 0.5),           # 初始上升
    (0, 0, TAKEOFF_HEIGHT), # 起飞到巡航高度
    
    # 任务2：前进（从障碍物下方通过）
    (2, 0, TAKEOFF_HEIGHT),         # 前进
    (4, 0, OBSTACLE_PASS_HEIGHT),   # 降低高度准备通过障碍物
    (6, 0, OBSTACLE_PASS_HEIGHT),   # 通过第一个障碍物
    (8, 0, OBSTACLE_PASS_HEIGHT),   # 通过第二个障碍物
    (8, 0, CRUISING_HEIGHT),        # 恢复巡航高度
    
    # 任务3：左转并穿过障碍物
    (8, 2, CRUISING_HEIGHT),   # 开始左转
    (8, 4, CRUISING_HEIGHT),   # 继续左转
    (8, 6, CRUISING_HEIGHT),   # 完成左转
    
    # 任务4：后退（原任务4和5的简化）
    (6, 6, CRUISING_HEIGHT),   # 后退
    (4, 6, CRUISING_HEIGHT),   # 继续后退
    
    # 任务5：降落
    (4, 6, 1.0),              # 开始降落
    (4, 6, LANDING_HEIGHT),   # 最终降落
]

# 任务转换索引
TASK_TRANSITION_INDICES = {
    "TAKEOFF_COMPLETE": 2,
    "FORWARD_COMPLETE": 7,
    "LEFT_TURN_COMPLETE": 10,
    "BACKWARD_COMPLETE": 12,
    "LANDING_COMPLETE": 14,
}

# 传感器阈值
HEIGHT_THRESHOLD = 0.05  # 米（高度测量容差）
POSITION_THRESHOLD = 0.3  # 米（位置容差，放宽以适应预设路径）

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

# 任务状态常量（替代Enum）
class TaskState:
    INITIALIZING = 0
    TAKEOFF = 1
    FORWARD_FLIGHT = 2      # 前进（从障碍物下方通过）
    LEFT_TURN = 3          # 左转（穿过障碍物）
    BACKWARD_FLIGHT = 4    # 后退
    LANDING = 5
    MISSION_COMPLETE = 6
    ERROR = 7
# The archived route uses synthetic position feedback.
SIMULATION_MODE = True

# Set the measured voltage-divider ratio in ignored config_local.py.
BATTERY_VOLTAGE_DIVIDER = None
try:
    from config_local import BATTERY_VOLTAGE_DIVIDER
except ImportError:
    pass
