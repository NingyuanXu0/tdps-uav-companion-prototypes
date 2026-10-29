"""
无人机系统配置参数 - MAVLink增强版本
适配Pixhawk4飞控的完整配置

作者: Score99团队
日期: 2025年4月
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 20  # Hz - 降低频率以适应MAVLink通信
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY
AUTO_TAKEOFF_DELAY = 10  # 增加到10秒，给系统更多初始化时间

# UART通信参数（Pixhawk4 TELEM端口）
UART_TX_PIN = 17  # GPIO17 - U1TXD - 连接到Pixhawk4 TELEM RX
UART_RX_PIN = 18  # GPIO18 - U1RXD - 连接到Pixhawk4 TELEM TX（修正：原来是16）
MAVLINK_BAUD_RATE = 57600  # Pixhawk4 TELEM端口标准波特率

# MAVLink系统参数
MAVLINK_SYSTEM_ID = 255  # 伴随计算机系统ID
MAVLINK_COMPONENT_ID = 190  # 伴随计算机组件ID
TARGET_SYSTEM_ID = 1  # Pixhawk4系统ID
TARGET_COMPONENT_ID = 1  # Pixhawk4组件ID

# OFFBOARD控制参数
OFFBOARD_POSITION_RATE = 20  # Hz - 位置目标发送频率
OFFBOARD_TIMEOUT = 0.5  # 秒 - OFFBOARD模式超时时间
HEARTBEAT_RATE = 2  # Hz - 心跳发送频率
INITIAL_POSITION_STREAM_DURATION = 2  # 秒 - 初始位置流持续时间

# 测试参数
TEST_HOVER_HEIGHT = 0.25  # 降低到0.25米用于初始测试
TEST_HOVER_TIME = 5.0    # 延长到5秒，便于观察

# 安全参数
ARM_RETRY_COUNT = 3      # 解锁重试次数
COMMAND_RETRY_DELAY = 0.5  # 命令重试延迟（秒）
TAKEOFF_TIMEOUT = 15.0   # 起飞超时时间（秒）
LANDING_TIMEOUT = 15.0   # 降落超时时间（秒）
EMERGENCY_DISARM_ATTEMPTS = 5  # 紧急锁定尝试次数
MIN_BATTERY_VOLTAGE = 14.0  # 最小电池电压（4S电池）

# 位置控制参数
POSITION_TOLERANCE = 0.1  # 米 - 位置到达容差
ALTITUDE_RATE_LIMIT = 0.5  # 米/秒 - 最大垂直速度
HORIZONTAL_RATE_LIMIT = 1.0  # 米/秒 - 最大水平速度
YAW_RATE_LIMIT = 30.0  # 度/秒 - 最大偏航速度

# 传感器配置
SENSOR_I2C_SCL_PIN = 5  # GPIO5 - I2C时钟线
SENSOR_I2C_SDA_PIN = 4  # GPIO4 - I2C数据线
SENSOR_I2C_FREQ = 400000  # Hz - I2C频率
VL53L0X_ADDRESS = 0x29  # VL53L0X默认地址
SENSOR_TIMEOUT = 1000  # 毫秒 - 传感器读取超时

# 高度传感器滤波参数
HEIGHT_FILTER_WINDOW = 5  # 滤波窗口大小
HEIGHT_OUTLIER_THRESHOLD = 0.5  # 米 - 异常值阈值
HEIGHT_CHANGE_RATE_LIMIT = 2.0  # 米/秒 - 最大高度变化率

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
    "WARNING": [(1, 0.2), (0, 0.1)] * 3,  # 警告模式
    "SENSOR_ERROR": [(1, 0.05), (0, 0.05)] * 20,  # 超快速闪烁
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
    EMERGENCY = 10     # 紧急状态

# 系统诊断参数
DIAGNOSTICS_INTERVAL = 5.0  # 秒 - 诊断信息输出间隔
PERFORMANCE_LOG_INTERVAL = 10.0  # 秒 - 性能日志间隔
MAX_LOOP_TIME_WARNING = 0.1  # 秒 - 循环时间警告阈值

# 错误恢复参数
MAX_CONSECUTIVE_ERRORS = 5  # 最大连续错误次数
ERROR_RECOVERY_DELAY = 1.0  # 秒 - 错误恢复延迟
WATCHDOG_TIMEOUT = 30000  # 毫秒 - 看门狗超时

# 调试选项
MAVLINK_DEBUG = True  # MAVLink消息调试
SENSOR_DEBUG = True  # 传感器数据调试
PERFORMANCE_DEBUG = True  # 性能监控调试

# 备用UART配置（如果需要使用UART2）
# UART2使用GPIO43(U0TXD)和GPIO44(U0RXD)
UART2_TX_PIN = 43  # GPIO43 - U0TXD
UART2_RX_PIN = 44  # GPIO44 - U0RXD