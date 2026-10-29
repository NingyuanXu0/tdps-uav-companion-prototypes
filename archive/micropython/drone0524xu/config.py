"""
无人机系统配置参数 - 起飞降落测试版本
仅包含起飞和降落测试所需的基本参数
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 50  # Hz
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY

# UART通信参数（Pixhawk连接）
UART_TX_PIN = 10  # ESP32 TX → Pixhawk RX
UART_RX_PIN = 9   # ESP32 RX ← Pixhawk TX
UART_BAUDRATE = 57600  # 标准MAVLink波特率

# 传感器参数
VL53L0X_SCL_PIN = 5
VL53L0X_SDA_PIN = 4
HCSR04_TRIGGER_PIN = 12
HCSR04_ECHO_PIN = 13
SENSOR_READ_RETRIES = 3
SENSOR_TIMEOUT_MS = 100  # 传感器超时时间

# 任务参数 - 起飞降落测试
TAKEOFF_HEIGHT = 2.0  # 起飞高度：2.0米（天花板灯高度）
LANDING_HEIGHT = 0.1  # 降落高度：0.1米
TEST_HOVER_TIME = 5.0  # 悬停时间：5.0秒
LANDING_SPEED = 0.5   # 降落速度：0.5米/秒

# 自动启动参数
AUTO_START_DELAY = 10.0  # 自动启动延迟：10秒
USE_AUTO_START = True    # 是否使用自动启动
USE_WIFI_CONTROL = False # 是否使用WiFi控制

# 位置控制阈值
HEIGHT_THRESHOLD = 0.1   # 高度容差：0.1米
HEIGHT_CHECK_STABLE_COUNT = 5  # 高度稳定检查次数
POSITION_THRESHOLD = 0.2 # 位置容差：0.2米

# 安全参数
MAX_ALTITUDE = 3.0       # 最大飞行高度：3.0米
MIN_BATTERY_VOLTAGE = 13.0  # 最低电池电压：13.0V
EMERGENCY_LAND_HEIGHT = 0.5  # 紧急降落触发高度

# LED指示灯引脚和闪烁模式
LED_PIN = 2  # 内置LED引脚
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,           # 快速闪烁3次
    "WAITING_FOR_START": [(1, 0.1), (0, 1.9)],     # 非常慢的闪烁
    "AUTO_START_COUNTDOWN": [(1, 0.2), (0, 0.2)],  # 中速闪烁
    "TAKEOFF": [(1, 0.5), (0, 0.5)],               # 慢速闪烁
    "HOVERING": [(1, 1), (0, 1)],                  # 1秒亮1秒灭
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,           # 快速闪烁5次
    "TEST_COMPLETE": [(1, 2)],                      # 持续亮起
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,           # 快速闪烁10次
    "WARNING": [(1, 0.3), (0, 0.3)] * 3,          # 警告闪烁
}

# 按钮引脚定义
START_BUTTON_PIN = 15    # 启动按钮
ABORT_BUTTON_PIN = 14    # 中止按钮

# WiFi配置（如果启用）
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
WIFI_PORT = 8080
WIFI_TIMEOUT = 30  # WiFi连接超时时间（秒）

# 日志配置
LOG_FILENAME = "flight_log.txt"
LOG_MAX_SIZE = 100000  # 最大日志文件大小（字节）
LOG_LEVELS = ["INFO", "WARNING", "ERROR", "DEBUG"]

# 任务状态定义
TASK_STATES = {
    "INITIALIZING": 0,
    "WAITING_FOR_START": 1,
    "TAKEOFF": 2,
    "HOVERING": 3,
    "LANDING": 4,
    "TEST_COMPLETE": 5,
    "ERROR": 6,
    "ABORT": 7
}

# 错误代码定义
ERROR_CODES = {
    "NONE": 0,
    "SENSOR_INIT_FAILED": 1,
    "FCM_INIT_FAILED": 2,
    "TAKEOFF_TIMEOUT": 3,
    "LANDING_TIMEOUT": 4,
    "HEIGHT_SENSOR_FAILED": 5,
    "BATTERY_LOW": 6,
    "EMERGENCY_STOP": 7,
    "COMMUNICATION_LOST": 8
}

# 注释掉的完整任务参数（供将来使用）
"""
# 完整任务参数
CRUISING_HEIGHT = 2.7  # 巡航高度：2.7米
OBSTACLE_AVOIDANCE_HEIGHT = 2.6  # 避障高度：2.6米
MINIMUM_SAFE_HEIGHT = 2.5  # 最小安全高度：2.5米
PICKUP_HEIGHT = 0.2  # 拾取高度：0.2米
DROPOFF_HEIGHT = 0.2  # 投放高度：0.2米
MISSION_COMPLETE_WAIT_TIME = 15  # 任务完成等待时间：15秒

# 路径规划参数
TUNNEL_WIDTH = 0.5  # 隧道宽度：0.5米
TUNNEL_HEIGHT = 0.6  # 隧道高度：0.6米
BEAM_HEIGHT = 2.8  # 横梁高度：2.8米
RED_BOX_SIZE = 1.2  # 红色拾取区域大小：1.2米
GREEN_BOX_SIZE = 1.2  # 绿色投放区域大小：1.2米
PACKAGE_SIZE = 0.06  # 包裹大小：0.06米
MARKER_SIZE = 0.06  # 标记大小：0.06米
"""
# Keep deployment credentials in an ignored, board-local file.
try:
    from config_local import WIFI_SSID, WIFI_PASSWORD
except ImportError:
    pass
