"""
无人机系统配置参数 - 起飞降落测试版本
仅包含起飞和降落测试所需的基本参数
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 50  # Hz
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY

# 通信参数
# 修正：使用ESP32-S3-DevKitC-1上可用的GPIO引脚
I2C_SCL_PIN = 18  # 原为22，但GPIO22在引脚表中未列出
I2C_SDA_PIN = 8   # 原为21，改为GPIO8以确保兼容性
I2C_FREQUENCY = 400000  # 400 kHz
FCM_I2C_ADDRESS = 0x08

# 传感器参数
VL53L0X_SCL_PIN = 5
VL53L0X_SDA_PIN = 4
HCSR04_TRIGGER_PIN = 12
HCSR04_ECHO_PIN = 13
SENSOR_READ_RETRIES = 3

# 任务参数 - 仅保留起飞降落相关
TAKEOFF_HEIGHT = 2.0  # 起飞高度：2.0米
LANDING_HEIGHT = 0.1  # 降落高度：0.1米
TEST_HOVER_TIME = 5.0  # 悬停时间：5.0秒

# 自动启动参数
AUTO_START_DELAY = 10.0  # 自动启动延迟：10秒
USE_AUTO_START = True  # 是否使用自动启动
USE_WIFI_CONTROL = False  # 是否使用WiFi控制

# 位置阈值
HEIGHT_THRESHOLD = 0.1  # 高度容差：0.1米
POSITION_THRESHOLD = 0.2  # 位置容差：0.2米

# LED指示灯闪烁模式
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # 快速闪烁3次
    "WAITING_FOR_START": [(1, 0.1), (0, 1.9)],  # 非常慢的闪烁
    "AUTO_START_COUNTDOWN": [(1, 0.2), (0, 0.2)],  # 中速闪烁
    "TAKEOFF": [(1, 0.5), (0, 0.5)],  # 慢速闪烁
    "HOVERING": [(1, 1), (0, 1)],  # 1秒亮1秒灭
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,  # 快速闪烁5次
    "TEST_COMPLETE": [(1, 2)],  # 持续亮起
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,  # 快速闪烁10次
}

# FCM命令代码
FCM_COMMANDS = {
    "MOTORS_OFF": 0x00,
    "MOTORS_ON": 0x01,
    "SET_POSITION": 0x02,
    "SET_VELOCITY": 0x03,
    "SET_ATTITUDE": 0x04,
    "SET_ALTITUDE": 0x05,
    "RC_OVERRIDE": 0x06,
    "SET_MODE": 0x07,
    "TAKEOFF": 0x08,
    "LAND": 0x09,
    "GET_STATUS": 0x0A,
    "HEARTBEAT": 0x0B,
    "EMERGENCY_STOP": 0xFF,
}

# WiFi配置（如果启用）
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
WIFI_PORT = 8080

# 注释掉的完整任务参数
"""
# HC12通信参数
# HC12_TX_PIN = 17
# HC12_RX_PIN = 16
# HC12_SET_PIN = 23
# HC12_BAUD_RATE = 9600
# HC12_CHANNEL = 1

# 任务参数（完整任务时使用）
# CRUISING_HEIGHT = 2.7  # 米
# OBSTACLE_AVOIDANCE_HEIGHT = 2.6  # 米
# MINIMUM_SAFE_HEIGHT = 2.5  # 米
# PICKUP_HEIGHT = 0.2  # 米
# DROPOFF_HEIGHT = 0.2  # 米
# MISSION_COMPLETE_WAIT_TIME = 15  # 秒

# 预设路径航点（完整任务时使用）
# PATH_WAYPOINTS = [...]

# 任务转换索引（完整任务时使用）
# TASK_TRANSITION_INDICES = {...}
"""
# Keep deployment credentials in an ignored, board-local file.
try:
    from config_local import WIFI_SSID, WIFI_PASSWORD
except ImportError:
    pass
