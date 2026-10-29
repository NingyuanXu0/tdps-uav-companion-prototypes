"""
无人机系统配置参数
包含通信设置、传感器配置和任务参数
"""

# 系统参数
DEBUG_MODE = True
TASK_EXECUTION_FREQUENCY = 20  # Hz，降低刷新频率
MAIN_LOOP_DELAY = 1 / TASK_EXECUTION_FREQUENCY

# I²C通信参数 - FCM通信
I2C_SCL_PIN = 22
I2C_SDA_PIN = 21
I2C_FREQUENCY = 400000  # 400 kHz
FCM_I2C_ADDRESS = 0x08

# I²C通信参数 - VL53L0X传感器
VL53L0X_SCL_PIN = 5
VL53L0X_SDA_PIN = 4
VL53L0X_XSHUT_PIN = 18  # 可选，复位控制
VL53L0X_ADDRESS = 0x29

# 任务参数
TAKEOFF_HEIGHT = 0.5  # 米，降低为0.5米高度
HOVERING_TIME = 10.0  # 秒
LANDING_HEIGHT = 0.1  # 米

# 飞行速度参数
ASCENT_SPEED = 0.01  # 米/刷新周期，降低上升速度
DESCENT_SPEED = 0.01  # 米/刷新周期，降低下降速度

# 按钮引脚定义
START_BUTTON_PIN = 0
EMERGENCY_BUTTON_PIN = 15

# FCM命令代码
FCM_COMMANDS = {
    "MOTORS_OFF": 0x00,
    "MOTORS_ON": 0x01,
    "SET_ALTITUDE": 0x05,
    "EMERGENCY_STOP": 0xFF,
}

# LED指示模式
LED_PATTERNS = {
    "STARTUP": [(1, 0.1), (0, 0.1)] * 3,  # 快速闪烁3次
    "READY": [(1, 1), (0, 1)],            # 慢闪，等待起飞
    "TAKEOFF": [(1, 0.5), (0, 0.5)],      # 中速闪烁
    "HOVERING": [(1, 0.2), (0, 0.2)],     # 快闪
    "LANDING": [(1, 0.1), (0, 0.1)] * 5,  # 快速闪烁5次
    "COMPLETE": [(1, 2)],                 # 持续亮起
    "ERROR": [(1, 0.1), (0, 0.1)] * 10,   # 快速闪烁10次
}

# 高度传感器参数
HEIGHT_THRESHOLD = 0.05  # 米（高度测量容差）
MAX_RANGE = 2.0  # 米（VL53L0X最大测量范围）