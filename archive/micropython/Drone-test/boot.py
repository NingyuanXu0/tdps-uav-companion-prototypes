# 启动文件
# 这个文件在ESP32-S3上电时首先执行
import gc
import machine
import time
import esp

# 启用垃圾回收
gc.enable()

# 设置CPU频率
machine.freq(240000000)  # 设置为240MHz以获得最佳性能

# 禁用调试输出以提高性能
esp.osdebug(None)

# 初始化状态LED
led = machine.Pin(2, machine.Pin.OUT)

# 闪烁LED表示启动
for _ in range(3):
    led.value(1)
    time.sleep(0.1)
    led.value(0)
    time.sleep(0.1)

print("无人机控制系统启动中...")

# 清理内存
gc.collect()