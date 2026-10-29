# TDPS UAV Companion Prototypes

TDPS 课程项目的无人机伴随控制器代码，保留 Score99 团队在 ESP32 / ESP32-S3、MicroPython、Arduino 和 ESP-IDF 上的多个独立原型。飞控侧面向 Pixhawk4 / PX4。

本仓库收录源码、已有测试与诊断工具源码、构建配置、MAVLink 源码依赖及必要说明。课程报告、课件、演示文稿、图片视频、财务和团队资料、飞行日志、虚拟环境、缓存及原始压缩包不在发布范围内。不同日期的实现按目录保存；同名模块应在各自版本内使用。

## 目录与入口

| 路径 | 用途与入口 | 当前边界 |
| --- | --- | --- |
| `prototypes/esp-idf/` | ESP-IDF 工程；`main/acm_core.c` 中的 `app_main()` | PX4 通信、预设航点状态机、HC12 完成消息；测距读取仍为示例 |
| `prototypes/arduino/mission_route/mission_route.ino` | 独立 Arduino 草图；`setup()` / `loop()` | 起飞、隧道航段、转向、定高、区域穿越和降落流程 |
| `prototypes/arduino/mavlink_debug/mavlink_debug.ino` | 独立串口诊断草图 | 接收飞控状态、发送心跳及手动串口命令；自身不产生 OFFBOARD 设定点流 |
| `archive/arduino/drone0529-split/` | `drone_control.cpp/.h` 拆分模块 | 提供初始化、收发与任务函数；未提供独立 `setup()` / `loop()` |
| `archive/micropython/` | 各版本通常由本目录 `main.py` 启动 | 历史路线、传感器、通信及台架测试实现，见下表 |
| `libraries/MAVLink/` | 原项目携带的 MAVLink-Arduino 2.0.17 源码依赖 | Arduino 与 ESP-IDF 共用，保留上游许可 |
| `libraries/mavlink_diy/` | Arduino MAVLink 通信辅助库 | 位置、姿态、模式和解锁指令；由调用方初始化串口及安排持续更新 |
| `tools/sensor_diagnostics/` | VL53L0X 测距诊断脚本 | 依赖 MicroPython 板卡和实际接线 |
| `tools/hc12_task6/hc12_task6_test.py` | HC12 任务六通信工具 | 依赖 MicroPython 和 HC12 硬件 |
| `tools/analysis/untitled.m` | MATLAB 测距误差分析脚本 | 保留原脚本内的数据和绘图逻辑 |
| `archive/esp-idf/drone0529-source/Untitled-1.sh` | 原始工程生成脚本 | 历史记录，会创建工程及下载依赖；当前工程入口是 `prototypes/esp-idf/` |

两个 Arduino 草图是独立目标。拆分控制模块与诊断草图含各自的全局对象，不应合并为一个草图编译。

## MicroPython 版本

选择一个完整目录部署到板卡，保留该目录内部的 `config.py`、`communication.py`、`utils.py` 等依赖。不要把不同快照的同名文件混合到同一个模块搜索路径中。

| 目录 | 定位 |
| --- | --- |
| `Drone/`、`drone-code/` | 早期多任务伴随控制与预设路线原型 |
| `Drone-test/` | 按键启动的低高度起飞、悬停、降落原型；另含 `boot.py`、高度传感器及飞行控制模块 |
| `drone0519/` | 路线原型及平铺目录的 mock、仿真、测试框架与同步工具源码 |
| `drone0519test/` | 另一套桌面测试、集成与 GUI 工具源码，主运行模块与 0519 有重合 |
| `drone0521/`、`drone0523/` | 后续任务与感知迭代 |
| `drone0523-simulation/` | 路线仿真快照 |
| `drone0524/`、`drone0526/` | UART 接口迭代，其中部分飞行控制方法仍只记录日志 |
| `drone0524xu/` | 增强测距与文本串口协议版本；含 `vl53l0x_test_improved.py` |
| `drone0525-all/` | 多任务路线演示；`SIMULATION_MODE = True`，位置反馈使用合成数据 |
| `drone0525-only12/` | 起飞、悬停、降落流程；感知模块为模拟桩 |
| `drone0525-mavlink-directory/` | 原解压目录中的 MAVLink 通信版本 |
| `drone0525-mavlink-zip/` | 原 ZIP 内另一份 MAVLink 通信版本，与上一目录分别保留 |
| `drone0527/` | 含轻量 MAVLink 帮助函数的任务版本 |
| `drone0528/` | 按键触发的 PX4 电机测试原型，`main.py` 调用 `motor_test.py` |

`drone0519/run_tests.py`、`drone0519test/run_system_test.py` 等属于桌面工具。`drone0519/sync_to_esp.py` 优先使用 `mpremote`；其旧手工 REPL 回退路径仍按历史实现保留。此类工具的报告输出由 `.gitignore` 排除。

## 配置与依赖

### MicroPython

需要面向实际板卡的 MicroPython 固件，以及所选版本使用的 `machine`、`network`、`micropython` 等板端模块。导入 `enum` 的快照还需要固件或兼容模块提供对应 API。普通桌面 Python 环境不能直接运行依赖板端硬件的主程序。

按实际接线修改所选版本 `config.py` 中的 UART、I2C、ADC、LED 和按键引脚。各快照的接线定义不同；早期 ESP32 的 GPIO22、GPIO34 等定义不能直接套用到 ESP32-S3。

Wi-Fi 默认值已替换为占位符。使用 Wi-Fi 的快照可在同目录建立被 Git 忽略的 `config_local.py`：

```python
WIFI_SSID = "replace-with-your-ssid"
WIFI_PASSWORD = "replace-with-your-password"
```

`drone-code`、`drone0519`、`drone0519test`、`drone0521` 和 `drone0525-all` 的 ADC 电池检测还需要在该文件设置 `BATTERY_VOLTAGE_DIVIDER`。数值应取自实际分压电路的 `V_battery / V_adc`；默认 `None` 会明确阻止初始化，避免沿用无法达到原低压阈值的固定倍率。ADC 参考电压、校准和电池阈值也须与硬件相符。

可选桌面工具依赖列在 [`tools/requirements-desktop.txt`](tools/requirements-desktop.txt)。GUI 工具另需操作系统提供 Tkinter。桌面依赖不能替代板端模块。

### Arduino

使用 Arduino ESP32 板卡支持包，并让工具链同时找到 `libraries/MAVLink/`。在 Arduino IDE 中可将该目录安装为本地库；Arduino CLI 可通过 `--libraries <本仓库绝对路径>/libraries` 指定库搜索目录。辅助库入口是 `mavlink_DIY.h`，主依赖入口是 `MAVLink.h`。

`mission_route.ino` 中的 `TDPS_I2C_SDA` / `TDPS_I2C_SCL` 应按实际接线配置，可通过构建宏或修改定义设置。ESP32-S3 默认将未确认的 SCL 设为 `-1`，从而跳过示例 I2C 初始化。Arduino 路线主循环以约 50 Hz 执行；拆分模块需要调用方持续更新 `taskTimer`、接收飞控消息和调用任务函数。

### ESP-IDF

工程使用 ESP-IDF 的 FreeRTOS、UART、旧式 I2C 驱动、NVS 和定时器 API。原资料未固定 SDK 版本；本次未选择或验证新的工具链版本。MAVLink 组件引用仓库内的共享依赖，无需生成脚本从远程下载主分支。

使用者配置好 ESP-IDF 后，可从 `prototypes/esp-idf/` 手动运行 `bash build.sh`。脚本使用已导出的环境，或从 `IDF_PATH/export.sh` 加载环境；首次默认目标为 `esp32s3`，可用 `TDPS_IDF_TARGET` 指定其他目标。已有 `sdkconfig` 在未显式指定目标时继续使用。

通过 `idf.py menuconfig` 的 **TDPS board configuration** 设置 SDA/SCL。ESP32-S3 的 SCL 默认 `-1`；未配置有效引脚时测距示例禁用。UART 引脚和波特率位于 `main/acm_core.h`。

### PX4 与协议

飞控的估计、姿态稳定和电调控制由外部 PX4 实现。本仓库不包含完整 PX4 固件、参数导出或飞控接收端的自定义协议实现。

飞控 TELEM 端口的波特率、系统与组件 ID、位置/姿态消息流及 OFFBOARD 参数须与所选代码一致。轻量 Python MAVLink 实现面向 **MAVLink 1**，未实现通用 MAVLink 2 接收或签名；0528 的接收器只处理支持的消息 ID。随附 C 依赖为 MAVLink v2 生成库，二者不能按目录名称视作同一种协议能力。

早期 I2C 自定义协议、0524xu 文本协议需要配套接收端；不能直接假定 PX4 原生接受这些格式。0524/0526 部分方法仍为日志桩。0528 的 `MAV_CMD_DO_MOTOR_TEST` 能否执行取决于 PX4 版本、配置和命令反馈；发送成功只表示串口写入完成。

## 本次最小修复

- 统一早期文件名与导入名，修复 `perception` 拼写、缺失导入、未定义传感器常量及返回值；调整 LED 默认闪烁逻辑，避免初始化被无限等待阻塞。
- 修复两份 0525 MAVLink 版本中的方法作用域和报文排列；修正相关版本的心跳、COMMAND_LONG、SET_MODE、位置/速度目标、CRC 和掩码，补上 0528 的流接收缓冲与缺失接口。
- 0528 初始化保留未解锁状态，READY 状态不自动发送前进速度；电机测试先请求上锁，再根据飞控心跳状态进入测试流程。
- 修复 Arduino 调试草图的括号、重复定义，修复拆分模块和辅助库中明确的字段、掩码和函数实现错误，统一使用随附库的头文件入口。
- 调整 Arduino / ESP-IDF 的 OFFBOARD 初始化次序，先发送设定点，再请求模式和解锁，并等待飞控反馈；ESP-IDF 状态更新加锁，移除持有状态锁时的秒级延迟。
- 修正航段距离积分跨状态累计和坐标投影、降落等待的计时起点及完成消息中的固定日期。日历时间使用降落时捕获的系统时钟；时钟未设置时明确报告不可用，并附降落时的启动计时值。
- 修复已有桌面工具的目录导入、测试发现与返回码、缺失 Tkinter 导入和 `mpremote` 参数；保留原有测试设计及其局限。
- 将 Wi-Fi 凭据替换为占位符，保留本地私有配置入口；电池分压比与板卡接线改为显式配置，并整理构建路径。

## 功能边界

当前代码覆盖高层任务状态机、路线和通信原型、基础测距与滤波、按键/LED、电机命令和完成消息。多个快照会自动进入起飞任务，具体行为由所选版本入口决定。

完整的 OpenMV 视觉检测、机械夹爪驱动与抓取成功反馈、真实水平定位融合、NTP/RTC 同步尚未在该代码集中形成完整实现。早期“抓取/投放”步骤存在日志、注释或标志位实现；较晚 C/Arduino 路线的任务四、五是区域穿越。将 `SIMULATION_MODE` 改为 `False` 本身不会补齐定位系统，ESP-IDF 的简化 VL53L0X 读取也不等于完整传感器驱动。

本次修复保留这些原型边界，没有补造缺失的硬件功能，也没有据此宣称某个快照已完成实机任务。运行效果仍需由使用者在相应工具链、板卡、飞控和接线上确认。

## 署名与许可

团队源码保留原文件中的署名，本次未新增统一许可证。MAVLink-Arduino 的上游 MIT 许可证位于 [`libraries/MAVLink/LICENSE.txt`](libraries/MAVLink/LICENSE.txt)，依赖来源见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。
