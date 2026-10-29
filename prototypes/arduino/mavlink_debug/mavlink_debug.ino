/**
 * @file esp32_px4_debug.ino
 * @brief ESP32与PX4 MAVLink通信调试程序 (MAVLink官方标准合规)
 * @author Team Score99
 * @date 2025
 * 
 * 功能：
 * 1. 测试ESP32与PX4的MAVLink通信
 * 2. 显示详细的连接和消息状态
 * 3. 监控接收到的MAVLink消息
 * 4. 测试发送心跳和基本命令
 * 
 * MAVLink合规性：
 * ✅ 所有消息均使用MAVLink官方标准协议
 * ✅ 替换已废弃的REQUEST_DATA_STREAM为MAV_CMD_SET_MESSAGE_INTERVAL
 * ✅ 替换已废弃的SET_MODE为MAV_CMD_DO_SET_MODE
 * ✅ 使用标准的COMMAND_LONG协议发送命令
 * ✅ 符合MAVLink 2.0规范
 * 
 * 硬件连接：
 * - ESP32-S3 TX2 (GPIO17) -> PX4 TELEM2 RX
 * - ESP32-S3 RX2 (GPIO16) -> PX4 TELEM2 TX
 * - ESP32-S3 GND -> PX4 GND
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <MAVLink.h>

// 串口配置
#define MAV_SERIAL_BAUD 57600     // MAVLink串口波特率
#define DEBUG_SERIAL_BAUD 115200  // 调试串口波特率

// MAVLink配置
#define MAVLINK_SYSTEM_ID 255     // GCS系统ID
#define MAVLINK_COMPONENT_ID 190  // 伴随计算机组件ID
#define TARGET_SYSTEM_ID 1        // PX4系统ID
#define TARGET_COMPONENT_ID 1     // PX4组件ID

// 调试配置
#define HEARTBEAT_INTERVAL 1000   // 心跳发送间隔(ms)
#define STATUS_PRINT_INTERVAL 2000 // 状态打印间隔(ms)
#define CONNECTION_TIMEOUT 5000    // 连接超时时间(ms)

// MAVLink命令枚举值 (确保使用官方标准值)
#ifndef MAV_CMD_COMPONENT_ARM_DISARM
#define MAV_CMD_COMPONENT_ARM_DISARM 400
#endif

#ifndef MAV_CMD_DO_SET_MODE
#define MAV_CMD_DO_SET_MODE 176
#endif

#ifndef MAV_CMD_SET_MESSAGE_INTERVAL
#define MAV_CMD_SET_MESSAGE_INTERVAL 511
#endif

#ifndef MAV_CMD_REQUEST_MESSAGE
#define MAV_CMD_REQUEST_MESSAGE 512
#endif

// MAVLink结果枚举值 (确保定义正确)
#ifndef MAV_RESULT_ACCEPTED
#define MAV_RESULT_ACCEPTED 0
#define MAV_RESULT_TEMPORARILY_REJECTED 1
#define MAV_RESULT_DENIED 2
#define MAV_RESULT_UNSUPPORTED 3
#define MAV_RESULT_FAILED 4
#define MAV_RESULT_IN_PROGRESS 5
#define MAV_RESULT_CANCELLED 6
#endif

// MAVLink模式标志
#ifndef MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED 1
#define MAV_MODE_FLAG_SAFETY_ARMED 128
#endif

// 简化的任务状态枚举 (用于状态机，虽然本调试程序主要用于通信测试)
enum TaskState {
    TASK_INIT = 0,
    TASK_WAIT_CONNECTION,
    TASK_CONNECTED,
    TASK_TESTING
};

// 串口对象
HardwareSerial SerialMAV(2);  // 使用Serial2连接PX4

// 通信状态结构体
struct CommStatus {
    // 连接状态
    bool isConnected = false;
    uint32_t lastHeartbeat = 0;
    uint32_t connectionStartTime = 0;
    
    // 消息计数
    uint32_t totalMessagesReceived = 0;
    uint32_t heartbeatCount = 0;
    uint32_t attitudeCount = 0;
    uint32_t positionCount = 0;
    uint32_t imuCount = 0;
    uint32_t otherCount = 0;
    
    // 发送计数
    uint32_t heartbeatsSent = 0;
    uint32_t commandsSent = 0;
    
    // PX4状态
    uint32_t px4Mode = 0;
    bool px4Armed = false;
    float px4Roll = 0;
    float px4Pitch = 0;
    float px4Yaw = 0;
    float px4Altitude = 0;
    
    // 错误统计
    uint32_t parseErrors = 0;
    uint32_t checksumErrors = 0;
};

CommStatus commStatus;

void setup() {
    // 启动延迟确保串口稳定
    delay(3000);
    
    // 初始化调试串口
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.println();
    Serial.println("========================================");
    Serial.println("    ESP32-PX4 MAVLink通信调试器");
    Serial.println("    Version: 1.0.0");
    Serial.println("    Team Score99");
    Serial.println("========================================");
    
    // 显示硬件信息
    Serial.println("\n[硬件信息]");
    Serial.printf("芯片型号: %s\n", ESP.getChipModel());
    Serial.printf("CPU频率: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Flash大小: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("可用内存: %d KB\n", ESP.getFreeHeap() / 1024);
    
    // 初始化MAVLink串口
    Serial.println("\n[串口初始化]");
    Serial.printf("MAVLink串口波特率: %d\n", MAV_SERIAL_BAUD);
    Serial.println("引脚配置: RX=GPIO16, TX=GPIO17");
    
    SerialMAV.begin(MAV_SERIAL_BAUD, SERIAL_8N1, 16, 17);
    delay(1000);
    
    if (SerialMAV) {
        Serial.println("✓ MAVLink串口初始化成功");
    } else {
        Serial.println("✗ MAVLink串口初始化失败");
        return;
    }
    
    // 初始化通信状态
    commStatus.connectionStartTime = millis();
    
    Serial.println("\n[开始调试]");
    Serial.println("等待PX4连接...");
    Serial.println("请确保PX4已启动并且TELEM2端口已配置");
    Serial.println();
}

void loop() {
    // 处理接收的MAVLink消息
    processReceivedMessages();
    
    // 发送心跳（定期）
    static uint32_t lastHeartbeatSent = 0;
    if (millis() - lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeatSent = millis();
    }
    
    // 打印状态信息（定期）
    static uint32_t lastStatusPrint = 0;
    if (millis() - lastStatusPrint >= STATUS_PRINT_INTERVAL) {
        printDetailedStatus();
        lastStatusPrint = millis();
    }
    
    // 检查连接状态
    checkConnectionStatus();
    
    // 处理串口命令（如果有的话）
    processSerialCommands();
    
    delay(10); // 小延迟避免过度占用CPU
}

// 处理接收的MAVLink消息
void processReceivedMessages() {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (SerialMAV.available()) {
        uint8_t byte = SerialMAV.read();
        
        // 解析MAVLink消息
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            commStatus.totalMessagesReceived++;
            
            // 处理不同类型的消息
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    handleHeartbeat(msg);
                    break;
                    
                case MAVLINK_MSG_ID_ATTITUDE:
                    handleAttitude(msg);
                    break;
                    
                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                    handleLocalPosition(msg);
                    break;
                    
                case MAVLINK_MSG_ID_HIGHRES_IMU:
                    handleHighresIMU(msg);
                    break;
                    
                case MAVLINK_MSG_ID_SYS_STATUS:
                    handleSysStatus(msg);
                    break;
                    
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    handleCommandAck(msg);
                    break;
                    
                default:
                    commStatus.otherCount++;
                    // 显示未处理的消息（可选）
                    Serial.printf("[其他消息] ID: %d, 来源: %d.%d\n", 
                                  msg.msgid, msg.sysid, msg.compid);
                    break;
            }
            
            // 首次收到消息时标记为已连接
            if (!commStatus.isConnected) {
                commStatus.isConnected = true;
                Serial.println("\n🎉 [连接成功] 已与PX4建立MAVLink通信!");
                Serial.printf("连接用时: %.1f秒\n", 
                              (millis() - commStatus.connectionStartTime) / 1000.0f);
                Serial.println();
            }
        } else {
            // 解析错误统计
            if (status.parse_error) {
                commStatus.parseErrors++;
            }
            if (status.msg_received == MAVLINK_FRAMING_BAD_CRC) {
                commStatus.checksumErrors++;
            }
        }
    }
}

// 处理心跳消息
void handleHeartbeat(mavlink_message_t& msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    
    commStatus.heartbeatCount++;
    commStatus.lastHeartbeat = millis();
    commStatus.px4Mode = heartbeat.custom_mode;
    commStatus.px4Armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
    
    Serial.printf("[心跳] 模式: %s | 解锁: %s | 系统: %d.%d\n",
                  getFlightModeName(heartbeat.custom_mode),
                  commStatus.px4Armed ? "是" : "否",
                  msg.sysid, msg.compid);
}

// 处理姿态消息
void handleAttitude(mavlink_message_t& msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&msg, &attitude);
    
    commStatus.attitudeCount++;
    commStatus.px4Roll = attitude.roll * 180.0f / PI;
    commStatus.px4Pitch = attitude.pitch * 180.0f / PI;
    commStatus.px4Yaw = attitude.yaw * 180.0f / PI;
    
    // 每10次显示一次姿态信息
    if (commStatus.attitudeCount % 10 == 0) {
        Serial.printf("[姿态] Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f°\n",
                      commStatus.px4Roll, commStatus.px4Pitch, commStatus.px4Yaw);
    }
}

// 处理位置消息
void handleLocalPosition(mavlink_message_t& msg) {
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(&msg, &pos);
    
    commStatus.positionCount++;
    commStatus.px4Altitude = -pos.z;  // NED坐标系
    
    // 每10次显示一次位置信息
    if (commStatus.positionCount % 10 == 0) {
        Serial.printf("[位置] X: %.2f | Y: %.2f | 高度: %.2f米\n",
                      pos.x, pos.y, commStatus.px4Altitude);
    }
}

// 处理高精度IMU消息
void handleHighresIMU(mavlink_message_t& msg) {
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(&msg, &imu);
    
    commStatus.imuCount++;
    
    // 每20次显示一次IMU信息
    if (commStatus.imuCount % 20 == 0) {
        Serial.printf("[IMU] 加速度: (%.2f, %.2f, %.2f) | 陀螺: (%.2f, %.2f, %.2f)\n",
                      imu.xacc, imu.yacc, imu.zacc,
                      imu.xgyro, imu.ygyro, imu.zgyro);
    }
}

// 处理系统状态消息
void handleSysStatus(mavlink_message_t& msg) {
    mavlink_sys_status_t status;
    mavlink_msg_sys_status_decode(&msg, &status);
    
    Serial.printf("[系统状态] 电压: %.1fV | 电流: %.1fA | 电量: %d%%\n",
                  status.voltage_battery / 1000.0f,
                  status.current_battery / 100.0f,
                  status.battery_remaining);
}

// 处理命令确认消息
void handleCommandAck(mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    
    // 解析命令名称
    const char* cmdName = getCommandName(ack.command);
    const char* result = getResultName(ack.result);
    
    Serial.printf("[命令确认] %s | 结果: %s",
                  cmdName, result);
    
    // 显示额外的结果参数（如果有）
    if (ack.result_param2 != 0) {
        Serial.printf(" | 详情: %d", ack.result_param2);
    }
    Serial.println();
}

// 发送心跳
void sendHeartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                               MAV_TYPE_ONBOARD_CONTROLLER, MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_ACTIVE);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    commStatus.heartbeatsSent++;
}

// 检查连接状态
void checkConnectionStatus() {
    if (commStatus.isConnected) {
        uint32_t timeSinceLastHeartbeat = millis() - commStatus.lastHeartbeat;
        
        if (timeSinceLastHeartbeat > CONNECTION_TIMEOUT) {
            commStatus.isConnected = false;
            Serial.println("\n⚠️  [连接丢失] PX4心跳超时!");
            Serial.printf("最后心跳时间: %d秒前\n", timeSinceLastHeartbeat / 1000);
        }
    }
}

// 打印详细状态
void printDetailedStatus() {
    float uptime = millis() / 1000.0f;
    
    Serial.println("\n" + String("=================================================="));
    Serial.printf("运行时间: %.1f秒 | 连接状态: %s\n", 
                  uptime, commStatus.isConnected ? "已连接" : "未连接");
    
    if (commStatus.isConnected) {
        Serial.printf("最后心跳: %d秒前\n", 
                      (millis() - commStatus.lastHeartbeat) / 1000);
    }
    
    Serial.println("\n[消息统计]");
    Serial.printf("总接收: %d | 心跳: %d | 姿态: %d | 位置: %d | IMU: %d | 其他: %d\n",
                  commStatus.totalMessagesReceived,
                  commStatus.heartbeatCount,
                  commStatus.attitudeCount,
                  commStatus.positionCount,
                  commStatus.imuCount,
                  commStatus.otherCount);
    
    Serial.printf("发送心跳: %d | 命令: %d\n",
                  commStatus.heartbeatsSent,
                  commStatus.commandsSent);
    
    if (commStatus.parseErrors > 0 || commStatus.checksumErrors > 0) {
        Serial.printf("错误统计 - 解析: %d | 校验: %d\n",
                      commStatus.parseErrors,
                      commStatus.checksumErrors);
    }
    
    if (commStatus.isConnected) {
        Serial.println("\n[PX4状态]");
        Serial.printf("飞行模式: %s | 解锁: %s\n",
                      getFlightModeName(commStatus.px4Mode),
                      commStatus.px4Armed ? "是" : "否");
        Serial.printf("姿态: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n",
                      commStatus.px4Roll, commStatus.px4Pitch, commStatus.px4Yaw);
        Serial.printf("高度: %.2f米\n", commStatus.px4Altitude);
    }
    
    Serial.printf("\n[内存状态] 可用: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.println(String("==================================================") + "\n");
}

// 处理串口命令（简单测试命令）
void processSerialCommands() {
    if (!Serial.available()) return;
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "help" || command == "h") printHelp();
    else if (command == "status" || command == "s") printDetailedStatus();
    else if (command == "reset" || command == "r") resetStatistics();
    else if (command == "test" || command == "t") testCommand();
    else if (command == "arm") armDisarm(true);
    else if (command == "disarm") armDisarm(false);
    else if (command == "offboard") setOffboardMode();
    else Serial.println("未知命令，输入 help 查看帮助");
}

// 解锁/上锁
void armDisarm(bool arm) {
    if (!commStatus.isConnected) {
        Serial.printf("未连接到PX4，无法发送%s命令\n", arm ? "解锁" : "上锁");
        return;
    }
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
        MAV_CMD_COMPONENT_ARM_DISARM,  // 官方标准解锁/上锁命令
        0,  // confirmation
        arm ? 1.0f : 0.0f,  // param1: 1=解锁, 0=上锁
        0, 0, 0, 0, 0, 0);  // param2-7: 未使用
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    commStatus.commandsSent++;
    Serial.printf("[命令] 发送%s命令 (MAV_CMD_COMPONENT_ARM_DISARM)\n", arm ? "解锁" : "上锁");
}

// 打印帮助信息
void printHelp() {
    Serial.println("\n=== 调试命令帮助 ===");
    Serial.println("help (h)   - 显示此帮助");
    Serial.println("status (s) - 显示详细状态");
    Serial.println("reset (r)  - 重置统计数据");
    Serial.println("test (t)   - 发送测试命令 (SET_MESSAGE_INTERVAL)");
    Serial.println("arm        - 尝试解锁飞机");
    Serial.println("disarm     - 尝试上锁飞机"); 
    Serial.println("offboard   - 切换到OFFBOARD模式");
    Serial.println("==================");
    Serial.println("注意: 所有命令均使用MAVLink官方标准协议");
}

// 重置统计数据
void resetStatistics() {
    commStatus.totalMessagesReceived = 0;
    commStatus.heartbeatCount = 0;
    commStatus.attitudeCount = 0;
    commStatus.positionCount = 0;
    commStatus.imuCount = 0;
    commStatus.otherCount = 0;
    commStatus.heartbeatsSent = 0;
    commStatus.commandsSent = 0;
    commStatus.parseErrors = 0;
    commStatus.checksumErrors = 0;
    
    Serial.println("统计数据已重置");
}

// 设置OFFBOARD模式 (使用官方标准MAV_CMD_DO_SET_MODE命令)
void setOffboardMode() {
    if (!commStatus.isConnected) {
        Serial.println("未连接到PX4，无法发送OFFBOARD模式命令");
        return;
    }
    
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // 使用官方推荐的MAV_CMD_DO_SET_MODE命令
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
        MAV_CMD_DO_SET_MODE,  // 官方标准模式设置命令
        0,  // confirmation
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  // param1: base_mode
        6,  // param2: PX4 main_mode，DO_SET_MODE 使用未移位的主模式值
        0, 0, 0, 0, 0);  // param3-7: 未使用
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    commStatus.commandsSent++;
    Serial.println("[命令] 发送OFFBOARD模式切换命令 (MAV_CMD_DO_SET_MODE)");
}

// 发送测试命令
void testCommand() {
    if (!commStatus.isConnected) {
        Serial.println("未连接到PX4，无法发送测试命令");
        return;
    }
    
    // 使用官方推荐的MAV_CMD_SET_MESSAGE_INTERVAL命令请求ATTITUDE消息
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
        MAV_CMD_SET_MESSAGE_INTERVAL,  // 使用官方标准命令
        0,  // confirmation
        MAVLINK_MSG_ID_ATTITUDE,  // param1: 消息ID
        100000,  // param2: 间隔(微秒) - 10Hz
        0, 0, 0, 0, 0);  // param3-7: 未使用
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    commStatus.commandsSent++;
    Serial.println("已发送消息间隔设置命令 (ATTITUDE @10Hz)");
}

// 获取命令名称
const char* getCommandName(uint16_t command) {
    switch (command) {
        case MAV_CMD_COMPONENT_ARM_DISARM: return "ARM/DISARM";
        case MAV_CMD_DO_SET_MODE: return "SET_MODE";
        case MAV_CMD_SET_MESSAGE_INTERVAL: return "SET_MSG_INTERVAL";
        case MAV_CMD_REQUEST_MESSAGE: return "REQUEST_MESSAGE";
        default:
            static char unknown[20];
            sprintf(unknown, "CMD_%d", command);
            return unknown;
    }
}

// 获取结果名称
const char* getResultName(uint8_t result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "已接受";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "临时拒绝";
        case MAV_RESULT_DENIED: return "拒绝";
        case MAV_RESULT_UNSUPPORTED: return "不支持";
        case MAV_RESULT_FAILED: return "失败";
        case MAV_RESULT_IN_PROGRESS: return "进行中";
        case MAV_RESULT_CANCELLED: return "已取消";
        default: return "未知";
    }
}

// 获取飞行模式名称
const char* getFlightModeName(uint32_t custom_mode) {
    switch ((custom_mode >> 16) & 0xff) {
        case 1: return "MANUAL";
        case 2: return "ALTCTL";
        case 3: return "POSCTL";
        case 4: return "AUTO";
        case 5: return "ACRO";
        case 6: return "OFFBOARD";
        case 7: return "STABILIZED";
        case 8: return "RATTITUDE";
        default: return "UNKNOWN";
    }
}

