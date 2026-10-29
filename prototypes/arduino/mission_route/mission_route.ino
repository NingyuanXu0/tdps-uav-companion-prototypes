/**
 * @file Score99_Drone_PX4.ino
 * @brief Score99团队无人机自主飞行控制程序（PX4 + MAVLink 2.0）
 * @author Team Score99
 * @date 2025
 * 
 * 硬件连接：
 * - ESP32-S3 TX2 (GPIO17) -> PX4 TELEM2 RX
 * - ESP32-S3 RX2 (GPIO16) -> PX4 TELEM2 TX
 * - ESP32-S3 GND -> PX4 GND
 * - VL53L0X: 通过 TDPS_I2C_SDA / TDPS_I2C_SCL 配置实际引脚
 * - HC12: TX(GPIO4), RX(GPIO5)
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <time.h>
#include <MAVLink.h>  // 需要安装mavlink库

// 串口定义
HardwareSerial SerialMAV(2);  // 使用Serial2连接PX4
HardwareSerial SerialHC12(1); // 使用Serial1连接HC12

// 按实际接线覆盖；ESP32-S3 没有 GPIO22，未配置时禁用示例 I2C。
#ifndef TDPS_I2C_SDA
#define TDPS_I2C_SDA 21
#endif
#ifndef TDPS_I2C_SCL
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define TDPS_I2C_SCL -1
#else
#define TDPS_I2C_SCL 22
#endif
#endif

// 串口配置
#define MAV_SERIAL_BAUD 57600
#define HC12_SERIAL_BAUD 9600
#define DEBUG_SERIAL_BAUD 115200

// MAVLink配置
#define MAVLINK_SYSTEM_ID 255    // GCS系统ID
#define MAVLINK_COMPONENT_ID 190 // 伴随计算机组件ID
#define TARGET_SYSTEM_ID 1       // PX4系统ID
#define TARGET_COMPONENT_ID 1    // PX4组件ID

// 飞行参数
#define TAKEOFF_ALTITUDE 3.0f    // 起飞高度（米）
#define CRUISE_SPEED 0.5f        // 巡航速度（米/秒）
#define TURN_RATE 30.0f          // 转向速率（度/秒）
#define OBSTACLE_HEIGHT 2.5f     // 避障高度（米）

// 任务时间定义（秒）
#define STARTUP_DELAY 5.0f
#define TUNNEL_DISTANCE_1 2.0f   // 隧道1距离（米）
#define TUNNEL_DISTANCE_2 2.0f   // 隧道2距离（米）
#define TUNNEL_DISTANCE_3 2.0f   // 隧道3距离（米）
#define TURN_ANGLE 90.0f         // 转向角度（度）
#define OBSTACLE_DISTANCE 3.0f   // 避障段距离（米）
#define TASK4_DISTANCE 1.5f      // 任务4距离（米）
#define TASK5_DISTANCE 1.5f      // 任务5距离（米）
#define LANDING_WAIT_TIME 15.0f  // 降落后等待时间（秒）

// 任务状态枚举
enum TaskState {
    TASK_INIT = 0,
    TASK_WAIT_CONNECTION,
    TASK_STARTUP_DELAY,
    TASK_ARM_TAKEOFF,
    TASK_TAKEOFF_CLIMB,
    TASK_TUNNEL_1,
    TASK_TUNNEL_2,
    TASK_TUNNEL_3,
    TASK_TURN_RIGHT,
    TASK_OBSTACLE_AVOID,
    TASK_AREA_4,
    TASK_AREA_5,
    TASK_LANDING,
    TASK_WAIT_LANDED,
    TASK_COMMUNICATION,
    TASK_COMPLETE,
    TASK_EMERGENCY_STOP
};

// 全局变量
TaskState currentTask = TASK_INIT;
float taskTimer = 0.0f;
float taskStartTime = 0.0f;
float distanceTraveled = 0.0f;
float targetDistance = 0.0f;
float targetYaw = 0.0f;
float initialYaw = 0.0f;
bool isArmed = false;
bool inOffboardMode = false;
bool connectionLost = false;
uint32_t lastHeartbeat = 0;
uint32_t lastAttitudeUpdate = 0;
static uint32_t lastPositionUpdate = 0;
static time_t landingTime = 0;
static uint32_t landingUptime = 0;

// PX4状态数据
struct {
    float roll;         // 滚转角（度）
    float pitch;        // 俯仰角（度）
    float yaw;          // 偏航角（度）
    float x;            // 本地 NED X 位置（米）
    float y;            // 本地 NED Y 位置（米）
    float altitude;     // 高度（米）
    float vx;           // X轴速度（米/秒）
    float vy;           // Y轴速度（米/秒）
    float vz;           // Z轴速度（米/秒）
    float rollRate;     // 滚转角速度（弧度/秒）
    float pitchRate;    // 俯仰角速度（弧度/秒）
    float yawRate;      // 偏航角速度（弧度/秒）
    float ax;           // X轴加速度（米/秒²）
    float ay;           // Y轴加速度（米/秒²）
    float az;           // Z轴加速度（米/秒²）
    uint32_t mode;       // 飞行模式
    bool armed;         // 解锁状态
} px4State;

// VL53L0X传感器（简化版）
float laserHeight = 0.0f;

void setup() {
    // 初始化调试串口
    Serial.begin(DEBUG_SERIAL_BAUD);
    delay(1000);
    
    Serial.println("===========================================");
    Serial.println("Score99团队无人机控制系统启动");
    Serial.println("版本: 1.0.0 (PX4 + MAVLink 2.0)");
    Serial.println("===========================================");
    
    // 初始化MAVLink串口（连接PX4）
    SerialMAV.begin(MAV_SERIAL_BAUD, SERIAL_8N1, 16, 17);  // RX=16, TX=17
    Serial.println("MAVLink串口初始化完成 (57600bps)");
    
    // 初始化HC12串口
    SerialHC12.begin(HC12_SERIAL_BAUD, SERIAL_8N1, 5, 4);  // RX=5, TX=4
    Serial.println("HC12串口初始化完成 (9600bps)");
    
    // 初始化I2C（VL53L0X）
    if (TDPS_I2C_SDA >= 0 && TDPS_I2C_SCL >= 0) {
        Wire.begin(TDPS_I2C_SDA, TDPS_I2C_SCL);
        Serial.println("I2C总线初始化完成");
    } else {
        Serial.println("I2C示例已禁用：请配置实际板卡的 TDPS_I2C_SDA / TDPS_I2C_SCL");
    }
    
    // 初始化状态
    currentTask = TASK_WAIT_CONNECTION;
    taskStartTime = millis() / 1000.0f;
    
    Serial.println("\n等待与PX4建立连接...");
}

void loop() {
    // 接收并处理MAVLink消息
    receiveMavlinkMessages();
    
    // 更新任务计时器
    float currentTime = millis() / 1000.0f;
    taskTimer = currentTime - taskStartTime;
    
    // 检查心跳超时
    if (millis() - lastHeartbeat > 3000 && currentTask != TASK_WAIT_CONNECTION) {
        Serial.println("\n[错误] MAVLink连接丢失！执行紧急制动");
        emergencyStop();
        return;
    }
    
    // 定期输出状态信息（每500ms）
    static uint32_t lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 500) {
        printStatus();
        lastStatusPrint = millis();
    }
    
    // 执行任务状态机
    executeTask();
    
    // 发送心跳（1Hz）
    static uint32_t lastHeartbeatSent = 0;
    if (millis() - lastHeartbeatSent > 1000) {
        sendHeartbeat();
        lastHeartbeatSent = millis();
    }
    delay(20);  // 约 50 Hz 的设定点循环
}

// 接收并处理MAVLink消息
void receiveMavlinkMessages() {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (SerialMAV.available()) {
        uint8_t byte = SerialMAV.read();
        
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            if (msg.sysid != TARGET_SYSTEM_ID || msg.compid != TARGET_COMPONENT_ID) continue;
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
                    
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    handleCommandAck(msg);
                    break;
            }
        }
    }
}

// 处理心跳消息
void handleHeartbeat(mavlink_message_t& msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    
    lastHeartbeat = millis();
    px4State.mode = heartbeat.custom_mode;
    px4State.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
    
    // 检查是否在OFFBOARD模式
    inOffboardMode = (((heartbeat.custom_mode >> 16) & 0xff) == 6);  // PX4 OFFBOARD模式值
    
    if (currentTask == TASK_WAIT_CONNECTION) {
        Serial.println("\n[信息] 成功连接到PX4飞控");
        Serial.printf("飞行模式: %s\n", getFlightModeName(heartbeat.custom_mode));
        Serial.printf("解锁状态: %s\n", px4State.armed ? "已解锁" : "未解锁");
        currentTask = TASK_STARTUP_DELAY;
        taskStartTime = millis() / 1000.0f;
    }
}

// 处理姿态消息
void handleAttitude(mavlink_message_t& msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&msg, &attitude);
    
    px4State.roll = attitude.roll * 180.0f / PI;
    px4State.pitch = attitude.pitch * 180.0f / PI;
    px4State.yaw = attitude.yaw * 180.0f / PI;
    px4State.rollRate = attitude.rollspeed;
    px4State.pitchRate = attitude.pitchspeed;
    px4State.yawRate = attitude.yawspeed;
    
    lastAttitudeUpdate = millis();
}

// 处理本地位置消息
void handleLocalPosition(mavlink_message_t& msg) {
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(&msg, &pos);
    
    px4State.x = pos.x;
    px4State.y = pos.y;
    lastPositionUpdate = millis();
    px4State.altitude = -pos.z;  // NED坐标系，Z轴向下
    px4State.vx = pos.vx;
    px4State.vy = pos.vy;
    px4State.vz = pos.vz;
}

// 处理高精度IMU消息
void handleHighresIMU(mavlink_message_t& msg) {
    mavlink_highres_imu_t imu;
    mavlink_msg_highres_imu_decode(&msg, &imu);
    
    px4State.ax = imu.xacc;
    px4State.ay = imu.yacc;
    px4State.az = imu.zacc;
}

// 处理命令确认消息
void handleCommandAck(mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    
    if (ack.result != MAV_RESULT_ACCEPTED) {
        Serial.printf("\n[警告] 命令执行失败，命令ID: %d, 结果: %d\n", 
                      ack.command, ack.result);
    }
}

// 执行任务状态机
void executeTask() {
    switch (currentTask) {
        case TASK_INIT:
            // 初始化状态
            break;
            
        case TASK_WAIT_CONNECTION:
            // 等待连接
            break;
            
        case TASK_STARTUP_DELAY:
            if (taskTimer >= STARTUP_DELAY) {
                Serial.println("\n[任务] 启动延迟完成，准备起飞");
                currentTask = TASK_ARM_TAKEOFF;
                taskStartTime = millis() / 1000.0f;
            }
            break;
            
        case TASK_ARM_TAKEOFF: {
            // 先连续发送保持当前位置的设定点，再申请 OFFBOARD 和解锁。
            static uint32_t lastCommandTime = 0;
            if (lastPositionUpdate == 0 || millis() - lastPositionUpdate > 1000) {
                break;
            }
            sendPositionTarget(px4State.x, px4State.y, -px4State.altitude,
                               px4State.yaw * PI / 180.0f);
            if (taskTimer >= 1.2f && millis() - lastCommandTime >= 1000) {
                if (!inOffboardMode) {
                    setOffboardMode();
                } else if (!px4State.armed) {
                    armDisarm(true);
                }
                lastCommandTime = millis();
            }
            if (inOffboardMode && px4State.armed) {
                Serial.println("[任务] 开始起飞到3米高度");
                currentTask = TASK_TAKEOFF_CLIMB;
                taskStartTime = millis() / 1000.0f;
            }
            break;
        }
            
        case TASK_TAKEOFF_CLIMB:
            // 起飞爬升
            sendPositionTarget(0, 0, -TAKEOFF_ALTITUDE, 0);
            
            if (px4State.altitude >= TAKEOFF_ALTITUDE * 0.9f) {
                Serial.println("\n[任务] 起飞完成，开始隧道穿越第1段");
                currentTask = TASK_TUNNEL_1;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = TUNNEL_DISTANCE_1;
            }
            break;
            
        case TASK_TUNNEL_1:
            // 隧道第1段 - 向前飞行
            sendVelocityTarget(CRUISE_SPEED, 0, 0, 0);
            updateDistanceTraveled();
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 隧道第1段完成，进入第2段");
                currentTask = TASK_TUNNEL_2;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = TUNNEL_DISTANCE_2;
            }
            break;
            
        case TASK_TUNNEL_2:
            // 隧道第2段
            sendVelocityTarget(CRUISE_SPEED, 0, 0, 0);
            updateDistanceTraveled();
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 隧道第2段完成，进入第3段");
                currentTask = TASK_TUNNEL_3;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = TUNNEL_DISTANCE_3;
            }
            break;
            
        case TASK_TUNNEL_3:
            // 隧道第3段
            sendVelocityTarget(CRUISE_SPEED, 0, 0, 0);
            updateDistanceTraveled();
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 隧道穿越完成，开始右转90度");
                currentTask = TASK_TURN_RIGHT;
                taskStartTime = millis() / 1000.0f;
                initialYaw = px4State.yaw;
                targetYaw = initialYaw + TURN_ANGLE;
            }
            break;
            
        case TASK_TURN_RIGHT: {
            // 右转90度
            sendVelocityTarget(0, 0, 0, TURN_RATE);
            
            float yawDiff = abs(px4State.yaw - targetYaw);
            if (yawDiff > 180) yawDiff = 360 - yawDiff;
            
            if (yawDiff < 5.0f) {
                Serial.println("\n[任务] 转向完成，开始避障飞行");
                currentTask = TASK_OBSTACLE_AVOID;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = OBSTACLE_DISTANCE;
            }
            break;
        }
            
        case TASK_OBSTACLE_AVOID:
            // 避障飞行 - 保持2.5米以上高度
            {
                float heightError = OBSTACLE_HEIGHT - px4State.altitude;
                float vz = 0;
                if (heightError > 0.1f) vz = -0.3f;  // 需要上升
                else if (heightError < -0.1f) vz = 0.3f;  // 需要下降
                
                sendVelocityTarget(CRUISE_SPEED, 0, vz, 0);
                updateDistanceTraveled();
            }
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 避障完成，进入任务4空域");
                currentTask = TASK_AREA_4;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = TASK4_DISTANCE;
            }
            break;
            
        case TASK_AREA_4:
            // 任务4空域穿越
            sendVelocityTarget(CRUISE_SPEED, 0, 0, 0);
            updateDistanceTraveled();
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 任务4空域完成，进入任务5空域");
                currentTask = TASK_AREA_5;
                taskStartTime = millis() / 1000.0f;
                distanceTraveled = 0.0f;
                targetDistance = TASK5_DISTANCE;
            }
            break;
            
        case TASK_AREA_5:
            // 任务5空域穿越
            sendVelocityTarget(CRUISE_SPEED, 0, 0, 0);
            updateDistanceTraveled();
            
            if (distanceTraveled >= targetDistance) {
                Serial.println("\n[任务] 任务5空域完成，开始降落");
                currentTask = TASK_LANDING;
                taskStartTime = millis() / 1000.0f;
            }
            break;
            
        case TASK_LANDING:
            // 降落
            sendVelocityTarget(0, 0, 0.5f, 0);  // 向下0.5米/秒
            
            if (px4State.altitude < 0.2f) {
                Serial.println("\n[任务] 降落完成，等待15秒");
                landingTime = time(nullptr);
                landingUptime = millis();
                currentTask = TASK_WAIT_LANDED;
                taskStartTime = millis() / 1000.0f;
            }
            break;
            
        case TASK_WAIT_LANDED:
            // 停止所有运动
            sendVelocityTarget(0, 0, 0, 0);
            
            if (taskTimer >= LANDING_WAIT_TIME) {
                currentTask = TASK_COMMUNICATION;
                taskStartTime = millis() / 1000.0f;
            }
            break;
            
        case TASK_COMMUNICATION:
            // 发送任务完成消息
            sendMissionComplete();
            armDisarm(false);  // 上锁
            currentTask = TASK_COMPLETE;
            Serial.println("\n[任务] 所有任务完成！");
            break;
            
        case TASK_COMPLETE:
            // 任务完成
            break;
            
        case TASK_EMERGENCY_STOP:
            // 紧急停止
            sendVelocityTarget(0, 0, 0, 0);
            break;
    }
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
}

// 设置OFFBOARD模式
void setOffboardMode() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // PX4 OFFBOARD模式的custom_mode值
    uint32_t custom_mode = 393216;  // 0x60000
    
    mavlink_msg_set_mode_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                              TARGET_SYSTEM_ID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                              custom_mode);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    Serial.println("[命令] 发送OFFBOARD模式切换命令");
}

// 解锁/上锁
void armDisarm(bool arm) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                  TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
                                  MAV_CMD_COMPONENT_ARM_DISARM,
                                  0, // confirmation
                                  arm ? 1.0f : 0.0f, // param1
                                  0, 0, 0, 0, 0, 0); // param2-7
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
    
    Serial.printf("[命令] 发送%s命令\n", arm ? "解锁" : "上锁");
}

// 发送位置目标（NED坐标系）
void sendPositionTarget(float x, float y, float z, float yaw) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    uint16_t type_mask = 0x09F8;  // 使用位置和偏航，忽略速度/加速度/偏航速率
    
    mavlink_msg_set_position_target_local_ned_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        millis(), TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
        MAV_FRAME_LOCAL_NED, type_mask,
        x, y, z,  // 位置
        0, 0, 0,  // 速度
        0, 0, 0,  // 加速度
        yaw, 0);  // 偏航，偏航速率
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
}

// 发送速度目标（机体坐标系）
void sendVelocityTarget(float vx, float vy, float vz, float yaw_rate) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    uint16_t type_mask = 0x05C7;  // 使用速度和偏航速率
    
    mavlink_msg_set_position_target_local_ned_pack(
        MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
        millis(), TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
        MAV_FRAME_BODY_NED, type_mask,  // 使用机体坐标系
        0, 0, 0,  // 位置
        vx, vy, vz,  // 速度
        0, 0, 0,  // 加速度
        0, yaw_rate * PI / 180.0f);  // 偏航，偏航速率（转换为弧度）
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
}

// 更新行进距离
void updateDistanceTraveled() {
    static uint32_t lastUpdateTime = 0;
    static int lastTask = -1;
    uint32_t currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    if (lastTask == static_cast<int>(currentTask) && lastUpdateTime != 0 && dt < 1.0f) {
        float yaw = px4State.yaw * PI / 180.0f;
        // LOCAL_POSITION_NED 的水平速度投影到机体前向。
        distanceTraveled += (px4State.vx * cosf(yaw) + px4State.vy * sinf(yaw)) * dt;
    }
    lastTask = static_cast<int>(currentTask);
    lastUpdateTime = currentTime;
}

// 紧急停止
void emergencyStop() {
    currentTask = TASK_EMERGENCY_STOP;
    sendVelocityTarget(0, 0, 0, 0);
    connectionLost = true;
}

// 发送任务完成消息
void sendMissionComplete() {
    char message[128];
    if (landingTime >= 1577836800) {  // RTC 已设置到 2020 年或之后
        struct tm timeinfo;
        gmtime_r(&landingTime, &timeinfo);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        snprintf(message, sizeof(message), "Mission Accomplished! Landing time: %s\r\n", timestamp);
    } else {
        snprintf(message, sizeof(message),
                 "Mission Accomplished! Landing clock unavailable; uptime_ms=%lu\r\n",
                 static_cast<unsigned long>(landingUptime));
    }
    SerialHC12.print(message);
    Serial.print("[HC12] ");
    Serial.print(message);
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

// 打印状态信息
void printStatus() {
    Serial.println("\n--- 飞控状态 ---");
    Serial.printf("姿态: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n", 
                  px4State.roll, px4State.pitch, px4State.yaw);
    Serial.printf("高度: %.2f米\n", px4State.altitude);
    Serial.printf("速度: Vx=%.2f Vy=%.2f Vz=%.2f (m/s)\n", 
                  px4State.vx, px4State.vy, px4State.vz);
    Serial.printf("角速度: Roll=%.2f Pitch=%.2f Yaw=%.2f (rad/s)\n",
                  px4State.rollRate, px4State.pitchRate, px4State.yawRate);
    Serial.printf("加速度: Ax=%.2f Ay=%.2f Az=%.2f (m/s²)\n",
                  px4State.ax, px4State.ay, px4State.az);
    Serial.printf("模式: %s | 解锁: %s | 任务: %d | 行进: %.2fm\n",
                  getFlightModeName(px4State.mode), 
                  px4State.armed ? "是" : "否",
                  currentTask, distanceTraveled);
    Serial.println("----------------");
}