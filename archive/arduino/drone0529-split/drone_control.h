/**
 * @file drone_control.h
 * @brief Score99团队无人机控制系统头文件
 * @author Team Score99
 * @date 2025
 */

#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <time.h>
#include <MAVLink.h>

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

// PX4状态数据结构
struct PX4State {
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
};

// 全局变量声明
extern HardwareSerial SerialMAV;
extern HardwareSerial SerialHC12;
extern TaskState currentTask;
extern float taskTimer;
extern float taskStartTime;
extern float distanceTraveled;
extern float targetDistance;
extern float targetYaw;
extern float initialYaw;
extern bool isArmed;
extern bool inOffboardMode;
extern bool connectionLost;
extern uint32_t lastHeartbeat;
extern uint32_t lastAttitudeUpdate;
extern PX4State px4State;
extern float laserHeight;

// 函数声明
void initializeSystem();
void receiveMavlinkMessages();
void handleHeartbeat(mavlink_message_t& msg);
void handleAttitude(mavlink_message_t& msg);
void handleLocalPosition(mavlink_message_t& msg);
void handleHighresIMU(mavlink_message_t& msg);
void handleCommandAck(mavlink_message_t& msg);
void executeTask();
void sendHeartbeat();
void setOffboardMode();
void armDisarm(bool arm);
void sendPositionTarget(float x, float y, float z, float yaw);
void sendVelocityTarget(float vx, float vy, float vz, float yaw_rate);
void updateDistanceTraveled();
void emergencyStop();
void sendMissionComplete();
const char* getFlightModeName(uint32_t custom_mode);
void printStatus();

#endif // DRONE_CONTROL_H