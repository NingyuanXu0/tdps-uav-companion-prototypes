#ifndef ACM_CORE_H
#define ACM_CORE_H

#include <stdio.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

// MAVLINK协议头文件 - ESP-IDF环境
#include "common/mavlink.h"         // MAVLINK库头文件

// 确保NAN宏定义可用
#ifndef NAN
#define NAN (0.0/0.0)
#endif

// ========================= MAVLINK类型掩码定义 =========================
// SET_POSITION_TARGET_LOCAL_NED消息的类型掩码（标准MAVLINK定义）
#define POSITION_TARGET_TYPEMASK_X_IGNORE       0x0001
#define POSITION_TARGET_TYPEMASK_Y_IGNORE       0x0002
#define POSITION_TARGET_TYPEMASK_Z_IGNORE       0x0004
#define POSITION_TARGET_TYPEMASK_VX_IGNORE      0x0008
#define POSITION_TARGET_TYPEMASK_VY_IGNORE      0x0010
#define POSITION_TARGET_TYPEMASK_VZ_IGNORE      0x0020
#define POSITION_TARGET_TYPEMASK_AX_IGNORE      0x0040
#define POSITION_TARGET_TYPEMASK_AY_IGNORE      0x0080
#define POSITION_TARGET_TYPEMASK_AZ_IGNORE      0x0100
#define POSITION_TARGET_TYPEMASK_FORCE_SET      0x0200
#define POSITION_TARGET_TYPEMASK_YAW_IGNORE     0x0400
#define POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE 0x0800

// ========================= 系统配置 =========================
#define TAG                    "ACM_CORE"
#define UART_PORT_NUM          UART_NUM_1      // 与飞控通信的UART端口
#define UART_BAUD_RATE         57600           // MAVLINK通信波特率
#define HC12_UART_PORT         UART_NUM_2      // HC12通信模块UART端口
#define HC12_BAUD_RATE         9600            // HC12通信波特率
#define BUF_SIZE               1024            // 缓冲区大小
#define I2C_MASTER_NUM         I2C_NUM_0       // I2C主控制器
#define I2C_MASTER_FREQ_HZ     100000          // I2C频率

// ========================= 硬件引脚配置 =========================
// UART引脚配置
#define UART_TXD_PIN           17              // 与飞控通信TX引脚
#define UART_RXD_PIN           16              // 与飞控通信RX引脚
#define HC12_TXD_PIN           4               // HC12模块TX引脚
#define HC12_RXD_PIN           5               // HC12模块RX引脚

// I2C引脚配置
#define I2C_SDA_PIN            CONFIG_TDPS_I2C_SDA              // I2C数据线
#define I2C_SCL_PIN            CONFIG_TDPS_I2C_SCL              // I2C时钟线

// 传感器I2C地址
#define VL53L0X_ADDR           0x29            // VL53L0X激光测距传感器地址

// ========================= 系统参数 =========================
#define AUTO_TAKEOFF_DELAY     5000            // 自动起飞延时(毫秒)
#define HEARTBEAT_INTERVAL     1000000         // 心跳间隔(微秒) - 1Hz
#define MAIN_TASK_FREQ         100             // 主任务频率(毫秒) - 10Hz
#define SENSOR_TASK_FREQ       50              // 传感器任务频率(毫秒) - 20Hz
#define POSITION_TOLERANCE     0.3             // 位置到达容差(米)
#define ALTITUDE_TOLERANCE     0.2             // 高度到达容差(米)

// ========================= 系统状态定义 =========================
// 系统状态枚举
typedef enum {
    SYSTEM_INIT = 0,                           // 系统初始化
    SYSTEM_READY,                              // 系统就绪
    SYSTEM_ARMED,                              // 系统解锁
    SYSTEM_MISSION,                            // 执行任务中
    SYSTEM_ERROR,                              // 系统错误
    SYSTEM_LANDED                              // 已降落
} system_state_t;

// 任务状态枚举
typedef enum {
    TASK_IDLE = 0,                             // 空闲状态
    TASK_AUTO_TAKEOFF,                         // 自动起飞
    TASK_1_TAKEOFF,                            // 任务1: 起飞到指定高度
    TASK_2_TUNNELING,                          // 任务2: 隧道穿越
    TASK_3_OBSTACLE_AVOIDANCE,                 // 任务3: 障碍物躲避
    TASK_4_AREA_FLYTHROUGH,                    // 任务4: 搜索区域飞跃
    TASK_5_AREA_FLYTHROUGH,                    // 任务5: 投放区域飞跃
    TASK_6_LANDING_COMM,                       // 任务6: 降落和通信
    TASK_COMPLETED,                            // 任务完成
    TASK_ERROR                                 // 任务错误
} task_state_t;

// ========================= 数据结构定义 =========================
// 位置坐标结构体
typedef struct {
    float x;                                   // X坐标(米)
    float y;                                   // Y坐标(米)
    float z;                                   // Z坐标(米)
    float yaw;                                 // 偏航角(度)
} position_t;

// 传感器数据结构体
typedef struct {
    float laser_distance;                      // 激光测距数据(厘米)
    float altitude;                            // 高度数据(米)
    uint64_t timestamp;                        // 时间戳(微秒)
} sensor_data_t;

// 系统状态结构体
typedef struct {
    system_state_t system_state;               // 系统状态
    task_state_t task_state;                   // 当前任务状态
    position_t current_position;               // 当前位置
    position_t target_position;                // 目标位置
    sensor_data_t sensor_data;                 // 传感器数据
    bool armed;
    bool offboard;
    bool position_received;
    uint32_t last_heartbeat_ms;
    uint32_t last_position_ms;
    bool landing_confirmed;
    uint32_t landing_confirmed_ms;
    time_t landing_time;
    bool mavlink_connected;                    // MAVLINK连接状态
    bool auto_takeoff_triggered;               // 自动起飞触发标志
    uint32_t task_start_time;                  // 任务开始时间
    uint32_t system_start_time;                // 系统启动时间
    int current_waypoint_index;                // 当前航点索引
} acm_state_t;

// ========================= 预设航点定义 =========================
// 航点索引枚举
typedef enum {
    WP_START = 0,                              // 起始点
    WP_TAKEOFF_TARGET,                         // 起飞目标高度
    WP_TUNNEL_1_ENTRY,                         // 隧道1入口
    WP_TUNNEL_1_EXIT,                          // 隧道1出口
    WP_TUNNEL_2_ENTRY,                         // 隧道2入口
    WP_TUNNEL_2_EXIT,                          // 隧道2出口
    WP_TUNNEL_3_ENTRY,                         // 隧道3入口
    WP_TUNNEL_3_EXIT,                          // 隧道3出口(转弯点)
    WP_OBSTACLE_ENTRY,                         // 障碍物区域入口
    WP_OBSTACLE_EXIT,                          // 障碍物区域出口
    WP_SEARCH_AREA_FLYOVER,                    // 搜索区域上空飞跃点
    WP_DROPOFF_AREA_FLYOVER,                   // 投放区域上空飞跃点
    WP_LANDING_POINT,                          // 最终降落点
    WP_MAX_COUNT                               // 航点总数
} waypoint_index_t;

// ========================= 全局变量声明 =========================
extern acm_state_t g_acm_state;               // 全局系统状态
extern QueueHandle_t uart_queue;              // UART队列句柄
extern QueueHandle_t hc12_queue;              // HC12队列句柄
extern SemaphoreHandle_t state_mutex;         // 状态互斥锁
extern esp_timer_handle_t heartbeat_timer;    // 心跳计时器
extern const position_t preset_waypoints[];   // 预设航点数组

// ========================= 函数声明 =========================

// 系统初始化函数
void acm_system_init(void);                   // ACM系统初始化
void uart_init(void);                         // UART初始化
void hc12_init(void);                         // HC12通信模块初始化
void i2c_init(void);                          // I2C总线初始化
void sensor_init(void);                       // 传感器初始化
void mavlink_init(void);                      // MAVLINK协议初始化

// 主要任务函数
void main_task(void *pvParameters);           // 主控制任务
void mavlink_task(void *pvParameters);        // MAVLINK通信任务
void sensor_task(void *pvParameters);         // 传感器数据采集任务
void hc12_task(void *pvParameters);           // HC12通信任务

// 系统控制函数
void heartbeat_callback(void* arg);           // 心跳回调函数
void auto_takeoff_check(void);                // 自动起飞检查
void update_task_state(void);                 // 更新任务状态
void execute_current_task(void);              // 执行当前任务
void handle_task_transition(void);            // 处理任务转换

// MAVLINK通信函数
void send_mavlink_command(mavlink_message_t *msg);           // 发送MAVLINK指令
void process_mavlink_message(mavlink_message_t *msg);        // 处理MAVLINK消息
void send_position_command(position_t *target);              // 发送位置控制指令
void send_takeoff_command(float altitude);                   // 发送起飞指令
void send_land_command(void);                                // 发送降落指令
void send_arm_command(bool arm);                             // 发送解锁/上锁指令
void send_set_mode_command(uint8_t mode);                    // 发送飞行模式设置指令

// 传感器数据处理函数
float read_vl53l0x_distance(void);            // 读取VL53L0X激光测距数据
void process_sensor_data(void);               // 处理传感器数据

// HC12通信函数
void hc12_send_message(const char* message);  // 通过HC12发送消息
void send_mission_complete_message(void);     // 发送任务完成消息

// 工具函数
float calculate_distance_to_target(position_t *current, position_t *target);  // 计算到目标的距离
bool is_position_reached(position_t *current, position_t *target);           // 检查是否到达目标位置
uint32_t get_current_time_ms(void);           // 获取当前时间(毫秒)
void log_system_status(void);                 // 记录系统状态

// 安全函数
void emergency_stop(void);                    // 紧急停止
void system_error_handler(const char* error_msg);  // 系统错误处理

#endif // ACM_CORE_H