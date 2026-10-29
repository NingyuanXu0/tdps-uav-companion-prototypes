#include "acm_core.h"

/* PX4 伴随控制器原型。MAVLink 使用随仓库保留的生成头文件。
 * 本发布未执行构建、测试或硬件验证；传感器读取仍为示例实现。
 */

// ========================= 全局变量定义 =========================
acm_state_t g_acm_state;                      // 全局系统状态变量
QueueHandle_t uart_queue;                     // UART消息队列
QueueHandle_t hc12_queue;                     // HC12消息队列
SemaphoreHandle_t state_mutex;                // 状态保护互斥锁
static bool i2c_ready = false;
esp_timer_handle_t heartbeat_timer;           // 心跳定时器句柄

// ========================= 预设航点数组 =========================
const position_t preset_waypoints[WP_MAX_COUNT] = {
    // WP_START: 起始点
    {0.0, 0.0, 0.0, 0.0},
    // WP_TAKEOFF_TARGET: 起飞目标高度（天花板灯高度）
    {0.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_1_ENTRY: 隧道1入口
    {2.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_1_EXIT: 隧道1出口
    {4.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_2_ENTRY: 隧道2入口
    {6.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_2_EXIT: 隧道2出口
    {8.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_3_ENTRY: 隧道3入口
    {10.0, 0.0, 3.0, 0.0},
    // WP_TUNNEL_3_EXIT: 隧道3出口（转弯点）
    {12.0, 0.0, 3.0, 90.0},
    // WP_OBSTACLE_ENTRY: 障碍物区域入口（降低高度）
    {12.0, 2.0, 2.8, 90.0},
    // WP_OBSTACLE_EXIT: 障碍物区域出口
    {12.0, 6.0, 2.8, 90.0},
    // WP_SEARCH_AREA_FLYOVER: 搜索区域上空飞跃点
    {10.0, 8.0, 2.5, 90.0},
    // WP_DROPOFF_AREA_FLYOVER: 投放区域上空飞跃点
    {6.0, 8.0, 2.5, 90.0},
    // WP_LANDING_POINT: 最终降落点
    {6.0, 8.0, 0.0, 90.0}
};

// ========================= 主程序入口 =========================
void app_main(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "       ACM无人机自动控制系统启动        ");
    ESP_LOGI(TAG, "========================================");
    
    // 系统初始化
    acm_system_init();
    
    // 创建主要任务
    ESP_LOGI(TAG, "创建系统任务...");
    xTaskCreate(main_task, "main_task", 4096, NULL, 10, NULL);
    xTaskCreate(mavlink_task, "mavlink_task", 4096, NULL, 9, NULL);
    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 8, NULL);
    xTaskCreate(hc12_task, "hc12_task", 2048, NULL, 7, NULL);
    
    ESP_LOGI(TAG, "ACM系统启动完成，等待自动起飞...");
}

// ========================= 系统初始化函数 =========================
void acm_system_init(void) {
    ESP_LOGI(TAG, "开始系统初始化...");
    
    // 初始化NVS存储
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化各个模块
    uart_init();
    hc12_init();
    i2c_init();
    sensor_init();
    mavlink_init();
    
    // 创建同步对象
    state_mutex = xSemaphoreCreateMutex();
    hc12_queue = xQueueCreate(10, sizeof(char[128]));
    
    // 初始化系统状态
    memset(&g_acm_state, 0, sizeof(acm_state_t));
    g_acm_state.system_state = SYSTEM_INIT;
    g_acm_state.task_state = TASK_IDLE;
    g_acm_state.system_start_time = get_current_time_ms();
    g_acm_state.auto_takeoff_triggered = false;
    g_acm_state.current_waypoint_index = 0;
    
    // 创建心跳定时器
    const esp_timer_create_args_t heartbeat_timer_args = {
        .callback = &heartbeat_callback,
        .name = "heartbeat_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&heartbeat_timer_args, &heartbeat_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer, HEARTBEAT_INTERVAL));
    
    ESP_LOGI(TAG, "系统初始化完成");
}

// UART初始化（与飞控通信）
void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TXD_PIN, UART_RXD_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART初始化完成（飞控通信）");
}

// HC12通信模块初始化
void hc12_init(void) {
    uart_config_t hc12_config = {
        .baud_rate = HC12_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(HC12_UART_PORT, BUF_SIZE, BUF_SIZE, 10, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(HC12_UART_PORT, &hc12_config));
    ESP_ERROR_CHECK(uart_set_pin(HC12_UART_PORT, HC12_TXD_PIN, HC12_RXD_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "HC12通信模块初始化完成");
}

// I2C总线初始化
void i2c_init(void) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(I2C_SDA_PIN) || !GPIO_IS_VALID_OUTPUT_GPIO(I2C_SCL_PIN)) {
        ESP_LOGW(TAG, "I2C 示例已禁用：请在 menuconfig 中配置实际板卡的 SDA/SCL 引脚");
        return;
    }
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    
    i2c_ready = true;
    ESP_LOGI(TAG, "I2C总线初始化完成");
}

// 传感器初始化
void sensor_init(void) {
    ESP_LOGW(TAG, "VL53L0X 读取为示例，完整驱动和校准尚未集成");
}

// MAVLINK协议初始化
void mavlink_init(void) {
    ESP_LOGI(TAG, "MAVLINK协议初始化完成");
}

// ========================= 主要任务函数 =========================
void main_task(void *pvParameters) {
    ESP_LOGI(TAG, "主控制任务开始运行");
    
    while (1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        
        uint32_t now_ms = get_current_time_ms();
        if (g_acm_state.mavlink_connected &&
            now_ms - g_acm_state.last_heartbeat_ms > 3000) {
            g_acm_state.mavlink_connected = false;
            if (g_acm_state.auto_takeoff_triggered &&
                g_acm_state.task_state != TASK_COMPLETED &&
                g_acm_state.task_state != TASK_ERROR) {
                system_error_handler("飞控心跳超时");
            }
        }
        // 检查自动起飞条件
        auto_takeoff_check();
        
        // 更新任务状态
        update_task_state();
        
        // 执行当前任务
        execute_current_task();
        
        // 记录系统状态（调试用）
        log_system_status();
        
        xSemaphoreGive(state_mutex);
        
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_FREQ));
    }
}

void mavlink_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t status;
    
    ESP_LOGI(TAG, "MAVLINK通信任务开始运行");
    
    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                    process_mavlink_message(&msg);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "传感器数据采集任务开始运行");
    
    while (1) {
        // I2C 读取不占用状态锁，允许接收任务及时更新飞控状态。
        float laser_distance = read_vl53l0x_distance();
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        g_acm_state.sensor_data.laser_distance = laser_distance;
        g_acm_state.sensor_data.timestamp = esp_timer_get_time();
        
        // 处理传感器数据
        process_sensor_data();
        
        xSemaphoreGive(state_mutex);
        
        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_FREQ));
    }
}

void hc12_task(void *pvParameters) {
    char message[128];
    
    ESP_LOGI(TAG, "HC12通信任务开始运行");
    
    while (1) {
        // 检查是否有待发送的消息
        if (xQueueReceive(hc12_queue, message, pdMS_TO_TICKS(100))) {
            hc12_send_message(message);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========================= 系统控制函数 =========================
void heartbeat_callback(void* arg) {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    
    // 标准MAVLINK心跳参数
    heartbeat.type = MAV_TYPE_ONBOARD_CONTROLLER;                    // 伴随控制器
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;            // 非飞控组件
    heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // 启用自定义模式
    heartbeat.custom_mode = 0;                              // 自定义模式（0为默认）
    heartbeat.system_status = MAV_STATE_ACTIVE;             // 系统状态：活跃
    heartbeat.mavlink_version = 3;                          // MAVLINK版本
    
    mavlink_msg_heartbeat_encode(1, 200, &msg, &heartbeat);
    send_mavlink_command(&msg);
}

void auto_takeoff_check(void) {
    uint32_t current_time = get_current_time_ms();
    
    // 检查是否满足自动起飞条件
    if (!g_acm_state.auto_takeoff_triggered && 
        g_acm_state.system_state == SYSTEM_INIT &&
        g_acm_state.mavlink_connected && g_acm_state.position_received &&
        current_time - g_acm_state.last_position_ms < 1000 &&
        (current_time - g_acm_state.system_start_time) > AUTO_TAKEOFF_DELAY) {
        
        ESP_LOGI(TAG, "触发自动起飞序列");
        g_acm_state.auto_takeoff_triggered = true;
        g_acm_state.task_state = TASK_AUTO_TAKEOFF;
        g_acm_state.system_state = SYSTEM_READY;
        g_acm_state.task_start_time = current_time;
        g_acm_state.target_position = g_acm_state.current_position;
    }
}

void update_task_state(void) {
    uint32_t current_time = get_current_time_ms();
    
    switch (g_acm_state.task_state) {
        case TASK_AUTO_TAKEOFF:
            if (g_acm_state.mavlink_connected && g_acm_state.offboard && g_acm_state.armed &&
                current_time - g_acm_state.task_start_time >= 1200) {
                g_acm_state.task_state = TASK_1_TAKEOFF;
                g_acm_state.task_start_time = current_time;
                g_acm_state.current_waypoint_index = WP_TAKEOFF_TARGET;
                g_acm_state.system_state = SYSTEM_MISSION;
                ESP_LOGI(TAG, "开始执行任务1: 起飞到指定高度");
            }
            break;
            
        case TASK_1_TAKEOFF:
            if (is_position_reached(&g_acm_state.current_position, 
                                  (position_t*)&preset_waypoints[WP_TAKEOFF_TARGET])) {
                g_acm_state.task_state = TASK_2_TUNNELING;
                g_acm_state.task_start_time = current_time;
                g_acm_state.current_waypoint_index = WP_TUNNEL_1_ENTRY;
                ESP_LOGI(TAG, "任务1完成，开始执行任务2: 隧道穿越");
            }
            break;
            
        case TASK_2_TUNNELING:
            if (g_acm_state.current_waypoint_index >= WP_TUNNEL_3_EXIT) {
                if (is_position_reached(&g_acm_state.current_position, 
                                      (position_t*)&preset_waypoints[WP_TUNNEL_3_EXIT])) {
                    g_acm_state.task_state = TASK_3_OBSTACLE_AVOIDANCE;
                    g_acm_state.task_start_time = current_time;
                    g_acm_state.current_waypoint_index = WP_OBSTACLE_ENTRY;
                    ESP_LOGI(TAG, "任务2完成，开始执行任务3: 障碍物躲避");
                }
            }
            break;
            
        case TASK_3_OBSTACLE_AVOIDANCE:
            if (is_position_reached(&g_acm_state.current_position, 
                                  (position_t*)&preset_waypoints[WP_OBSTACLE_EXIT])) {
                g_acm_state.task_state = TASK_4_AREA_FLYTHROUGH;
                g_acm_state.task_start_time = current_time;
                g_acm_state.current_waypoint_index = WP_SEARCH_AREA_FLYOVER;
                ESP_LOGI(TAG, "任务3完成，开始执行任务4: 搜索区域飞跃");
            }
            break;
            
        case TASK_4_AREA_FLYTHROUGH:
            if (is_position_reached(&g_acm_state.current_position, 
                                  (position_t*)&preset_waypoints[WP_SEARCH_AREA_FLYOVER])) {
                g_acm_state.task_state = TASK_5_AREA_FLYTHROUGH;
                g_acm_state.task_start_time = current_time;
                g_acm_state.current_waypoint_index = WP_DROPOFF_AREA_FLYOVER;
                ESP_LOGI(TAG, "任务4完成，开始执行任务5: 投放区域飞跃");
            }
            break;
            
        case TASK_5_AREA_FLYTHROUGH:
            if (is_position_reached(&g_acm_state.current_position, 
                                  (position_t*)&preset_waypoints[WP_DROPOFF_AREA_FLYOVER])) {
                g_acm_state.task_state = TASK_6_LANDING_COMM;
                g_acm_state.task_start_time = current_time;
                g_acm_state.current_waypoint_index = WP_LANDING_POINT;
                ESP_LOGI(TAG, "任务5完成，开始执行任务6: 降落和通信");
            }
            break;
            
        case TASK_6_LANDING_COMM:
            if (g_acm_state.position_received && current_time - g_acm_state.last_position_ms < 1000 &&
                g_acm_state.current_position.z < ALTITUDE_TOLERANCE) {
                if (!g_acm_state.landing_confirmed) {
                    g_acm_state.landing_confirmed = true;
                    g_acm_state.landing_confirmed_ms = current_time;
                    g_acm_state.landing_time = time(NULL);
                }
                if (current_time - g_acm_state.landing_confirmed_ms >= 15000) {
                    send_mission_complete_message();
                    g_acm_state.task_state = TASK_COMPLETED;
                    g_acm_state.system_state = SYSTEM_LANDED;
                    ESP_LOGI(TAG, "任务6完成，所有任务执行完毕");
                }
            } else {
                g_acm_state.landing_confirmed = false;
            }
            break;

        case TASK_IDLE:
        case TASK_ERROR:
        case TASK_COMPLETED:
            // 等待、错误或任务完成状态
            break;
            
        default:
            ESP_LOGW(TAG, "未知任务状态: %d", g_acm_state.task_state);
            break;
    }
}

void execute_current_task(void) {
    switch (g_acm_state.task_state) {
        case TASK_AUTO_TAKEOFF: {
            static uint32_t last_command_ms = 0;
            uint32_t now_ms = get_current_time_ms();
            // OFFBOARD 需要先收到持续的设定点；模式和解锁以心跳反馈为准。
            send_position_command(&g_acm_state.target_position);
            if (now_ms - g_acm_state.task_start_time >= 1200 && now_ms - last_command_ms >= 1000) {
                if (!g_acm_state.offboard) {
                    send_set_mode_command(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
                } else if (!g_acm_state.armed) {
                    send_arm_command(true);
                }
                last_command_ms = now_ms;
            }
            break;
        }
        case TASK_1_TAKEOFF:
            // 持续发送本地起飞目标，保持 OFFBOARD 设定点流。
            send_position_command((position_t*)&preset_waypoints[WP_TAKEOFF_TARGET]);
            break;

        case TASK_2_TUNNELING:
            // 按顺序穿越隧道
            if (g_acm_state.current_waypoint_index <= WP_TUNNEL_3_EXIT) {
                send_position_command((position_t*)&preset_waypoints[g_acm_state.current_waypoint_index]);
                
                // 检查是否到达当前航点
                if (is_position_reached(&g_acm_state.current_position, 
                                      (position_t*)&preset_waypoints[g_acm_state.current_waypoint_index])) {
                    g_acm_state.current_waypoint_index++;
                    ESP_LOGI(TAG, "到达航点 %d，前往下一个航点", g_acm_state.current_waypoint_index - 1);
                }
            }
            break;
            
        case TASK_3_OBSTACLE_AVOIDANCE:
            // 降低高度通过障碍物
            if (g_acm_state.current_waypoint_index == WP_OBSTACLE_ENTRY) {
                send_position_command((position_t*)&preset_waypoints[WP_OBSTACLE_ENTRY]);
                if (is_position_reached(&g_acm_state.current_position, 
                                      (position_t*)&preset_waypoints[WP_OBSTACLE_ENTRY])) {
                    g_acm_state.current_waypoint_index = WP_OBSTACLE_EXIT;
                }
            } else {
                send_position_command((position_t*)&preset_waypoints[WP_OBSTACLE_EXIT]);
            }
            break;
            
        case TASK_4_AREA_FLYTHROUGH:
            // 飞跃搜索区域上空
            send_position_command((position_t*)&preset_waypoints[WP_SEARCH_AREA_FLYOVER]);
            break;
            
        case TASK_5_AREA_FLYTHROUGH:
            // 飞跃投放区域上空
            send_position_command((position_t*)&preset_waypoints[WP_DROPOFF_AREA_FLYOVER]);
            break;
            
        case TASK_6_LANDING_COMM: {
            static uint32_t last_land_command_ms = 0;
            uint32_t now_ms = get_current_time_ms();
            send_position_command((position_t*)&preset_waypoints[WP_LANDING_POINT]);
            if (now_ms - g_acm_state.task_start_time >= 2000 && now_ms - last_land_command_ms >= 1000) {
                send_land_command();
                last_land_command_ms = now_ms;
            }
            break;
        }

        default:
            break;
    }
}

// ========================= MAVLINK通信函数 =========================
void send_mavlink_command(mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(UART_PORT_NUM, (const char*)buf, len);
}

void process_mavlink_message(mavlink_message_t *msg) {
    if (msg->sysid != 1 || msg->compid != 1) return;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(msg, &heartbeat);
            g_acm_state.last_heartbeat_ms = get_current_time_ms();
            g_acm_state.armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            g_acm_state.offboard = ((heartbeat.custom_mode >> 16) & 0xff) == 6;
            g_acm_state.mavlink_connected = true;
            ESP_LOGD(TAG, "收到飞控心跳包");
            break;
        }
        
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t pos;
            mavlink_msg_local_position_ned_decode(msg, &pos);
            
            g_acm_state.position_received = true;
            g_acm_state.last_position_ms = get_current_time_ms();
            g_acm_state.current_position.x = pos.x;
            g_acm_state.current_position.y = pos.y;
            g_acm_state.current_position.z = -pos.z; // NED坐标系转换
            
            ESP_LOGD(TAG, "当前位置: (%.2f, %.2f, %.2f)", 
                    g_acm_state.current_position.x,
                    g_acm_state.current_position.y,
                    g_acm_state.current_position.z);
            break;
        }
        
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(msg, &attitude);
            g_acm_state.current_position.yaw = attitude.yaw * 180.0f / M_PI;
            break;
        }

        case MAVLINK_MSG_ID_COMMAND_ACK: {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(msg, &ack);
            ESP_LOGI(TAG, "指令确认: %d, 结果: %d", ack.command, ack.result);
            break;
        }
        
        default:
            ESP_LOGD(TAG, "收到MAVLINK消息: %d", msg->msgid);
            break;
    }
    xSemaphoreGive(state_mutex);
}

void send_position_command(position_t *target) {
    mavlink_message_t msg;
    mavlink_set_position_target_local_ned_t cmd;
    
    cmd.time_boot_ms = esp_timer_get_time() / 1000;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.coordinate_frame = MAV_FRAME_LOCAL_NED;
    // 正确的type_mask: 仅控制位置和偏航角，忽略速度和加速度
    // 位掩码 0x09F8：位置和偏航有效
    cmd.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | 
                    POSITION_TARGET_TYPEMASK_VY_IGNORE | 
                    POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE | 
                    POSITION_TARGET_TYPEMASK_AY_IGNORE | 
                    POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    cmd.x = target->x;
    cmd.y = target->y;
    cmd.z = -target->z; // NED坐标系转换（Z轴向下为正）
    cmd.vx = 0.0;
    cmd.vy = 0.0;
    cmd.vz = 0.0;
    cmd.afx = 0.0;
    cmd.afy = 0.0;
    cmd.afz = 0.0;
    cmd.yaw = target->yaw * M_PI / 180.0; // 转换为弧度
    cmd.yaw_rate = 0.0;
    
    mavlink_msg_set_position_target_local_ned_encode(1, 200, &msg, &cmd);
    send_mavlink_command(&msg);
    
    ESP_LOGD(TAG, "发送位置指令: (%.2f, %.2f, %.2f, %.1f°)", 
             target->x, target->y, target->z, target->yaw);
}

void send_takeoff_command(float altitude) {
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    
    // 标准MAVLINK起飞命令参数
    cmd.target_system = 1;                      // 目标系统ID
    cmd.target_component = 1;                   // 目标组件ID
    cmd.command = MAV_CMD_NAV_TAKEOFF;          // 起飞命令
    cmd.confirmation = 0;                       // 确认计数器
    cmd.param1 = -1.0;                          // 最小俯仰角（-1表示使用默认值）
    cmd.param2 = 0.0;                           // 空参数
    cmd.param3 = 0.0;                           // 起飞速度（0表示使用默认值）
    cmd.param4 = NAN;                           // 偏航角（NAN表示使用当前偏航角）
    cmd.param5 = NAN;                           // 纬度（本地坐标系使用NAN）
    cmd.param6 = NAN;                           // 经度（本地坐标系使用NAN）
    cmd.param7 = altitude;                      // 起飞高度（米）
    
    mavlink_msg_command_long_encode(1, 200, &msg, &cmd);
    send_mavlink_command(&msg);
    
    ESP_LOGI(TAG, "发送起飞指令: 高度 %.2f米", altitude);
}

void send_land_command(void) {
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    
    // 标准MAVLINK降落命令参数
    cmd.target_system = 1;                      // 目标系统ID
    cmd.target_component = 1;                   // 目标组件ID
    cmd.command = MAV_CMD_NAV_LAND;             // 降落命令
    cmd.confirmation = 0;                       // 确认计数器
    cmd.param1 = 0.0;                           // 中止高度（0表示降落到地面）
    cmd.param2 = 0.0;                           // 降落模式（0表示正常降落）
    cmd.param3 = 0.0;                           // 接近半径（0表示使用默认值）
    cmd.param4 = NAN;                           // 偏航角（NAN表示使用当前偏航角）
    cmd.param5 = NAN;                           // 纬度（本地坐标系使用NAN）
    cmd.param6 = NAN;                           // 经度（本地坐标系使用NAN）
    cmd.param7 = 0.0;                           // 高度（0表示地面）
    
    mavlink_msg_command_long_encode(1, 200, &msg, &cmd);
    send_mavlink_command(&msg);
    
    ESP_LOGI(TAG, "发送降落指令");
}

void send_arm_command(bool arm) {
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    
    // 标准MAVLINK解锁/上锁命令参数
    cmd.target_system = 1;                      // 目标系统ID
    cmd.target_component = 1;                   // 目标组件ID
    cmd.command = MAV_CMD_COMPONENT_ARM_DISARM; // 解锁/上锁命令
    cmd.confirmation = 0;                       // 确认计数器
    cmd.param1 = arm ? 1.0 : 0.0;               // 1=解锁, 0=上锁
    cmd.param2 = 0.0;                           // 强制解锁标志（0=正常检查）
    cmd.param3 = 0.0;                           // 预留参数
    cmd.param4 = 0.0;                           // 预留参数
    cmd.param5 = 0.0;                           // 预留参数
    cmd.param6 = 0.0;                           // 预留参数
    cmd.param7 = 0.0;                           // 预留参数
    
    mavlink_msg_command_long_encode(1, 200, &msg, &cmd);
    send_mavlink_command(&msg);
    
    ESP_LOGI(TAG, "发送%s指令", arm ? "解锁" : "上锁");
}

void send_set_mode_command(uint8_t mode) {
    mavlink_message_t msg;
    mavlink_set_mode_t set_mode;
    
    // 标准MAVLINK设置模式命令参数
    set_mode.target_system = 1;                 // 目标系统ID
    set_mode.base_mode = mode;                  // 基础模式
    set_mode.custom_mode = (6U << 16);                   // PX4 OFFBOARD 主模式
    
    mavlink_msg_set_mode_encode(1, 200, &msg, &set_mode);
    send_mavlink_command(&msg);
    
    ESP_LOGI(TAG, "发送飞行模式设置指令: %d", mode);
}

// ========================= 传感器数据处理函数 =========================
float read_vl53l0x_distance(void) {
    if (!i2c_ready) return -1.0f;
    /* 
     * VL53L0X激光测距传感器驱动实现
     * 注意：这是一个简化的实现示例
     * 实际项目中需要集成完整的VL53L0X驱动库
     * 
     * 完整的VL53L0X驱动应包括：
     * 1. 传感器初始化和配置
     * 2. 校准程序
     * 3. 测距模式设置
     * 4. 数据读取和校验
     * 5. 错误处理
     * 
     * 推荐使用ST官方的VL53L0X API库
     */
    uint8_t range_data[2] = {0};
    float distance = 0.0;
    
    // 简化的I2C读取实现（仅为示例）
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x8E, true); // 寄存器地址（示例）
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, range_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // 转换原始数据为距离值（具体算法取决于传感器设置）
        distance = (float)((range_data[0] << 8) | range_data[1]) / 10.0;
        ESP_LOGD(TAG, "激光测距: %.1f cm", distance);
    } else {
        ESP_LOGW(TAG, "读取VL53L0X失败，错误代码: %s", esp_err_to_name(ret));
        distance = -1.0; // 错误标识
    }
    
    // 限制合理的距离范围
    if (distance > 400.0 || distance < 0.0) {
        distance = -1.0; // 保留无效读数，避免当成无障碍距离
    }
    
    return distance;
}

void process_sensor_data(void) {
    // 处理传感器数据，可用于障碍物检测等
    if (g_acm_state.sensor_data.laser_distance > 0 && 
        g_acm_state.sensor_data.laser_distance < 30.0) {
        ESP_LOGW(TAG, "检测到近距离障碍物: %.1f cm", g_acm_state.sensor_data.laser_distance);
    }
}

// ========================= HC12通信函数 =========================
void hc12_send_message(const char* message) {
    int len = strlen(message);
    int written = uart_write_bytes(HC12_UART_PORT, message, len);
    
    if (written == len) {
        ESP_LOGI(TAG, "HC12发送: %s", message);
    } else {
        ESP_LOGW(TAG, "HC12发送失败");
    }
}

void send_mission_complete_message(void) {
    char message[128];
    if (g_acm_state.landing_time >= 1577836800) {
        struct tm timeinfo;
        char timestamp[32];
        gmtime_r(&g_acm_state.landing_time, &timeinfo);
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
        snprintf(message, sizeof(message), "mission accomplished - landing time: %s\n", timestamp);
    } else {
        snprintf(message, sizeof(message),
                 "mission accomplished - landing clock unavailable; uptime_ms=%lu\n",
                 (unsigned long)g_acm_state.landing_confirmed_ms);
    }

    // 将消息放入HC12发送队列
    if (xQueueSend(hc12_queue, message, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "任务完成消息已加入发送队列");
    } else {
        ESP_LOGW(TAG, "任务完成消息发送队列已满");
    }
}

// ========================= 工具函数 =========================
float calculate_distance_to_target(position_t *current, position_t *target) {
    float dx = target->x - current->x;
    float dy = target->y - current->y;
    float dz = target->z - current->z;
    
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

bool is_position_reached(position_t *current, position_t *target) {
    float distance = calculate_distance_to_target(current, target);
    float altitude_diff = fabsf(target->z - current->z);
    
    return (distance < POSITION_TOLERANCE && altitude_diff < ALTITUDE_TOLERANCE);
}

uint32_t get_current_time_ms(void) {
    return esp_timer_get_time() / 1000;
}

void log_system_status(void) {
    static uint32_t last_log_time = 0;
    uint32_t current_time = get_current_time_ms();
    
    // 每5秒记录一次状态
    if (current_time - last_log_time > 5000) {
        ESP_LOGI(TAG, "系统状态 - 任务:%d, 位置:(%.2f,%.2f,%.2f), 激光:%.1fcm", 
                 g_acm_state.task_state,
                 g_acm_state.current_position.x,
                 g_acm_state.current_position.y,
                 g_acm_state.current_position.z,
                 g_acm_state.sensor_data.laser_distance);
        last_log_time = current_time;
    }
}

// ========================= 安全函数 =========================
void emergency_stop(void) {
    ESP_LOGE(TAG, "紧急停止触发！");
    send_land_command();
    g_acm_state.task_state = TASK_ERROR;
    g_acm_state.system_state = SYSTEM_ERROR;
}

void system_error_handler(const char* error_msg) {
    ESP_LOGE(TAG, "系统错误: %s", error_msg);
    emergency_stop();
}