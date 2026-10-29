#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// 引入我们的ACM核心模块
extern void app_main(void);

// ESP-IDF标准入口（如果需要的话）
void app_main_wrapper(void) {
    ESP_LOGI("MAIN", "无人机ACM系统启动中...");
    
    // 调用我们的主程序
    app_main();
}
