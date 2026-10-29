#!/bin/bash
# ESP-IDF无人机项目标准设置脚本

# =====================================================================
# 第一步：创建标准ESP-IDF项目结构
# =====================================================================

# 创建项目目录
cd ~/Desktop
mkdir -p drone_esp32_project
cd drone_esp32_project

# 创建标准目录结构
mkdir -p main
mkdir -p components/mavlink_lib/include

# =====================================================================
# 第二步：创建项目根配置文件
# =====================================================================

# 创建 CMakeLists.txt (项目根目录)
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_acm_system)
EOF

# =====================================================================
# 第三步：创建main组件配置
# =====================================================================

# 创建 main/CMakeLists.txt
cat > main/CMakeLists.txt << 'EOF'
idf_component_register(
    SRCS "acm_core.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver 
        esp_timer 
        nvs_flash
        freertos
        esp_common
        mavlink_lib
    EMBED_FILES
)
EOF

# =====================================================================
# 第四步：创建简化的主程序入口
# =====================================================================

# 创建 main/main.c  
cat > main/main.c << 'EOF'
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
EOF

# =====================================================================
# 第五步：创建MAVLINK组件配置
# =====================================================================

# 创建 components/mavlink_lib/CMakeLists.txt
cat > components/mavlink_lib/CMakeLists.txt << 'EOF'
idf_component_register(
    INCLUDE_DIRS 
        "include"
        "include/common"
    REQUIRES ""
)
EOF

# =====================================================================
# 第六步：下载MAVLINK头文件
# =====================================================================

echo "正在下载MAVLINK头文件..."
cd components/mavlink_lib

# 下载预编译的MAVLINK C库
curl -L -o mavlink.zip https://github.com/mavlink/c_library_v2/archive/refs/heads/master.zip
unzip mavlink.zip
cp -r c_library_v2-master/* include/
rm -rf c_library_v2-master mavlink.zip

echo "MAVLINK头文件下载完成"

# =====================================================================
# 第七步：复制修改过的代码文件
# =====================================================================

cd ../../main

# 注意：需要手动复制你的 acm_core.h 和 acm_core.c 文件到这里
echo "请将你的 acm_core.h 和 acm_core.c 文件复制到 main/ 目录"
echo "项目结构应该是："
echo "drone_esp32_project/"
echo "├── CMakeLists.txt"
echo "├── main/"
echo "│   ├── CMakeLists.txt"
echo "│   ├── main.c"
echo "│   ├── acm_core.h      # 你需要复制这个文件"
echo "│   └── acm_core.c      # 你需要复制这个文件"
echo "└── components/"
echo "    └── mavlink_lib/"
echo "        ├── CMakeLists.txt"
echo "        └── include/"
echo "            └── common/"
echo "                └── mavlink.h"

# =====================================================================
# 第八步：创建编译脚本
# =====================================================================

cd ..
cat > build.sh << 'EOF'
#!/bin/bash
# ESP-IDF编译脚本

echo "设置ESP-IDF环境..."
source ~/esp/esp-idf/export.sh

echo "设置目标芯片为ESP32-S3..."
idf.py set-target esp32s3

echo "开始编译..."
idf.py build

echo "编译完成！"
echo "要烧录到设备，请运行: idf.py flash monitor"
EOF

chmod +x build.sh

echo ""
echo "✅ ESP-IDF项目设置完成！"
echo ""
echo "📋 接下来的步骤："
echo "1. 将你的 acm_core.h 和 acm_core.c 复制到 main/ 目录"
echo "2. 运行: ./build.sh"
echo "3. 连接ESP32-S3开发板"
echo "4. 运行: idf.py flash monitor"
echo ""