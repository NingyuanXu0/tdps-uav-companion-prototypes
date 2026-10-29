#!/bin/bash
# ESP-IDF 编译入口；由使用者在配置硬件和工具链后手动运行。
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if ! command -v idf.py >/dev/null 2>&1; then
    if [[ -z "${IDF_PATH:-}" || ! -f "${IDF_PATH}/export.sh" ]]; then
        echo "请先导出 ESP-IDF 环境，或设置 IDF_PATH。" >&2
        exit 1
    fi
    source "${IDF_PATH}/export.sh"
fi

if [[ -n "${TDPS_IDF_TARGET:-}" ]]; then
    idf.py set-target "${TDPS_IDF_TARGET}"
elif [[ ! -f sdkconfig ]]; then
    idf.py set-target esp32s3
fi
idf.py build
