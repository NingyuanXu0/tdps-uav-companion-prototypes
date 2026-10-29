# ---------------------------------------------
#  hc12_task6_test.py
# ---------------------------------------------
"""
Standalone functional test for Task-6 HC12 wireless communication.

1. Initialise HC12 (channel/baud taken from config.py)
2. Simulate post-landing wait (15 s)
3. Transmit mission-complete message with timestamp
4. Optionally listen 5 s for an echo / acknowledgement
"""

import time
from communication import HC12Interface           # HC12 驱动封装  :contentReference[oaicite:0]{index=0}
from utils import log, format_mission_complete_message  # 日志与消息格式化  :contentReference[oaicite:1]{index=1}

POST_LANDING_DELAY = 15          # 秒
LISTEN_TIMEOUT_MS   = 5000       # 监听回执时间

def main() -> None:
    log("=== HC12 Task-6 communication self-test ===")

    # 1. 初始化 HC12
    hc12 = HC12Interface()
    if not hc12.initialized:
        log("HC12 init failed – aborting test", "ERROR")
        return

    # 2. 等待 Task-6 规定的 15 s 冷却时间
    log(f"Simulating post-landing wait {POST_LANDING_DELAY}s")
    time.sleep(POST_LANDING_DELAY)

    # 3. 生成并发送 Mission Complete 消息
    message = format_mission_complete_message()
    if hc12.send_message(message):
        log("Tx OK ✅")
    else:
        log("Tx failed ❌", "ERROR")
        return

    # 4. （可选）监听回环 / 回执
    log(f"Listening {LISTEN_TIMEOUT_MS/1000}s for echo/ack …")
    reply = hc12.receive_message(timeout_ms=LISTEN_TIMEOUT_MS)
    if reply:
        log(f"Rx → «{reply}»")
    else:
        log("No reply (timeout)")

    log("=== HC12 self-test end ===")

if __name__ == "__main__":
    main()
