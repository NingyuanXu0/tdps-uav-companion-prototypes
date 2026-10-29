import machine
import time
import struct
from config import UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE, CONNECTION_TIMEOUT
from utils import log
from mavlink import (
    build_command_long, build_set_mode, build_set_position_target_local_ned,
    build_heartbeat, build_motor_test_command,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_SERVO,
    MAV_MODE_STABILIZE_ARMED, MAV_MODE_STABILIZE_DISARMED,
    parse_mavlink_message, MAVLINK_MSG_ID_HEARTBEAT,
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_MODE_FLAG_SAFETY_ARMED
)

class FCMInterface:
    def __init__(self):
        try:
            self.uart = machine.UART(1,
                                     baudrate=UART_BAUDRATE,
                                     tx=machine.Pin(UART_TX_PIN),
                                     rx=machine.Pin(UART_RX_PIN))
            self.initialized = True
            log("Pixhawk UART interface initialized successfully")
            self.armed = False
            self.flight_mode = None
            self.last_heartbeat = None
            self._rx_buffer = bytearray()
            self._seq = 0
        except Exception as e:
            self.initialized = False
            log(f"Pixhawk UART initialization failed: {e}", "ERROR")

    def update(self):
        try:
            if self.uart.any():
                data = self.uart.read(256)
                if data:
                    self._process_mavlink_data(data)
        except Exception as e:
            log(f"UART receive failed: {e}", "WARNING")

    def _process_mavlink_data(self, data):
        messages = parse_mavlink_message(data, self._rx_buffer)
        for msg in messages:
            if msg['msg_id'] == MAVLINK_MSG_ID_HEARTBEAT and msg['system_id'] == 1 and msg['component_id'] == 1:
                self.last_heartbeat = time.ticks_ms()
                custom_mode, _, _, base_mode, _, _ = struct.unpack('<IBBBBB', msg['payload'])
                self.armed = bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                self.flight_mode = custom_mode
                log(f"Heartbeat received - System: {msg['system_id']}, Component: {msg['component_id']}")

    def _send_mavlink(self, frame):
        if not self.initialized:
            return False
        written = self.uart.write(frame)
        if written != len(frame):
            return False
        self._seq = (self._seq + 1) & 255
        return True

    def send_heartbeat(self):
        return self._send_mavlink(build_heartbeat(self._seq))

    def set_mode_by_name(self, mode_name):
        PX4_MODES = {
            "MANUAL": 1,
            "ALTCTL": 2,
            "POSCTL": 3,
            "AUTO": 4,
            "ACRO": 5,
            "OFFBOARD": 6,
            "STABILIZED": 7
        }

        if mode_name not in PX4_MODES:
            log(f"Unknown flight mode: {mode_name}", "ERROR")
            return False

        custom_mode = PX4_MODES[mode_name] << 16
        base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

        frame = build_set_mode(
            seq=self._seq,
            base_mode=base_mode,
            custom_mode=custom_mode
        )
        log(f"Setting PX4 flight mode to {mode_name}")
        self._send_mavlink(frame)
        return True

    def arm_motors(self):
        return self._send_mavlink(build_command_long(
            self._seq, MAV_CMD_COMPONENT_ARM_DISARM, param1=1.0))

    def disarm_motors(self):
        return self._send_mavlink(build_command_long(
            self._seq, MAV_CMD_COMPONENT_ARM_DISARM, param1=0.0))

    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        frame = build_set_position_target_local_ned(
            seq=self._seq,
            vx=vx,
            vy=vy,
            vz=vz,
            yaw_rate=yaw_rate
        )
        self._send_mavlink(frame)
        log(f"Velocity command sent: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}")
    def is_connected(self):
        return (self.initialized and self.last_heartbeat is not None and
                time.ticks_diff(time.ticks_ms(), self.last_heartbeat) < CONNECTION_TIMEOUT * 1000)

    def test_motor_individual(self, motor_num, throttle, duration):
        if not self.is_connected() or self.armed:
            return False
        if not 1 <= motor_num <= 4 or not 0 <= throttle <= 100 or duration < 0:
            return False
        return self._send_mavlink(build_motor_test_command(
            self._seq, motor_num, 0, throttle, duration, motor_count=1))

    def test_all_motors(self, throttle, duration):
        sent = True
        for motor_num in range(1, 5):
            sent = self.test_motor_individual(motor_num, throttle, duration) and sent
        return sent

    def stop_motor_test(self):
        return self.test_all_motors(0, 0)

    def emergency_stop(self):
        self.stop_motor_test()
        return self.disarm_motors()
