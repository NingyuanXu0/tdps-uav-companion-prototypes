import struct

# MAVLink constants
MAVLINK_STX = 0xFE
MAVLINK_SYSID = 255     # ESP32 / ground control system ID
MAVLINK_COMPID = 0      # Component ID

# MAVLink command IDs
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_MOTOR_TEST = 209  # 添加电机测试指令
MAV_CMD_DO_SET_MODE = 176

# Flight modes for PX4
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7

# MAV_MODE flags
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_TEST_ENABLED = 2
MAV_MODE_FLAG_AUTO_ENABLED = 4
MAV_MODE_FLAG_GUIDED_ENABLED = 8
MAV_MODE_FLAG_STABILIZE_ENABLED = 16
MAV_MODE_FLAG_HIL_ENABLED = 32
MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
MAV_MODE_FLAG_SAFETY_ARMED = 128

# 兼容性常量（用于旧代码）
MAV_MODE_STABILIZE_ARMED = MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED
MAV_MODE_STABILIZE_DISARMED = MAV_MODE_FLAG_STABILIZE_ENABLED

# Message IDs
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_SET_MODE = 11
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_COMMAND_ACK = 77

# CRC Extra values for message validation
CRC_EXTRA = {
    MAVLINK_MSG_ID_HEARTBEAT: 50,
    MAVLINK_MSG_ID_SET_MODE: 89,
    MAVLINK_MSG_ID_COMMAND_LONG: 152,
    MAVLINK_MSG_ID_COMMAND_ACK: 143,
    84: 143
}

def crc_x25(data):
    """Calculate CRC for MAVLink with CRC_EXTRA"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_heartbeat(seq):
    payload = struct.pack('<IBBBBB', 0, 18, 8, 0, 0, 3)
    header = struct.pack('<6B', MAVLINK_STX, len(payload), seq & 255,
                         MAVLINK_SYSID, MAVLINK_COMPID, 0)
    crc = crc_x25(header[1:] + payload + bytes([50]))
    return header + payload + struct.pack('<H', crc)

def build_command_long(seq, command, param1=0, param2=0, param3=0,
                       param4=0, param5=0, param6=0, param7=0,
                       target_sys=1, target_comp=1):
    payload = struct.pack('<7fHBBB', param1, param2, param3, param4,
                          param5, param6, param7, command, target_sys,
                          target_comp, 0)
    header = struct.pack('<6B', MAVLINK_STX, len(payload), seq & 255,
                         MAVLINK_SYSID, MAVLINK_COMPID, 76)
    crc = crc_x25(header[1:] + payload + bytes([152]))
    return header + payload + struct.pack('<H', crc)

def build_set_mode(seq, base_mode, custom_mode, target_sys=1):
    payload = struct.pack('<IBB', custom_mode, target_sys, base_mode)
    header = struct.pack('<6B', MAVLINK_STX, len(payload), seq & 255,
                         MAVLINK_SYSID, MAVLINK_COMPID, 11)
    crc = crc_x25(header[1:] + payload + bytes([89]))
    return header + payload + struct.pack('<H', crc)

def build_motor_test_command(seq, motor_instance, throttle_type, throttle_value, 
                           timeout_sec=10, motor_count=4, test_order=0, target_sys=1, target_comp=1):
    """
    Build motor test command using MAV_CMD_DO_MOTOR_TEST
    motor_instance: 1-4 for individual motors, 0 for all
    throttle_type: 0=PWM, 1=percentage, 2=pilot throttle channel
    throttle_value: throttle value based on type
    """
    return build_command_long(
        seq=seq,
        command=MAV_CMD_DO_MOTOR_TEST,
        param1=float(motor_instance),
        param2=float(throttle_type),
        param3=float(throttle_value),
        param4=float(timeout_sec),
        param5=float(motor_count),
        param6=float(test_order),
        param7=0,
        target_sys=target_sys,
        target_comp=target_comp
    )

def parse_mavlink_message(data, buffer=None):
    """Decode the supported MAVLink 1 frames; keep a caller-owned stream buffer."""
    if buffer is None:
        buffer = bytearray()
    buffer.extend(data)
    messages = []
    while len(buffer) >= 8:
        if buffer[0] != MAVLINK_STX:
            del buffer[0]
            continue
        frame_len = buffer[1] + 8
        if len(buffer) < frame_len:
            break
        msg_id = buffer[5]
        extra = CRC_EXTRA.get(msg_id)
        frame = bytes(buffer[:frame_len])
        if extra is None:
            del buffer[:frame_len]
            continue
        received_crc = struct.unpack('<H', frame[-2:])[0]
        if crc_x25(frame[1:-2] + bytes([extra])) != received_crc:
            del buffer[0]
            continue
        del buffer[:frame_len]
        if msg_id == 0 and frame[1] != 9:
            continue
        messages.append({'seq': frame[2], 'system_id': frame[3],
                         'component_id': frame[4], 'msg_id': msg_id,
                         'payload': frame[6:-2], 'crc': received_crc})
    return messages

def build_set_position_target_local_ned(seq, vx=0, vy=0, vz=0, yaw_rate=0.0,
                                         target_sys=1, target_comp=1):
    type_mask = 0x05C7  # Ignore position, acceleration and yaw; use velocity/yaw rate.
    payload = struct.pack('<I11fHBBB', 0, 0, 0, 0, vx, vy, vz,
                          0, 0, 0, 0, yaw_rate, type_mask,
                          target_sys, target_comp, 1)
    header = struct.pack('<6B', MAVLINK_STX, len(payload), seq & 255,
                         MAVLINK_SYSID, MAVLINK_COMPID, 84)
    crc = crc_x25(header[1:] + payload + bytes([143]))
    return header + payload + struct.pack('<H', crc)

