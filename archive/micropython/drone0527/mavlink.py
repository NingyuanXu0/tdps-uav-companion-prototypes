import struct

# MAVLink constants
MAVLINK_STX = 0xFE
MAVLINK_SYSID = 255     # ESP32 / ground control system ID
MAVLINK_COMPID = 0      # Component ID
MAVLINK_SEQ = 0         # Sequence number, auto-incremented externally

# MAVLink command IDs
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_NAV_TAKEOFF = 22
MAV_MODE_GUIDED_ARMED = 0b100011
MAV_MODE_GUIDED = 0b000011

# Message IDs
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_SET_MODE = 11

def crc_x25(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc & 0xFFFF

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

