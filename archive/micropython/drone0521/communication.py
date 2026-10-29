"""
Communication module for interfacing with FCM (Holybro Pixhawk4).
This module uses I2C protocol to emulate RC functionality, enabling communication
between ACM and Pixhawk4.

Main functionality:
1. Send flight control commands to Pixhawk4 via I2C protocol
2. Support position, velocity, attitude control commands
3. Support RC channel override (rc_override) mode to precisely emulate RC control
4. Provide safety mechanisms like command retransmission and emergency stop
5. HC12 wireless communication for mission complete transmission

Author: Score99 team
Date: 2025 April
"""
import machine
import time
import struct
import ujson
from config import (
    I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQUENCY, FCM_I2C_ADDRESS,
    FCM_COMMANDS, HC12_TX_PIN, HC12_RX_PIN, HC12_SET_PIN, 
    HC12_BAUD_RATE, HC12_CHANNEL
)
from utils import log

class HC12Interface:
    """
    Interface class for communicating with HC12 wireless module via UART.
    Used for transmitting mission complete message in Task 6.
    """
    
    def __init__(self):
        """
        Initialize UART communication with HC12 module.
        Configure the module with specified channel and baud rate.
        """
        try:
            # Initialize HC12 SET pin for configuration
            self.set_pin = machine.Pin(HC12_SET_PIN, machine.Pin.OUT)
            self.set_pin.value(1)  # Set HIGH for normal operation
            
            # Initialize UART for HC12 communication
            self.uart = machine.UART(2, baudrate=HC12_BAUD_RATE, tx=HC12_TX_PIN, rx=HC12_RX_PIN)
            self.uart.init(bits=8, parity=None, stop=1, timeout=1000)
            
            # Configure HC12 channel (enter AT command mode)
            self._enter_command_mode()
            self._set_channel(HC12_CHANNEL)
            self._exit_command_mode()
            
            self.initialized = True
            log("HC12 communication interface initialized successfully")
            
        except Exception as e:
            self.initialized = False
            log(f"HC12 communication interface initialization failed: {e}", "ERROR")
    
    def _enter_command_mode(self):
        """Enter AT command mode"""
        self.set_pin.value(0)  # Set LOW to enter command mode
        time.sleep(0.1)  # Wait for mode change
    
    def _exit_command_mode(self):
        """Exit AT command mode"""
        self.set_pin.value(1)  # Set HIGH to exit command mode
        time.sleep(0.1)  # Wait for mode change
    
    def _set_channel(self, channel):
        """Set HC12 channel"""
        if not 1 <= channel <= 127:
            log(f"Invalid HC12 channel: {channel}, must be 1-127", "ERROR")
            return False
            
        # Send AT command to set channel
        command = f"AT+C{channel:03d}\r\n".encode()
        self.uart.write(command)
        time.sleep(0.1)  # Wait for command processing
        
        # Read response
        response = None
        if self.uart.any():
            response = self.uart.read()
            
        # Verify response
        if response and b"OK" in response:
            log(f"HC12 channel set to {channel}")
            return True
        else:
            log("Failed to set HC12 channel", "ERROR")
            return False
    
    def send_message(self, message):
        """
        Send a message through HC12 module.
        
        Args:
            message: String message to send
            
        Returns:
            True if message sent successfully, False otherwise
        """
        if not self.initialized:
            log("HC12 interface not initialized, cannot send message", "ERROR")
            return False
            
        try:
            # Encode message and send
            encoded_message = message.encode()
            self.uart.write(encoded_message)
            log(f"Message sent via HC12: {message}")
            return True
            
        except Exception as e:
            log(f"Failed to send message via HC12: {e}", "ERROR")
            return False
    
    def receive_message(self, timeout_ms=1000):
        """
        Receive a message from HC12 module.
        
        Args:
            timeout_ms: Maximum time to wait for message (milliseconds)
            
        Returns:
            Received message string or None if no message received
        """
        if not self.initialized:
            log("HC12 interface not initialized, cannot receive message", "ERROR")
            return None
            
        try:
            # Set timeout
            self.uart.timeout = timeout_ms
            
            # Check if data is available
            if self.uart.any():
                # Read and decode message
                data = self.uart.read()
                if data:
                    message = data.decode().strip()
                    log(f"Message received via HC12: {message}")
                    return message
            
            return None
            
        except Exception as e:
            log(f"Failed to receive message via HC12: {e}", "ERROR")
            return None

class FCMInterface:
    """
    Interface class for communicating with Holybro Pixhawk4 flight control module via I2C.
    This class emulates RC functionality to translate high-level control commands
    into a format Pixhawk4 can understand.
    Supports two control modes:
    1. Command mode: Directly send position, velocity, or attitude commands
    2. RC override mode: Emulate standard 8-channel RC signals
    """
    
    def __init__(self):
        """
        Initialize I2C communication.
        Configure I2C bus and check if Pixhawk4 is available.
        """
        try:
            # Configure I2C interface with specified pins and frequency
            # Pixhawk4 operates in I2C slave mode, ESP32 as master
            self.i2c = machine.I2C(1, 
                                   scl=machine.Pin(I2C_SCL_PIN), 
                                   sda=machine.Pin(I2C_SDA_PIN), 
                                   freq=I2C_FREQUENCY)
            self.fcm_addr = FCM_I2C_ADDRESS
            self.initialized = True
            
            # Check if Pixhawk4 is responding
            devices = self.i2c.scan()
            if self.fcm_addr not in devices:
                log(f"Pixhawk4 not found on I2C bus (address:0x{self.fcm_addr:02X}). Available devices: {[hex(d) for d in devices]}", "ERROR")
                self.initialized = False
                log("Check wiring and ensure Pixhawk4 is powered on", "ERROR")
            else:
                log(f"Pixhawk4 I2C interface initialized successfully, device address:0x{self.fcm_addr:02X}")
                
            # Initialize command sequence counter (for command tracking)
            self.cmd_sequence = 0
            
            # Last command status tracking (for retry mechanism)
            self.last_command = None
            self.last_data = None
            self.last_timestamp = 0
            self.retry_count = 0
            self.max_retries = 3
            self.heartbeat_interval = 1.0  # seconds
            self.last_heartbeat = time.time()
            
            # Pixhawk4 parameters
            self.armed = False
            self.flight_mode = "STABILIZE"  # Default flight mode
            
            # RC channel settings, default to center values
            # Channel 1: Roll - 1500 is neutral
            # Channel 2: Pitch - 1500 is neutral
            # Channel 3: Throttle - 1000 is lowest position
            # Channel 4: Yaw - 1500 is neutral
            # Channel 5: Flight mode switch - 1100/1500/1900 for different modes
            # Channels 6-8: Auxiliary channels
            self.rc_channels = [1500, 1500, 1000, 1500, 1100, 1100, 1100, 1100]
            
        except Exception as e:
            self.initialized = False
            log(f"Pixhawk4 I2C interface initialization failed: {e}", "ERROR")
    
    def _calculate_checksum(self, data):
        """
        Calculate data checksum to ensure data integrity.
        
        Args:
            data: Byte array
            
        Returns:
            Checksum (16-bit)
        """
        crc = 0xFFFF
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return crc
    
    def send_command(self, command, data=None):
        """
        Send command to Pixhawk4.
        
        Args:
            command: Command code from FCM_COMMANDS configuration
            data: Optional data payload, can be list/tuple or dictionary
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.initialized:
            log("Pixhawk4 interface not initialized, cannot send command", "ERROR")
            return False
        
        cmd_byte = FCM_COMMANDS.get(command)
        if cmd_byte is None:
            log(f"Unknown command: {command}", "ERROR")
            return False
        
        try:
            # Prepare command packet
            # Pixhawk4 MAVLink compatible format: start marker(1 byte) + length(1 byte) + sequence(1 byte) + 
            # system ID(1 byte) + component ID(1 byte) + command ID(1 byte) + data + checksum(2 bytes)
            start_marker = 0xFE  # MAVLink v1 compatible marker
            system_id = 1        # Typically 1 represents autopilot
            component_id = 1     # Main flight controller component
            
            # Build basic frame header
            header = bytearray([start_marker, 0, self.cmd_sequence & 0xFF, system_id, component_id, cmd_byte])
            
            # Build data section
            payload = bytearray()
            if data:
                if isinstance(data, (list, tuple)):
                    # Convert numeric data to bytes
                    for value in data:
                        if isinstance(value, float):
                            # Convert float to 4 bytes (IEEE 754)
                            payload.extend(struct.pack('<f', value))  # Little endian
                        elif isinstance(value, int):
                            # Convert int to 4 bytes (Pixhawk4 requires 4-byte alignment)
                            payload.extend(struct.pack('<i', value))  # Little endian
                elif isinstance(data, dict):
                    # Convert dictionary to JSON then to bytes
                    json_data = ujson.dumps(data)
                    payload.extend(json_data.encode())
            
            # Update frame length
            header[1] = len(payload)
            
            # Combine complete frame
            frame = header + payload
            
            # Calculate and add checksum
            checksum = self._calculate_checksum(frame[1:])
            frame.extend(struct.pack('<H', checksum))  # Little endian
            
            # Send packet to Pixhawk4
            self.i2c.writeto(self.fcm_addr, frame)
            
            # Wait for acknowledgement (can be expanded to read Pixhawk4 response)
            time.sleep(0.01)
            
            # Record command sent status
            log(f"Command sent to Pixhawk4: {command}, sequence: {self.cmd_sequence}")
            self.last_command = command
            self.last_data = data
            self.last_timestamp = time.time()
            self.retry_count = 0
            self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
            
            # Update internal state for certain commands
            if command == "MOTORS_ON":
                self.armed = True
            elif command == "MOTORS_OFF":
                self.armed = False
                
            return True
            
        except Exception as e:
            log(f"Failed to send command to Pixhawk4: {e}", "ERROR")
            
            # Try to retry critical commands
            if command in ["SET_POSITION", "SET_VELOCITY", "EMERGENCY_STOP", "RC_OVERRIDE"]:
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    log(f"Retrying command {command} (attempt {self.retry_count}/{self.max_retries})")
                    time.sleep(0.1)
                    return self.send_command(command, data)
            
            return False
    
    def read_status(self):
        """
        Read status information from Pixhawk4.
        
        Returns:
            Status dictionary or None (if read failed)
        """
        if not self.initialized:
            return None
        
        try:
            # Send status request command
            if 'GET_STATUS' in FCM_COMMANDS:
                status_cmd = FCM_COMMANDS['GET_STATUS']
                cmd_frame = bytearray([0xFE, 0, self.cmd_sequence & 0xFF, 1, 1, status_cmd])
                checksum = self._calculate_checksum(cmd_frame[1:])
                cmd_frame.extend(struct.pack('<H', checksum))
                
                self.i2c.writeto(self.fcm_addr, cmd_frame)
                self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
                
                # Wait for Pixhawk4 to prepare response
                time.sleep(0.02)
                
                # Read status response (assume max 128 bytes)
                try:
                    response = self.i2c.readfrom(self.fcm_addr, 128)
                    
                    # Validate response format
                    if len(response) > 6 and response[0] == 0xFE:
                        # Simple response parsing (should be detailed based on Pixhawk4 protocol)
                        msg_len = response[1]
                        if len(response) >= msg_len + 8:  # Header(6) + data(msg_len) + checksum(2)
                            # Verify checksum
                            recv_checksum = struct.unpack('<H', response[msg_len+6:msg_len+8])[0]
                            calc_checksum = self._calculate_checksum(response[1:msg_len+6])
                            
                            if recv_checksum == calc_checksum:
                                # Extract data portion
                                data = response[6:msg_len+6]
                                
                                # Parse data based on command type
                                # This is a simplified example, actual should parse based on Pixhawk4 format
                                status = {
                                    'armed': data[0] if len(data) > 0 else 0,
                                    'flight_mode': data[1] if len(data) > 1 else 0,
                                    'battery': struct.unpack('<f', data[2:6])[0] if len(data) > 5 else 0,
                                    'altitude': struct.unpack('<f', data[6:10])[0] if len(data) > 9 else 0
                                }
                                return status
                    
                    log("Invalid Pixhawk4 status response", "WARNING")
                except Exception as e:
                    log(f"Error reading Pixhawk4 response: {e}", "ERROR")
            
            # If unable to read actual status, return stored status
            return {
                'armed': self.armed,
                'flight_mode': self.flight_mode,
                'rc_channels': self.rc_channels.copy()
            }
                
        except Exception as e:
            log(f"Error getting Pixhawk4 status: {e}", "ERROR")
            return None
    
    def update(self):
        """
        Periodic update, send heartbeat to keep connection active.
        Call this method periodically in the main loop.
        
        Returns:
            True if update successful, False otherwise
        """
        current_time = time.time()
        
        # Check if heartbeat needs to be sent
        if current_time - self.last_heartbeat >= self.heartbeat_interval:
            try:
                # Heartbeat packet, simple no-payload command
                if 'HEARTBEAT' in FCM_COMMANDS:
                    heartbeat_cmd = FCM_COMMANDS['HEARTBEAT']
                    cmd_frame = bytearray([0xFE, 0, self.cmd_sequence & 0xFF, 1, 1, heartbeat_cmd])
                    checksum = self._calculate_checksum(cmd_frame[1:])
                    cmd_frame.extend(struct.pack('<H', checksum))
                    
                    self.i2c.writeto(self.fcm_addr, cmd_frame)
                    self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
                    self.last_heartbeat = current_time
                    return True
            except Exception as e:
                log(f"Failed to send heartbeat: {e}", "WARNING")
                return False
        
        return True
    
    # Basic command API
    
    def motors_on(self):
        """
        Turn on motors (arm Pixhawk4).
        Equivalent to RC arming operation.
        """
        log("Command: Motors ON (arm)")
        return self.send_command("MOTORS_ON")
    
    def motors_off(self):
        """
        Turn off motors (disarm Pixhawk4).
        Equivalent to RC disarming operation.
        """
        log("Command: Motors OFF (disarm)")
        return self.send_command("MOTORS_OFF")
    
    def emergency_stop(self):
        """
        Emergency stop all motors.
        Equivalent to RC emergency stop switch.
        """
        log("Command: EMERGENCY STOP", "WARNING")
        # Send command multiple times to ensure delivery
        success = self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")  # Send again regardless of first result
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")  # Third time
        self.armed = False
        return success
    
    def set_position(self, x, y, z):
        """
        Set target position (position mode).
        Equivalent to RC stick operations in position mode.
        
        Args:
            x, y, z: Position coordinates (meters)
        """
        # Ensure position mode
        if self.flight_mode != "POSITION":
            self.set_flight_mode("POSITION")
            
        log(f"Command: Set position to ({x:.2f}, {y:.2f}, {z:.2f})m")
        return self.send_command("SET_POSITION", (x, y, z))
    
    def set_velocity(self, vx, vy, vz):
        """
        Set velocity vector (velocity mode).
        Equivalent to RC stick operations in velocity mode.
        
        Args:
            vx, vy, vz: Velocity components (meters/second)
        """
        log(f"Command: Set velocity to ({vx:.2f}, {vy:.2f}, {vz:.2f})m/s")
        return self.send_command("SET_VELOCITY", (vx, vy, vz))
    
    def set_altitude(self, altitude):
        """
        Set target altitude.
        Equivalent to RC throttle control (in altitude hold mode).
        
        Args:
            altitude: Target altitude (meters)
        """
        log(f"Command: Set altitude to {altitude:.2f}m")
        return self.send_command("SET_ALTITUDE", (altitude,))
    
    def set_attitude(self, roll, pitch, yaw):
        """
        Set target attitude (angles).
        Equivalent to RC attitude control (in attitude mode).
        
        Args:
            roll: Roll angle (degrees), controls left/right tilt
            pitch: Pitch angle (degrees), controls forward/backward tilt
            yaw: Yaw angle (degrees), controls horizontal rotation
        """
        log(f"Command: Set attitude to (roll:{roll:.1f}°, pitch:{pitch:.1f}°, yaw:{yaw:.1f}°)")
        return self.send_command("SET_ATTITUDE", (roll, pitch, yaw))
    
    def set_flight_mode(self, mode):
        """
        Set Pixhawk4 flight mode - optimized for position control.
        Equivalent to RC flight mode switch.
        
        Args:
            mode: Flight mode string, e.g. "STABILIZE", "POSITION", etc.
        
        Returns:
            Setting successful returns True, otherwise False
        """
        if not self.initialized:
            log("Pixhawk4 interface not initialized, cannot set flight mode", "ERROR")
            return False
            
        try:
            # Prioritize "POSITION" mode for preset path following
            valid_modes = ["POSITION", "STABILIZE", "ALTITUDE", "LOITER", "MISSION", "RTL", "LAND"]
            if mode not in valid_modes:
                log(f"Invalid flight mode: {mode}, valid modes are: {valid_modes}", "ERROR")
                return False
                
            # Use "POSITION" mode as default for obstacle avoidance
            if mode == "STABILIZE" and self.flight_mode != "POSITION":
                log("Using POSITION mode instead of STABILIZE for better path following")
                mode = "POSITION"
            
            # Map mode to Pixhawk4 mode number
            mode_map = {
                "STABILIZE": 0,
                "POSITION": 3,
                "ALTITUDE": 2,
                "LOITER": 5,
                "MISSION": 4,
                "RTL": 6,
                "LAND": 9
            }
            
            mode_number = mode_map[mode]
            
            # Send mode switch command
            if 'SET_MODE' in FCM_COMMANDS:
                log(f"Command: Setting flight mode to {mode}")
                result = self.send_command("SET_MODE", (mode_number,))
                if result:
                    self.flight_mode = mode
                return result
            else:
                log("Flight mode setting command not defined", "ERROR")
                return False
                
        except Exception as e:
            log(f"Error setting flight mode: {e}", "ERROR")
            return False
    
    def takeoff(self, altitude):
        """
        Automatic takeoff to specified height.
        Equivalent to RC auto takeoff function.
        
        Args:
            altitude: Target altitude (meters)
        
        Returns:
            True if command sent successfully, False otherwise
        """
        log(f"Command: Automatic takeoff to {altitude:.2f}m")
        
        # Ensure motors are armed
        if not self.armed:
            self.motors_on()
            time.sleep(0.5)  # Wait for motors to start
            
        # Send takeoff command
        if 'TAKEOFF' in FCM_COMMANDS:
            return self.send_command("TAKEOFF", (altitude,))
        else:
            # If no dedicated takeoff command, simulate with position control
            self.set_flight_mode("POSITION")
            time.sleep(0.5)
            return self.set_position(0, 0, altitude)  # Assume takeoff point is origin
    
    def land(self):
        """
        Automatic landing.
        Equivalent to RC auto land function.
        
        Returns:
            True if command sent successfully, False otherwise
        """
        log("Command: Automatic landing")
        
        # Send landing command
        if 'LAND' in FCM_COMMANDS:
            return self.send_command("LAND")
        else:
            # If no dedicated landing command, switch to land mode
            return self.set_flight_mode("LAND")
    
    # RC channel control API
    
    def rc_override(self, channels):
        """
        Send RC channel override command, directly emulate RC signals.
        
        Args:
            channels: List containing 8 channel values (1000-2000)
            
        Returns:
            True if command sent successfully, False otherwise
        """
        if len(channels) != 8:
            log(f"RC channel count error: need 8, provided {len(channels)}", "ERROR")
            return False
            
        # Validate channel value range
        for i, value in enumerate(channels):
            if value < 800 or value > 2200:
                log(f"Channel {i+1} value out of range (800-2200): {value}", "WARNING")
                # Correct to valid range
                channels[i] = max(800, min(2200, value))
        
        # Update internal channel state
        self.rc_channels = channels.copy()
        
        # Send RC override command
        log(f"Command: RC channel override {channels}")
        return self.send_command("RC_OVERRIDE", channels)
    
    def arm_motors_rc(self):
        """Arm motors using RC channel commands"""
        log("RC Command: Arm motors")
        # Throttle low, yaw right
        channels = self.rc_channels.copy()
        channels[2] = 1000  # Throttle minimum
        channels[3] = 2000  # Yaw maximum right
        self.rc_override(channels)
        time.sleep(2)  # Wait 2 seconds to ensure command is processed
        
        # Return yaw to center
        channels[3] = 1500
        self.rc_override(channels)
        self.armed = True
        return True
    
    def disarm_motors_rc(self):
        """Disarm motors using RC channel commands"""
        log("RC Command: Disarm motors")
        # Throttle low, yaw left
        channels = self.rc_channels.copy()
        channels[2] = 1000  # Throttle minimum
        channels[3] = 1000  # Yaw maximum left
        self.rc_override(channels)
        time.sleep(2)  # Wait 2 seconds to ensure command is processed
        
        # Return yaw to center
        channels[3] = 1500
        self.rc_override(channels)
        self.armed = False
        return True
    
    def takeoff_rc(self, target_height=2.0, step=50, delay=0.1):
        """
        Takeoff using RC channel commands to specified height
        
        Args:
            target_height: Target height (meters)
            step: Throttle increase step
            delay: Delay between steps (seconds)
        """
        log(f"RC Command: Takeoff, target height: {target_height}m")
        
        # Ensure armed
        if not self.armed:
            self.arm_motors_rc()
            time.sleep(1)
        
        # Gradually increase throttle
        channels = self.rc_channels.copy()
        for throttle in range(1000, 1500, step):
            channels[2] = throttle
            self.rc_override(channels)
            time.sleep(delay)
        
        # Switch to altitude hold mode
        self.set_rc_flight_mode(2)  # Switch to altitude hold mode
        time.sleep(3)  # Wait for altitude to stabilize
        
        log("RC Command: Target height reached")
        return True
    
    def land_rc(self, step=20, delay=0.2):
        """
        Land using RC channel commands
        
        Args:
            step: Throttle decrease step
            delay: Delay between steps (seconds)
        """
        log("RC Command: Begin landing")
        
        # Gradually decrease throttle
        channels = self.rc_channels.copy()
        for throttle in range(channels[2], 1100, -step):
            channels[2] = throttle
            self.rc_override(channels)
            time.sleep(delay)
        
        # Final reduction to minimum throttle
        channels[2] = 1000
        self.rc_override(channels)
        
        # Wait for complete landing
        time.sleep(3)
        
        # Disarm motors
        self.disarm_motors_rc()
        
        log("RC Command: Landing complete")
        return True
    
    def move_forward_rc(self, speed=200, duration=2):
        """
        Move forward using RC channel commands
        
        Args:
            speed: Speed (0-500)
            duration: Duration (seconds)
        """
        log(f"RC Command: Move forward, speed:{speed}, time:{duration}s")
        channels = self.rc_channels.copy()
        # Increase pitch channel to tilt drone forward
        channels[1] = 1500 + speed
        self.rc_override(channels)
        
        # Hold for specified time
        time.sleep(duration)
        
        # Return to center
        channels[1] = 1500
        self.rc_override(channels)
        return True
    
    def move_backward_rc(self, speed=200, duration=2):
        """
        Move backward using RC channel commands
        
        Args:
            speed: Speed (0-500)
            duration: Duration (seconds)
        """
        log(f"RC Command: Move backward, speed:{speed}, time:{duration}s")
        channels = self.rc_channels.copy()
        # Decrease pitch channel to tilt drone backward
        channels[1] = 1500 - speed
        self.rc_override(channels)
        
        # Hold for specified time
        time.sleep(duration)
        
        # Return to center
        channels[1] = 1500
        self.rc_override(channels)
        return True
    
    def move_left_rc(self, speed=200, duration=2):
        """
        Move left using RC channel commands
        
        Args:
            speed: Speed (0-500)
            duration: Duration (seconds)
        """
        log(f"RC Command: Move left, speed:{speed}, time:{duration}s")
        channels = self.rc_channels.copy()
        # Decrease roll channel to tilt drone left
        channels[0] = 1500 - speed
        self.rc_override(channels)
        
        # Hold for specified time
        time.sleep(duration)
        
        # Return to center
        channels[0] = 1500
        self.rc_override(channels)
        return True
    
    def move_right_rc(self, speed=200, duration=2):
        """
        Move right using RC channel commands
        
        Args:
            speed: Speed (0-500)
            duration: Duration (seconds)
        """
        log(f"RC Command: Move right, speed:{speed}, time:{duration}s")
        channels = self.rc_channels.copy()
        # Increase roll channel to tilt drone right
        channels[0] = 1500 + speed
        self.rc_override(channels)
        
        # Hold for specified time
        time.sleep(duration)
        
        # Return to center
        channels[0] = 1500
        self.rc_override(channels)
        return True
    
    def yaw_rotate_rc(self, direction=1, speed=200, angle=90):
        """
        Rotate horizontally using RC channel commands
        
        Args:
            direction: 1=clockwise, -1=counterclockwise
            speed: Rotation speed (0-500)
            angle: Rotation angle (degrees)
        """
        # Calculate rotation time (approximate, should use sensor feedback in practice)
        duration = angle / (speed * 0.1)  # 0.1 is an estimated factor, needs calibration
        
        log(f"RC Command: {'Clockwise' if direction > 0 else 'Counterclockwise'} rotation {angle}°, speed:{speed}, estimated time:{duration:.1f}s")
        
        # Set yaw channel
        channels = self.rc_channels.copy()
        channels[3] = 1500 + (direction * speed)
        self.rc_override(channels)
        
        # Wait for rotation to complete
        time.sleep(duration)
        
        # Return to center
        channels[3] = 1500
        self.rc_override(channels)
        return True
    
    def set_rc_flight_mode(self, mode):
        """
        Set flight mode using RC channel
        
        Args:
            mode: 1=stabilize mode, 2=altitude hold mode, 3=position mode
        """
        if mode not in [1, 2, 3]:
            log(f"Invalid RC flight mode: {mode}, valid modes are: 1=stabilize, 2=altitude hold, 3=position", "ERROR")
            return False
            
        # Map mode to channel value
        mode_values = {
            1: 1100,  # Stabilize mode
            2: 1500,  # Altitude hold mode
            3: 1900   # Position mode
        }
        
        # Set mode channel
        channels = self.rc_channels.copy()
        channels[4] = mode_values[mode]
        
        # Map to internal flight mode
        mode_names = {
            1: "STABILIZE",
            2: "ALTITUDE",
            3: "POSITION"
        }
        
        log(f"RC Command: Set flight mode to {mode_names[mode]}")
        
        # Send RC override command
        result = self.rc_override(channels)
        
        # Update internal flight mode state
        if result:
            self.flight_mode = mode_names[mode]
            
        return result