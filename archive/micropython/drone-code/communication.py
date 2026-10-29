"""
Communication module for interfacing with FCM and external systems.
Handles I2C communication with flight controller and Wi-Fi transmission.
"""
import machine
import network
import socket
import time
import ubinascii
import ujson
from config import (
    I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQUENCY, FCM_I2C_ADDRESS,
    FCM_COMMANDS, WIFI_SSID, WIFI_PASSWORD, SERVER_IP, SERVER_PORT,
    WIFI_CONNECTION_TIMEOUT, SOCKET_TIMEOUT
)
from utils import log

class FCMInterface:
    """Interface for communication with Flight Control Module via I2C"""
    
    def __init__(self):
        """Initialize I2C communication"""
        try:
            self.i2c = machine.I2C(1, 
                                   scl=machine.Pin(I2C_SCL_PIN), 
                                   sda=machine.Pin(I2C_SDA_PIN), 
                                   freq=I2C_FREQUENCY)
            self.fcm_addr = FCM_I2C_ADDRESS
            self.initialized = True
            
            # Check if FCM is responding
            devices = self.i2c.scan()
            if self.fcm_addr not in devices:
                log(f"FCM not found on I2C bus. Available devices: {devices}", "ERROR")
                self.initialized = False
            else:
                log("FCM I2C interface initialized successfully")
                
            # Initialize command sequence counter
            self.cmd_sequence = 0
            
            # Last known state tracking
            self.last_command = None
            self.last_data = None
            self.retry_count = 0
            self.max_retries = 3
            
        except Exception as e:
            self.initialized = False
            log(f"FCM I2C interface initialization failed: {e}", "ERROR")
    
    def send_command(self, command, data=None):
        """
        Send command to FCM.
        
        Args:
            command: Command code from FCM_COMMANDS
            data: Optional data payload as list/tuple or dict
        
        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.initialized:
            log("FCM interface not initialized", "ERROR")
            return False
        
        cmd_byte = FCM_COMMANDS.get(command)
        if cmd_byte is None:
            log(f"Unknown command: {command}", "ERROR")
            return False
        
        try:
            # Prepare command packet
            packet = bytearray([cmd_byte])
            
            # Add sequence number to detect lost commands
            packet.extend(bytes([self.cmd_sequence & 0xFF]))
            self.cmd_sequence = (self.cmd_sequence + 1) & 0xFF
            
            # Add data if provided
            if data:
                if isinstance(data, (list, tuple)):
                    # Convert numerical data to bytes
                    for value in data:
                        if isinstance(value, float):
                            # Convert float to 4 bytes
                            import struct
                            packet.extend(struct.pack('f', value))
                        elif isinstance(value, int):
                            # Convert int to 2 bytes
                            packet.extend(bytes([value >> 8, value & 0xFF]))
                elif isinstance(data, dict):
                    # Convert dictionary to JSON
                    json_data = ujson.dumps(data)
                    packet.extend(json_data.encode())
            
            # Send packet to FCM
            self.i2c.writeto(self.fcm_addr, packet)
            
            # Wait for acknowledgment (if implemented on FCM)
            # In a real implementation, we would wait for a response here
            
            log(f"Sent command {command} to FCM")
            
            # Store last command for retry if needed
            self.last_command = command
            self.last_data = data
            self.retry_count = 0
            
            return True
            
        except Exception as e:
            log(f"Failed to send command to FCM: {e}", "ERROR")
            
            # Attempt retry for critical commands
            if command in ["SET_POSITION", "SET_VELOCITY", "EMERGENCY_STOP"]:
                self.retry_count += 1
                if self.retry_count <= self.max_retries:
                    log(f"Retrying command {command} (attempt {self.retry_count}/{self.max_retries})")
                    time.sleep(0.1)
                    return self.send_command(command, data)
            
            return False
    
    def motors_on(self):
        """Turn on motors"""
        return self.send_command("MOTORS_ON")
    
    def motors_off(self):
        """Turn off motors"""
        return self.send_command("MOTORS_OFF")
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        # Send command multiple times to ensure delivery
        success = self.send_command("EMERGENCY_STOP")
        time.sleep(0.1)
        self.send_command("EMERGENCY_STOP")  # Send again regardless of first result
        return success
    
    def set_position(self, x, y, z):
        """Set target position"""
        return self.send_command("SET_POSITION", (x, y, z))
    
    def set_velocity(self, vx, vy, vz):
        """Set velocity vector"""
        return self.send_command("SET_VELOCITY", (vx, vy, vz))
    
    def set_altitude(self, altitude):
        """Set target altitude"""
        return self.send_command("SET_ALTITUDE", (altitude,))
    
    def set_attitude(self, roll, pitch, yaw):
        """Set target attitude (in degrees)"""
        return self.send_command("SET_ATTITUDE", (roll, pitch, yaw))

class WirelessCommunication:
    """Handles Wi-Fi communication for mission status transmission"""
    
    def __init__(self):
        """Initialize Wi-Fi module"""
        self.wlan = network.WLAN(network.STA_IF)
        self.initialized = False
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 5
    
    def initialize(self):
        """Initialize Wi-Fi hardware"""
        try:
            if not self.wlan.active():
                self.wlan.active(True)
            self.initialized = True
            log("Wireless communication module initialized")
            return True
        except Exception as e:
            log(f"Failed to initialize wireless module: {e}", "ERROR")
            return False
    
    def connect(self, timeout=WIFI_CONNECTION_TIMEOUT):
        """
        Connect to Wi-Fi network.
        
        Args:
            timeout: Connection timeout in seconds
        
        Returns:
            True if connected successfully, False otherwise
        """
        if not self.initialized:
            if not self.initialize():
                return False
        
        try:
            # Check if already connected
            if self.wlan.isconnected():
                self.connected = True
                log(f"Already connected to Wi-Fi. IP: {self.wlan.ifconfig()[0]}")
                return True
            
            # Increment connection attempts
            self.connection_attempts += 1
            if self.connection_attempts > self.max_connection_attempts:
                log(f"Maximum connection attempts ({self.max_connection_attempts}) reached", "ERROR")
                return False
            
            # Connect to AP
            log(f"Connecting to Wi-Fi network: {WIFI_SSID} (attempt {self.connection_attempts}/{self.max_connection_attempts})")
            self.wlan.connect(WIFI_SSID, WIFI_PASSWORD)
            
            # Wait for connection with timeout
            start_time = time.time()
            while not self.wlan.isconnected():
                if time.time() - start_time > timeout:
                    log("Wi-Fi connection timeout", "WARNING")
                    return False
                time.sleep(0.1)
            
            self.connected = True
            ip_address = self.wlan.ifconfig()[0]
            log(f"Connected to Wi-Fi. IP address: {ip_address}")
            
            # Reset connection attempts on success
            self.connection_attempts = 0
            
            return True
            
        except Exception as e:
            log(f"Failed to connect to Wi-Fi: {e}", "ERROR")
            return False
    
    def send_mission_complete(self, message):
        """
        Send mission complete message to server.
        
        Args:
            message: Message string to send
        
        Returns:
            True if message sent successfully, False otherwise
        """
        if not self.connected:
            if not self.connect():
                return False
        
        try:
            # Create socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(SOCKET_TIMEOUT)
            
            # Connect to server
            log(f"Connecting to server {SERVER_IP}:{SERVER_PORT}")
            sock.connect((SERVER_IP, SERVER_PORT))
            
            # Add unique identifier to message
            device_id = ubinascii.hexlify(machine.unique_id()).decode()
            full_message = f"{device_id}|{message}"
            
            # Send message
            log(f"Sending message: {full_message}")
            sock.send(full_message.encode())
            
            # Wait for acknowledgment
            log("Waiting for server acknowledgment...")
            response = sock.recv(1024).decode().strip()
            sock.close()
            
            log(f"Mission complete message sent. Response: {response}")
            return True
            
        except Exception as e:
            log(f"Failed to send mission complete message: {e}", "ERROR")
            return False
    
    def disconnect(self):
        """Disconnect from Wi-Fi network"""
        if self.connected:
            self.wlan.disconnect()
            self.connected = False
            log("Disconnected from Wi-Fi network")
    
    def get_signal_strength(self):
        """Get current Wi-Fi signal strength (RSSI)"""
        if self.connected and self.wlan.isconnected():
            try:
                return self.wlan.status('rssi')
            except:
                return None
        return None