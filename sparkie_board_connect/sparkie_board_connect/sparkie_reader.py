#!/usr/bin/env python3
"""
Sparkie ESP32 Data Reader
Reads sensor data from ESP32 device via USB CDC interface
"""

import serial
import serial.tools.list_ports
from sparkie_board_connect.protos import base_pb2
import time
import json
import threading
from datetime import datetime
from google.protobuf.json_format import MessageToJson
from rclpy.logging import RcutilsLogger


class MessageStats:
    """Class to track message statistics"""
    
    def __init__(self):
        self.total_received = 0
        self.packets_received = 0  # Number of packets (single or batch)
        self.batches_received = 0  # Number of batches received
        self.checksum_errors = 0
        self.decode_errors = 0
        self.sync_errors = 0
        self.start_time = time.time()
        self.last_stats_time = time.time()
        
        # Rate tracking per message type
        self.message_rates = {}
    
    def record_success(self, message_type):
        self.total_received += 1
        
        # Track rate per type
        current_time = time.time()
        if message_type not in self.message_rates:
            self.message_rates[message_type] = {
                'count': 0,
                'last_time': current_time
            }
        
        self.message_rates[message_type]['count'] += 1
    
    def record_packet_received(self, is_batch=False, batch_size=1):
        """Record the reception of a packet"""
        self.packets_received += 1
        if is_batch:
            self.batches_received += 1
    
    def record_checksum_error(self):
        self.checksum_errors += 1
    
    def record_decode_error(self):
        self.decode_errors += 1
    
    def record_sync_error(self):
        self.sync_errors += 1


class SparkieDataReader:
    def __init__(self, logger : RcutilsLogger, port='/dev/ttyACM0', baudrate=115200, ):
        """
        Initialize the data reader
        
        Args:
            port: Serial port path (e.g., '/dev/ttyACM0' on Linux, 'COM3' on Windows)
            baudrate: Serial communication speed (default: 115200)
        """
        self.logger = logger
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.stats = MessageStats()
        self.auto_reconnect = True
        self.reconnect_delay = 2.0  # seconds
        self.max_reconnect_attempts = 0  # 0 = infinite attempts
        self.message_handlers = {}
        self.register_message_handler('pingPong', self.handle_ping)
            
    def connect(self):
        """
        Connect to the ESP32 device with retry logic
        """
        if not self.port:
            self.port = '/dev/ttyACM0'
            self.logger.warn(f"No ESP32 device found. Using default port: {self.port}")
            return False
        
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            time.sleep(2)  # Wait for connection to stabilize
            return True
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to {self.port}: {e}")
            return False
    
    def handle_ping(self, message):
        """
        Handle ping messages by sending a pong response
        """
        if message.pingPong.isPing:
            self.send_pong_response(message.pingPong.id, message.topicId)

    def disconnect(self):
        """
        Disconnect from the ESP32 device
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info("Disconnected from ESP32")
    
    def is_connected(self):
        """
        Check if the serial connection is still active
        """
        return (self.serial_conn is not None and 
                self.serial_conn.is_open and 
                self.serial_conn.readable())
    
    def attempt_reconnect(self):
        """
        Attempt to reconnect to the device
        """
        if not self.auto_reconnect:
            return False
        
        self.logger.info("Attempting to reconnect...")
        
        # Close current connection if it exists
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
            self.serial_conn = None
        
        # Try to reconnect
        attempt = 1
        while self.max_reconnect_attempts == 0 or attempt <= self.max_reconnect_attempts:
            self.logger.info(f"Reconnection attempt {attempt}...")
            
            # If we don't have a port, try to find one again
            if not self.port:
                self.port = self.find_esp32_port()
            
            if self.connect():
                self.logger.info("Reconnected successfully!")
                return True
            
            self.logger.warn(f"Reconnection attempt {attempt} failed, waiting {self.reconnect_delay}s...")
            time.sleep(self.reconnect_delay)
            attempt += 1
        
        self.logger.error(f"Failed to reconnect after {attempt-1} attempts")
        return False
    
    def calculate_checksum(self, length, data):
        """
        Calculate the checksum as done in the ESP32 code
        """
        checksum = 0xAA ^ length
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF
    
    def send_message(self, message):
        """
        Send a message to the ESP32 with improved error handling and auto-reconnect
        """
        if not self.is_connected():
            if self.auto_reconnect and self.attempt_reconnect():
                # Successfully reconnected, continue
                pass
            else:
                return False
        
        max_retries = 3
        retry_delay = 0.01  # 10ms delay between retries
        
        for attempt in range(max_retries):
            try:
                # Check connection before each attempt
                if not self.is_connected():
                    if self.auto_reconnect and self.attempt_reconnect():
                        # Successfully reconnected, continue
                        pass
                    else:
                        return False
                
                # Serialize the message
                data = message.SerializeToString()
                length = len(data)
                
                # Validate message length
                if length == 0 or length > 255:
                    self.logger.error(f"Invalid message length: {length}")
                    return False
                
                # Calculate checksum
                checksum = self.calculate_checksum(length, data)
                
                # Clear any pending data in buffers
                if attempt > 0:
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.reset_output_buffer()
                
                # Send the message atomically
                packet = bytes([0xAA]) + bytes([length]) + data + bytes([checksum])
                bytes_written = self.serial_conn.write(packet)
                
                # Verify all bytes were written
                if bytes_written != len(packet):
                    if attempt < max_retries - 1:
                        self.logger.warn(f"Write incomplete, retrying... (attempt {attempt + 1})")
                        time.sleep(retry_delay)
                        continue
                    else:
                        self.logger.error(f"Failed to write complete packet after {max_retries} attempts")
                        return False
                
                # Ensure data is flushed
                self.serial_conn.flush()
                return True
                
            except (serial.SerialException, OSError) as e:
                self.logger.warn(f"Send error: {e}")
                if attempt < max_retries - 1:
                    self.logger.info(f"Retrying send... (attempt {attempt + 1})")
                    time.sleep(retry_delay)
                    continue
                else:
                    self.logger.error(f"Error sending message after {max_retries} attempts")
                    if self.auto_reconnect:
                        self.logger.info("Connection lost, will attempt to reconnect on next operation")
                    return False
            except Exception as e:
                self.logger.error(f"Unexpected error sending message: {e}")
                return False
        
        return False
    
    def send_pong_response(self, ping_id, topic_id):
        """
        Send a pong response to a ping
        
        Args:
            ping_id: The ID from the received ping message
            topic_id: The topic ID from the received ping message (should be 0)
        """
        pong_message = base_pb2.Message()
        pong_message.topicId = topic_id  # Use the same topicId as the ping (usually 0)
        pong_message.timestamp = 0 # Current timestamp in milliseconds
        pong_message.pingPong.id = ping_id  # Use the same ID as the ping
        pong_message.pingPong.isPing = False  # This is a pong response
        
        success = self.send_message(pong_message)
        if not success:
            self.logger.error(f"Failed to send PONG response to ping {ping_id}")
        return success
    
    def send_ledstrip_command(self, effect, layer=1, topic_id=0):
        """
        Send a ledstrip command to the ESP32
        
        Args:
            effect: LED effect (0=off, 1=rainbow, 2=breathing, 3=solid)
            layer: LED layer (default 0)
            topic_id: Topic ID for the message (default 0 for ledstrip)
        """
        ledstrip_message = base_pb2.Message()
        ledstrip_message.topicId = topic_id
        ledstrip_message.timestamp = 0  # Current timestamp in milliseconds
        ledstrip_message.ledstripData.layer = layer
        ledstrip_message.ledstripData.effect = effect
        
        success = self.send_message(ledstrip_message)


        if success:
            effect_names = {0: "off", 1: "rainbow", 2: "breathing", 3: "solid"}
            effect_name = effect_names.get(effect, f"unknown({effect})")
            # self.logger.info(f"Sent LEDSTRIP command: {effect_name} (effect={effect}, layer={layer}, topicId={topic_id})")
        else:
            self.logger.error(f"Failed to send LEDSTRIP command")
        return success

    def send_motor_command(self, speed, topic_id=0):
        """
        Send a motor command to the ESP32
        
        Args:
            speed: Motor speed in m/s
            topic_id: Topic ID for the message (default 0)
        """
        motor_message = base_pb2.Message()
        motor_message.topicId = topic_id
        motor_message.timestamp = 0  # Current timestamp in milliseconds
        motor_message.motorCommand.speed = speed
        
        success = self.send_message(motor_message)
        # if success:
        #     self.logger.info(f"Sent MOTOR command: speed={speed:.3f} m/s (topicId={topic_id})")
        # else:
        #     self.logger.error(f"Failed to send MOTOR command")
        return success
    
    def send_servo_command(self, angle, topic_id=0):
        """
        Send a servo command to the ESP32
        
        Args:
            angle: Servo angle in degrees (typically 0-180)
            topic_id: Topic ID for the message (default 0)
        """
        servo_message = base_pb2.Message()
        servo_message.topicId = topic_id
        servo_message.timestamp = 0  # Current timestamp in milliseconds
        servo_message.servoData.angle_degrees = angle
        
        success = self.send_message(servo_message)
        # if success:
        #     self.logger.info(f"Sent SERVO command: angle={angle:.1f}° (topicId={topic_id})")
        # else:
        #     self.logger.error(f"Failed to send SERVO command")
        return success
    
    def send_speed_command(self, linear, angular, topic_id=0):
        """
        Send a speed command to the ESP32 (differential drive)
        
        Args:
            linear: Linear speed in m/s (forward/backward)
            angular: Angular speed in rad/s (rotation)
            topic_id: Topic ID for the message (default 0)
        """
        speed_message = base_pb2.Message()
        speed_message.topicId = topic_id
        speed_message.timestamp = 0  # Current timestamp in milliseconds
        speed_message.speedCommand.linear = linear
        speed_message.speedCommand.angular = angular
        
        success = self.send_message(speed_message)
        # if success:
        #     print(f"🤖 Sent SPEED command: linear={linear:.3f} m/s, angular={angular:.3f} rad/s (topicId={topic_id})")
        # else:
        #     print(f"❌ Failed to send SPEED command")
        return success

    def read_messages(self):
        """
        Read message(s) from the ESP32 with improved error handling and auto-reconnect
        Returns a list of decoded protobuf messages or empty list if error
        Handles both single Message and MessagePacket (batched messages)
        """
        if not self.is_connected():
            if self.auto_reconnect and self.attempt_reconnect():
                # Successfully reconnected, continue
                pass
            else:
                return []
        
        max_sync_attempts = 100  # Maximum number of bytes to read to find sync
        sync_attempts = 0
        
        try:
            # Search for the start byte in a more robust way
            while sync_attempts < max_sync_attempts:
                start_byte = self.serial_conn.read(1)
                if not start_byte:
                    return []
                    
                if start_byte[0] == 0xAA:
                    break
                sync_attempts += 1
            
            if sync_attempts >= max_sync_attempts:
                self.stats.record_sync_error()
                self.logger.warn("Failed to find start byte after many attempts")
                return []
            
            if sync_attempts > 0:
                self.stats.record_sync_error()
                #self.logger.debug(f"Skipped {sync_attempts} bytes to find sync")
            
            # Read length with timeout
            length_byte = self.serial_conn.read(1)
            if not length_byte:
                return []
            length = length_byte[0]
            
            # More rigorous length validation
            if length == 0 or length > 255:
                self.logger.error(f"Invalid packet length: {length}")
                return []
            
            # Read data with verification
            data = b''
            read_attempts = 0
            max_read_attempts = 10
            
            while len(data) < length and read_attempts < max_read_attempts:
                remaining = length - len(data)
                chunk = self.serial_conn.read(remaining)
                
                if not chunk:
                    read_attempts += 1
                    time.sleep(0.001)  # Small pause
                    continue
                    
                data += chunk
                read_attempts = 0  # Reset on successful read
            
            if len(data) != length:
                self.logger.error(f"Data read incomplete: expected {length}, got {len(data)}")
                return []
            
            # Read checksum
            checksum_byte = self.serial_conn.read(1)
            if not checksum_byte:
                return []
            received_checksum = checksum_byte[0]
            
            # Verify checksum
            calculated_checksum = self.calculate_checksum(length, data)
            if received_checksum != calculated_checksum:
                self.stats.record_checksum_error()
                self.logger.error(f"Checksum mismatch: received 0x{received_checksum:02X}, calculated 0x{calculated_checksum:02X}")
                self.logger.error(f"   Length: {length}, Data: {data.hex()}")
                return []
            
            # Try to decode as MessagePacket first (batch)
            try:
                message_packet = base_pb2.MessagePacket()
                message_packet.ParseFromString(data)
                
                # If there are messages in the packet, return the list
                if len(message_packet.messages) > 0:
                    # Update statistics for each message in the batch
                    for message in message_packet.messages:
                        content_field = message.WhichOneof('content')
                        self.stats.record_success(content_field or 'unknown')
                    
                    # Register the packet as batch
                    self.stats.record_packet_received(is_batch=True, batch_size=len(message_packet.messages))
                    
                    # Log the received batch (only if debug)
                    # self.logger.debug(f"Received batch with {len(message_packet.messages)} messages")
                    return list(message_packet.messages)
                else:
                    # Empty packet
                    self.stats.record_packet_received(is_batch=True, batch_size=0)
                    return []
                    
            except Exception:
                # It's not a MessagePacket, try as single Message
                pass
            
            # Try to decode as single Message (backward compatibility)
            try:
                message = base_pb2.Message()
                message.ParseFromString(data)
                
                # Determine the message type for statistics
                content_field = message.WhichOneof('content')
                self.stats.record_success(content_field or 'unknown')
                
                # Register the packet as single message
                self.stats.record_packet_received(is_batch=False, batch_size=1)
                
                return [message]  # Return as list for uniformity
                
            except Exception as e:
                self.stats.record_decode_error()
                self.logger.error(f"Protobuf decode error (tried both MessagePacket and Message): {e}")
                self.logger.error(f"   Data: {data.hex()}")
                return []
                
        except (serial.SerialException, OSError) as e:
            self.logger.error(f"Error reading message: {e}")
            if self.auto_reconnect:
                self.logger.info("Connection lost, attempting to reconnect...")
                if self.attempt_reconnect():
                    self.logger.info("Reconnected, continuing...")
                    return []  # Return empty list for this read, but connection is restored
            return []
        except Exception as e:
            self.logger.error(f"Unexpected error reading message: {e}")
            return []

    def read_message(self):
        """
        Read a single message from the ESP32 (backward compatibility method)
        Returns the first decoded protobuf message or None if error
        """
        messages = self.read_messages()
        return messages[0] if messages else None
    
    def register_message_handler(self, message_type: str, handler: callable):
        """
        Register a custom message handler for a specific message type
        
        Args:
            message_type: The type of message to handle (e.g., 'ledstripData', 'motorCommand')
            handler: A callable that takes a single argument (the message)
        """
        # if not hasattr(base_pb2.Message, message_type):
        #     raise ValueError(f"Invalid message type: {message_type}")
        
        self.message_handlers[message_type] = handler

    def start_reading(self):
        """
        Start continuous reading of data
        
        Args:
            save_to_file: Optional filename to save data to
            display_format: "pretty" or "json" for output format
            interactive: Enable interactive shell mode
        """
        if not self.connect():
            return
        
        self.reading = True

        self.read_thread = threading.Thread(target=self._read_loop)
        self.read_thread.daemon = True  # Daemon thread will exit when main program exits
        self.read_thread.start()
        self.logger.info("Started continuous reading of data")
    
    def _read_loop(self):
        try:
            while self.reading:
                if not self.is_connected():
                    if self.auto_reconnect:
                        self.logger.info("Connection lost, attempting to reconnect...")
                        if self.attempt_reconnect():
                            self.logger.info("Reconnected successfully!")
                        else:
                            self.logger.error("Failed to reconnect, will retry...")
                            time.sleep(2)
                            continue
                    else:
                        self.logger.error("Connection lost and auto-reconnect is disabled")
                        break
                
                messages = self.read_messages()
                for message in messages:
                    # Use the message handler to process the message
                    handler = self.message_handlers.get(message.WhichOneof("content"))
                    if handler:
                        handler(message)
                    
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
        except KeyboardInterrupt:
            self.logger.info("Stopping data reading...")
        finally:
            self.disconnect()

    def stop_reading(self):
        """
        Stop continuous reading of data
        """

        self.reading = False
        time.sleep(0.1)
        # Wait for the read thread to finish
        if self.read_thread:
            self.read_thread.join()

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.logger.info("Data reading stopped and connection closed")
        else:
            self.logger.info("No active connection to stop reading")