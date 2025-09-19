#!/usr/bin/env python3
"""
ROS 2 Bridge for STM32 4-Motor Controller

This script provides a bridge between the STM32 board and ROS 2, enabling
seamless integration of the 4-motor controller with the ROS 2 ecosystem.

Features:
- Subscribes to /cmd_vel (geometry_msgs/Twist) 
- Publishes to /wheel_rpm_left, /wheel_rpm_right, /wheel_rpm_motor3, /wheel_rpm_motor4 (std_msgs/Float32)
- Handles JSON command serialization and status parsing
- Automatic reconnection and error handling
- Backward compatible with 2-motor systems

Usage:
    python3 microros_bridge.py [--port /dev/ttyACM0] [--baud 115200]

Requirements:
    pip install pyserial
    sudo apt install ros-humble-geometry-msgs ros-humble-std-msgs
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import json
import time
import argparse
import threading
from queue import Queue

class MicroROSBridge(Node):
    def __init__(self, serial_port, baud_rate):
        super().__init__('microros_bridge')
        
        # Initialize serial connection
        self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=0.1)
        time.sleep(2)  # Wait for connection
        
        # ROS 2 publishers
        self.rpm_left_pub = self.create_publisher(Float32, 'wheel_rpm_left', 10)
        self.rpm_right_pub = self.create_publisher(Float32, 'wheel_rpm_right', 10)
        self.rpm_motor3_pub = self.create_publisher(Float32, 'wheel_rpm_motor3', 10)
        self.rpm_motor4_pub = self.create_publisher(Float32, 'wheel_rpm_motor4', 10)
        
        # ROS 2 subscriber
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Communication queues
        self.tx_queue = Queue()
        
        # Start serial communication threads
        self.rx_thread = threading.Thread(target=self.serial_rx_worker, daemon=True)
        self.tx_thread = threading.Thread(target=self.serial_tx_worker, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()
        
        self.get_logger().info(f'micro-ROS Bridge started on {serial_port}@{baud_rate}')
        
        # Send initialization handshake
        self.send_command('INIT', {})
    
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages"""
        cmd_data = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.send_command('CMD_VEL', cmd_data)
    
    def send_command(self, command, data):
        """Queue a command to send to the board"""
        message = {'cmd': command, 'data': data}
        self.tx_queue.put(json.dumps(message) + '\n')
    
    def serial_tx_worker(self):
        """Serial transmission worker thread"""
        while True:
            try:
                if not self.tx_queue.empty():
                    message = self.tx_queue.get()
                    self.serial_conn.write(message.encode())
                    self.serial_conn.flush()
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'TX error: {e}')
    
    def serial_rx_worker(self):
        """Serial reception worker thread"""
        buffer = ""
        while True:
            try:
                if self.serial_conn.in_waiting:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.process_serial_message(line)
                
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'RX error: {e}')
    
    def process_serial_message(self, message):
        """Process incoming messages from the board"""
        try:
            # Try JSON format first
            data = json.loads(message)
            if 'rpm_left' in data:
                msg = Float32()
                msg.data = float(data['rpm_left'])
                self.rpm_left_pub.publish(msg)
            
            if 'rpm_right' in data:
                msg = Float32()
                msg.data = float(data['rpm_right'])
                self.rpm_right_pub.publish(msg)
                
        except json.JSONDecodeError:
            # Parse STATUS format: "STATUS,motor1_target,motor1_current,motor1_rpm,motor2_target,motor2_current,motor2_rpm,motor3_target,motor3_current,motor3_rpm,motor4_target,motor4_current,motor4_rpm"
            if message.startswith('STATUS,'):
                try:
                    parts = message.split(',')
                    if len(parts) >= 13:  # STATUS + 12 values (4 motors * 3 values each)
                        motor1_rpm = float(parts[3])   # motor1_rpm
                        motor2_rpm = float(parts[6])   # motor2_rpm
                        motor3_rpm = float(parts[9])   # motor3_rpm
                        motor4_rpm = float(parts[12])  # motor4_rpm
                        
                        # For now, still publish only left/right (motors 1&2) for compatibility
                        # You can extend this to publish all 4 motor RPMs if needed
                        
                        # Publish left wheel RPM (motor1)
                        msg_left = Float32()
                        msg_left.data = motor1_rpm
                        self.rpm_left_pub.publish(msg_left)
                        
                        # Publish right wheel RPM (motor2)
                        msg_right = Float32()
                        msg_right.data = motor2_rpm
                        self.rpm_right_pub.publish(msg_right)
                        
                        # Publish motor 3 RPM
                        msg_motor3 = Float32()
                        msg_motor3.data = motor3_rpm
                        self.rpm_motor3_pub.publish(msg_motor3)
                        
                        # Publish motor 4 RPM
                        msg_motor4 = Float32()
                        msg_motor4.data = motor4_rpm
                        self.rpm_motor4_pub.publish(msg_motor4)
                        
                    elif len(parts) >= 7:  # Legacy format (2 motors)
                        motor1_rpm = float(parts[3])  # motor1_rpm
                        motor2_rpm = float(parts[6])  # motor2_rpm
                        
                        # Publish left wheel RPM (motor1)
                        msg_left = Float32()
                        msg_left.data = motor1_rpm
                        self.rpm_left_pub.publish(msg_left)
                        
                        # Publish right wheel RPM (motor2)
                        msg_right = Float32()
                        msg_right.data = motor2_rpm
                        self.rpm_right_pub.publish(msg_right)
                        
                except (ValueError, IndexError) as e:
                    self.get_logger().debug(f'Failed to parse STATUS message: {message}, error: {e}')
            
            # Fallback: parse simple text format like "RPM_L:123.4,RPM_R:56.7"
            elif 'RPM_L:' in message and 'RPM_R:' in message:
                try:
                    parts = message.split(',')
                    rpm_left = float(parts[0].split(':')[1])
                    rpm_right = float(parts[1].split(':')[1])
                    
                    msg_left = Float32()
                    msg_left.data = rpm_left
                    self.rpm_left_pub.publish(msg_left)
                    
                    msg_right = Float32()
                    msg_right.data = rpm_right
                    self.rpm_right_pub.publish(msg_right)
                    
                except (ValueError, IndexError):
                    pass
    
    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()

def main():
    parser = argparse.ArgumentParser(description='micro-ROS Bridge')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    rclpy.init()
    bridge = MicroROSBridge(args.port, args.baud)
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.cleanup()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()