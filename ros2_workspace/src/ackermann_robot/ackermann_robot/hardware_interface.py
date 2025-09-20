#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import serial
import time
import math
import threading
from dataclasses import dataclass
from typing import Optional

try:
    from pymodbus.client.sync import ModbusSerialClient as ModbusClient
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False
    print("⚠️  pymodbus not installed. Install with: pip3 install pymodbus")


@dataclass
class STM32Command:
    """STM32 motor command structure"""
    motor1_rpm: float = 0.0  # Left front
    motor2_rpm: float = 0.0  # Left rear  
    motor3_rpm: float = 0.0  # Right front
    motor4_rpm: float = 0.0  # Right rear


class STM32Controller:
    """Hardware interface for STM32 motor controller"""
    
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """Connect to STM32 controller"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            time.sleep(2)  # Wait for connection
            self.connected = True
            print(f"✅ STM32 controller connected on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to STM32: {e}")
            self.connected = False
            return False
    
    def send_motor_command(self, cmd: STM32Command) -> bool:
        """Send motor command to STM32"""
        if not self.connected or not self.serial_conn:
            return False
            
        try:
            # Format: "M1:RPM1,M2:RPM2,M3:RPM3,M4:RPM4\n"
            command = f"M1:{cmd.motor1_rpm:.0f},M2:{cmd.motor2_rpm:.0f},M3:{cmd.motor3_rpm:.0f},M4:{cmd.motor4_rpm:.0f}\n"
            self.serial_conn.write(command.encode())
            self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"❌ Failed to send command to STM32: {e}")
            return False
    
    def read_status(self) -> Optional[str]:
        """Read status from STM32"""
        if not self.connected or not self.serial_conn:
            return None
            
        try:
            if self.serial_conn.in_waiting > 0:
                return self.serial_conn.readline().decode().strip()
        except Exception as e:
            print(f"❌ Failed to read from STM32: {e}")
        return None
    
    def emergency_stop(self) -> bool:
        """Send emergency stop command"""
        if not self.connected:
            return False
        try:
            self.serial_conn.write(b"STOP\n")
            self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"❌ Emergency stop failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from STM32"""
        if self.serial_conn:
            self.serial_conn.close()
        self.connected = False


class ServoController:
    """Hardware interface for Leadshine servo motors via Modbus"""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 38400):
        self.port = port
        self.baudrate = baudrate
        self.client: Optional[ModbusClient] = None
        self.connected = False
        self.left_servo_id = 2
        self.right_servo_id = 3
        
        if not MODBUS_AVAILABLE:
            print("❌ Modbus not available. Servo control disabled.")
            return
            
    def connect(self) -> bool:
        """Connect to servo controllers"""
        if not MODBUS_AVAILABLE:
            return False
            
        try:
            self.client = ModbusClient(
                method='rtu',
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                parity='N',
                stopbits=1,
                bytesize=8
            )
            self.connected = self.client.connect()
            if self.connected:
                print(f"✅ Servo controllers connected on {self.port}")
            else:
                print(f"❌ Failed to connect to servo controllers on {self.port}")
            return self.connected
        except Exception as e:
            print(f"❌ Servo connection failed: {e}")
            return False
    
    def set_position(self, left_angle: float, right_angle: float) -> bool:
        """Set steering positions (angles in degrees)"""
        if not self.connected or not self.client:
            return False
            
        try:
            # Convert angles to servo position units (adjust as needed)
            # Assuming 10000 units = 360 degrees for Leadshine servos
            left_position = int(left_angle * 10000 / 360.0)
            right_position = int(right_angle * 10000 / 360.0)
            
            # Send position commands via Modbus
            # Register 0x0C00 is typically position command for Leadshine
            success1 = self.client.write_register(0x0C00, left_position, unit=self.left_servo_id)
            success2 = self.client.write_register(0x0C00, right_position, unit=self.right_servo_id)
            
            return success1 and success2
        except Exception as e:
            print(f"❌ Failed to set servo positions: {e}")
            return False
    
    def read_positions(self) -> tuple[float, float]:
        """Read current servo positions"""
        if not self.connected or not self.client:
            return 0.0, 0.0
            
        try:
            # Read position feedback (register 0x0C01 typically)
            left_result = self.client.read_holding_registers(0x0C01, 1, unit=self.left_servo_id)
            right_result = self.client.read_holding_registers(0x0C01, 1, unit=self.right_servo_id)
            
            if left_result and right_result:
                left_angle = left_result.registers[0] * 360.0 / 10000.0
                right_angle = right_result.registers[0] * 360.0 / 10000.0
                return left_angle, right_angle
        except Exception as e:
            print(f"❌ Failed to read servo positions: {e}")
            
        return 0.0, 0.0
    
    def disconnect(self):
        """Disconnect from servo controllers"""
        if self.client:
            self.client.close()
        self.connected = False


class HardwareInterfaceNode(Node):
    """ROS 2 node for hardware interface on Jetson Orin Nano"""
    
    def __init__(self):
        super().__init__('hardware_interface')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('stm32_port', '/dev/ttyACM0'),
                ('stm32_baudrate', 115200),
                ('servo_port', '/dev/ttyUSB0'),
                ('servo_baudrate', 38400),
                ('update_rate', 50.0),
                ('wheelbase', 0.64),
                ('track_width', 0.6),
                ('wheel_radius', 0.1),
            ]
        )
        
        # Get parameters
        self.stm32_port = self.get_parameter('stm32_port').value
        self.stm32_baudrate = self.get_parameter('stm32_baudrate').value
        self.servo_port = self.get_parameter('servo_port').value
        self.servo_baudrate = self.get_parameter('servo_baudrate').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # Robot parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.track_width = self.get_parameter('track_width').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Initialize hardware controllers
        self.stm32 = STM32Controller(self.stm32_port, self.stm32_baudrate)
        self.servos = ServoController(self.servo_port, self.servo_baudrate)
        
        # ROS 2 interfaces
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray, '/wheel_commands', self.wheel_commands_callback, 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.joint_commands_callback, 10)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # State variables
        self.current_wheel_rpms = [0.0, 0.0, 0.0, 0.0]
        self.current_steering_angles = [0.0, 0.0]  # left, right in degrees
        
        # Control loop timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
        
        # Connect to hardware
        self.connect_hardware()
        
        self.get_logger().info('Hardware Interface Node started for Jetson Orin Nano')
        
    def connect_hardware(self):
        """Connect to all hardware components"""
        self.get_logger().info('🔌 Connecting to hardware...')
        
        # Connect STM32
        if self.stm32.connect():
            self.get_logger().info('✅ STM32 motor controller connected')
        else:
            self.get_logger().warn('❌ STM32 motor controller connection failed')
        
        # Connect servos
        if self.servos.connect():
            self.get_logger().info('✅ Servo controllers connected')
        else:
            self.get_logger().warn('❌ Servo controllers connection failed')
    
    def wheel_commands_callback(self, msg: Float64MultiArray):
        """Handle wheel velocity commands (rad/s to RPM)"""
        if len(msg.data) >= 4:
            # Convert rad/s to RPM
            self.current_wheel_rpms = [vel * 9.549 for vel in msg.data[:4]]
    
    def joint_commands_callback(self, msg: JointState):
        """Handle steering joint commands"""
        if len(msg.position) >= 2:
            # Expecting steering angles in radians, convert to degrees
            self.current_steering_angles = [
                math.degrees(msg.position[0]),  # Left steering
                math.degrees(msg.position[1])   # Right steering
            ]
    
    def control_loop(self):
        """Main control loop - send commands and read feedback"""
        # Send motor commands
        if self.stm32.connected:
            cmd = STM32Command(
                motor1_rpm=self.current_wheel_rpms[0],  # Left front
                motor2_rpm=self.current_wheel_rpms[1],  # Left rear
                motor3_rpm=self.current_wheel_rpms[2],  # Right front
                motor4_rpm=self.current_wheel_rpms[3]   # Right rear
            )
            self.stm32.send_motor_command(cmd)
        
        # Send servo commands
        if self.servos.connected:
            self.servos.set_position(
                self.current_steering_angles[0],
                self.current_steering_angles[1]
            )
        
        # Read feedback and publish joint states
        self.publish_joint_states()
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Read servo positions
        left_angle, right_angle = self.servos.read_positions()
        
        # Joint names and positions
        joint_state.name = [
            'left_front_wheel_joint',
            'left_rear_wheel_joint',
            'right_front_wheel_joint', 
            'right_rear_wheel_joint',
            'left_steering_joint',
            'right_steering_joint'
        ]
        
        joint_state.position = [
            0.0, 0.0, 0.0, 0.0,  # Wheel positions (placeholder)
            math.radians(left_angle),
            math.radians(right_angle)
        ]
        
        joint_state.velocity = [
            self.current_wheel_rpms[0] / 9.549,  # Convert RPM to rad/s
            self.current_wheel_rpms[1] / 9.549,
            self.current_wheel_rpms[2] / 9.549,
            self.current_wheel_rpms[3] / 9.549,
            0.0, 0.0  # Steering velocities
        ]
        
        self.joint_state_pub.publish(joint_state)
    
    def emergency_stop(self):
        """Emergency stop all motors"""
        self.get_logger().warn('🚨 EMERGENCY STOP ACTIVATED')
        self.stm32.emergency_stop()
        self.current_wheel_rpms = [0.0, 0.0, 0.0, 0.0]
        self.current_steering_angles = [0.0, 0.0]
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('🔌 Disconnecting from hardware...')
        self.emergency_stop()
        self.stm32.disconnect()
        self.servos.disconnect()


def main(args=None):
    rclpy.init(args=args)
    
    node = HardwareInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()