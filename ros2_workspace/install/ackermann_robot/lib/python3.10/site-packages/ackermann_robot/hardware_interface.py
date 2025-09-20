#!/usr/bin/env python3
"""
Professional Ackermann Robot Hardware Interface

This module provides the hardware abstraction layer for a 4-wheel independent drive
Ackermann robot with dedicated steering motors. It interfaces between ROS2 and the
physical hardware components.

Author: Ackermann Robot Team
License: MIT
Version: 1.0.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial
import time
import math
from dataclasses import dataclass
from typing import Optional, List
import logging

# Import the MotorController class for Ackermann steering motors
from .motor_driver import MotorController


@dataclass
class STM32Command:
    """STM32 motor command structure for 4-wheel drive system"""
    motor1_rpm: float = 0.0  # Left front
    motor2_rpm: float = 0.0  # Left rear  
    motor3_rpm: float = 0.0  # Right front
    motor4_rpm: float = 0.0  # Right rear


class STM32Controller:
    """
    Professional STM32 motor controller interface
    
    Manages communication with STM32 microcontroller for 4-wheel drive control.
    Handles serial communication, command formatting, and connection management.
    """
    
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        """
        Initialize STM32 controller
        
        Args:
            port: Serial port path for STM32 communication
            baudrate: Communication baudrate (default: 115200)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self._logger = logging.getLogger(__name__)
        
    def connect(self) -> bool:
        """
        Establish connection to STM32 controller
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self._logger.info(f"Connecting to STM32 on {self.port}")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            time.sleep(2)  # Allow STM32 to reset
            self.connected = True
            self._logger.info(f"STM32 connected successfully on {self.port}")
            return True
        except Exception as e:
            self._logger.error(f"STM32 connection failed: {e}")
            return False
    
    def disconnect(self) -> None:
        """Safely disconnect from STM32 controller"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except Exception as e:
                self._logger.warning(f"Error during STM32 disconnect: {e}")
        self.serial = None
        self.connected = False
        self._logger.info("STM32 disconnected")
    
    def send_motor_commands(self, rpm_commands: List[float]) -> bool:
        """
        Send RPM commands to 4 motors
        
        Args:
            rpm_commands: List of RPM values [LF, LR, RF, RR]
            
        Returns:
            bool: True if commands sent successfully, False otherwise
        """
        if not self.connected or not self.serial:
            self._logger.warning("STM32 not connected, cannot send commands")
            return False
            
        try:
            # Format: "M1:{motor1_rpm},M2:{motor2_rpm},M3:{motor3_rpm},M4:{motor4_rpm}\n"
            command = f"M1:{rpm_commands[0]:.1f},M2:{rpm_commands[1]:.1f},M3:{rpm_commands[2]:.1f},M4:{rpm_commands[3]:.1f}\n"
            self.serial.write(command.encode())
            self.serial.flush()
            return True
        except Exception as e:
            self._logger.error(f"Failed to send motor commands: {e}")
            return False


class AckermannSteeringController:
    """
    Professional Ackermann steering controller using MotorController class
    
    Manages dedicated steering motors for precise Ackermann geometry control.
    Supports dual motor setup with independent left and right steering control.
    """
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 38400):
        """
        Initialize Ackermann steering controller
        
        Args:
            port: Serial port path for steering motor communication
            baudrate: Communication baudrate (default: 38400)
        """
        self.port = port
        self.baudrate = baudrate
        self.connected = False
        self.shared_serial = None
        self.left_motor = None   # Slave ID 2
        self.right_motor = None  # Slave ID 3
        self._logger = logging.getLogger(__name__)
        
        # Safety limits
        self.max_steering_angle = math.radians(26.0)  # ±26° limit
        
    def connect(self) -> bool:
        """Connect to steering motors"""
        try:
            print(f"Connecting to Ackermann steering motors on {self.port}...")
            
            # Create shared serial connection
            self.shared_serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate, 
                timeout=0.1
            )
            
            # Create motor controllers for left and right steering
            self.left_motor = MotorController(self.shared_serial, slave_id=2)   # Left steering
            self.right_motor = MotorController(self.shared_serial, slave_id=3)  # Right steering
            
            # Set both motors to position control mode
            self.left_motor.set_mode_position_control()
            self.right_motor.set_mode_position_control()
            
            self.connected = True
            print(f"Ackermann steering motors connected on {self.port}")
            return True
            
        except Exception as e:
            print(f"Steering motor connection failed: {e}")
            self.disconnect()
            return False
    
    def disconnect(self):
        """Disconnect from steering motors"""
        if self.left_motor:
            try:
                self.left_motor.stop_motion()
            except:
                pass
        if self.right_motor:
            try:
                self.right_motor.stop_motion()
            except:
                pass
                
        if self.shared_serial and self.shared_serial.is_open:
            try:
                self.shared_serial.close()
            except:
                pass
                
        self.left_motor = None
        self.right_motor = None
        self.shared_serial = None
        self.connected = False
        print("Ackermann steering motors disconnected")
    
    def set_steering_angles(self, theta_L_rad: float, theta_R_rad: float):
        """
        Set steering disk angles for Ackermann geometry
        
        Args:
            theta_L_rad: Left disk angle in radians
            theta_R_rad: Right disk angle in radians
        """
        if not self.connected:
            return False
            
        try:
            # Convert radians to degrees
            theta_L_deg = math.degrees(theta_L_rad)
            theta_R_deg = math.degrees(theta_R_rad)
            
            # Clamp to ±26° safety limits
            theta_L_deg = max(-26.0, min(26.0, theta_L_deg))
            theta_R_deg = max(-26.0, min(26.0, theta_R_deg))
            
            # Send commands to steering motors
            if self.left_motor:
                self.left_motor.set_angle_degrees(theta_L_deg)
            if self.right_motor:
                self.right_motor.set_angle_degrees(theta_R_deg)
                
            return True
            
        except Exception as e:
            print(f"Failed to set steering angles: {e}")
            return False


class HardwareInterface(Node):
    """
    Professional Ackermann Robot Hardware Interface Node
    
    Main ROS2 node that coordinates between kinematics commands and physical hardware.
    Manages 4-wheel drive motors via STM32 and steering motors via Modbus.
    
    Features:
    - Real-time wheel velocity control (RPM)
    - Ackermann steering angle control (radians)
    - Hardware safety monitoring
    - Professional error handling and logging
    """
    
    # Hardware configuration constants
    WHEEL_RADIUS = 0.075  # 75mm wheels in meters
    CONTROL_FREQUENCY = 50.0  # 50Hz control loop
    MAX_STEERING_ANGLE = math.radians(26.0)  # ±26° steering limit
    
    def __init__(self):
        """Initialize the hardware interface node"""
        super().__init__('hardware_interface')
        
        self.get_logger().info("Initializing Professional Ackermann Robot Hardware Interface")
        
        # Hardware controllers
        self.stm32_controller = STM32Controller(port="/dev/ttyACM0", baudrate=115200)
        self.steering_controller = AckermannSteeringController(port="/dev/ttyUSB0", baudrate=38400)
        
        # Initialize hardware connections
        if not self.stm32_controller.connect():
            self.get_logger().warn("Failed to connect to STM32 controller")
        if not self.steering_controller.connect():
            self.get_logger().warn("Failed to connect to steering controller")
        
        # ROS2 interface setup
        self.wheel_commands_sub = self.create_subscription(
            Float64MultiArray,
            'wheel_commands',
            self.wheel_commands_callback,
            10
        )
        
        # Control state variables
        self.current_wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # RPM
        self.current_steering_angles = [0.0, 0.0]  # radians
        
        # Control loop timer
        control_period = 1.0 / self.CONTROL_FREQUENCY
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info("Ackermann Hardware Interface initialized successfully")
    
    def wheel_commands_callback(self, msg):
        """
        Process wheel commands from kinematics node
        Format: [v_LF, v_LR, v_RF, v_RR, θ_L, θ_R]
        """
        if len(msg.data) < 6:
            self.get_logger().warn(f"Invalid wheel command length: {len(msg.data)}")
            return
        
        # Extract wheel velocities (m/s) and steering angles (rad)
        v_LF, v_LR, v_RF, v_RR, theta_L, theta_R = msg.data[:6]
        
        # Convert wheel velocities from m/s to RPM
        # RPM = (m/s) / (2π × wheel_radius) × 60
        wheel_radius = 0.075  # 75mm wheels
        conversion_factor = 60.0 / (2.0 * math.pi * wheel_radius)
        
        # Convert to RPM and store
        self.current_wheel_velocities = [
            v_LF * conversion_factor,  # Left Front RPM
            v_LR * conversion_factor,  # Left Rear RPM  
            v_RF * conversion_factor,  # Right Front RPM
            v_RR * conversion_factor   # Right Rear RPM
        ]
        
        # Store steering angles (already in radians)
        self.current_steering_angles = [theta_L, theta_R]
        
        # Optional debug output (uncomment for debugging)
        # self.get_logger().debug(
        #     f"Commands: Wheels=[{v_LF:.3f}, {v_LR:.3f}, {v_RF:.3f}, {v_RR:.3f}] m/s, "
        #     f"RPM=[{self.current_wheel_velocities[0]:.1f}, {self.current_wheel_velocities[1]:.1f}, "
        #     f"{self.current_wheel_velocities[2]:.1f}, {self.current_wheel_velocities[3]:.1f}], "
        #     f"Steering=[{math.degrees(theta_L):.1f}°, {math.degrees(theta_R):.1f}°]"
        # )
    
    def control_loop(self):
        """Main control loop - sends commands to hardware"""
        try:
            # Send wheel velocities to STM32 (4-wheel drive)
            if self.stm32_controller.connected:
                rpm_commands = self.current_wheel_velocities
                self.stm32_controller.send_motor_commands(rpm_commands)
            
            # Send steering angles to Ackermann steering motors
            if self.steering_controller.connected:
                theta_L, theta_R = self.current_steering_angles
                self.steering_controller.set_steering_angles(theta_L, theta_R)
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
    
    def shutdown(self):
        """Clean shutdown of hardware connections"""
        self.get_logger().info("Shutting down hardware interface...")
        
        # Stop all motors
        if self.stm32_controller.connected:
            self.stm32_controller.send_motor_commands([0, 0, 0, 0])
        if self.steering_controller.connected:
            self.steering_controller.set_steering_angles(0.0, 0.0)
        
        # Disconnect hardware
        self.stm32_controller.disconnect()
        self.steering_controller.disconnect()
        
        self.get_logger().info("Hardware interface shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    
    node = HardwareInterface()
    
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