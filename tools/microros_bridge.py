#!/usr/bin/env python3
"""
Professional ROS 2 Bridge for STM32 4-Motor Controller

This module provides a robust, production-ready bridge between the STM32-based
4-motor controller and ROS 2 ecosystem with comprehensive error handling,
logging, and configuration management.

Features:
    - High-performance serial communication with automatic reconnection
    - Comprehensive ROS 2 integration with all 4 motor RPM publishing
    - Robust error handling and recovery mechanisms
    - Professional logging with configurable levels
    - JSON and CSV protocol support with validation
    - Graceful shutdown and resource cleanup
    - Performance monitoring and diagnostics

Author: STM32 Robotics Team
Version: 2.0.0
License: MIT
"""

import sys
import time
import json
import logging
import argparse
import threading
import traceback
from dataclasses import dataclass
from typing import Optional, Dict, Any, List, Tuple
from queue import Queue, Empty
from pathlib import Path

# ROS 2 imports with error handling
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
except ImportError as e:
    print(f"ERROR: Failed to import ROS 2 dependencies: {e}")
    print("Please install: sudo apt install ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-diagnostic-msgs")
    sys.exit(1)

# Serial communication imports
try:
    import serial
    from serial.tools import list_ports
except ImportError as e:
    print(f"ERROR: Failed to import pyserial: {e}")
    print("Please install: pip install pyserial")
    sys.exit(1)


# ============================================================================
# CONFIGURATION AND CONSTANTS
# ============================================================================

@dataclass
class BridgeConfig:
    """Configuration parameters for the ROS 2 bridge"""
    # Serial communication
    serial_port: str = "/dev/ttyACM0"
    baud_rate: int = 115200
    serial_timeout: float = 0.1
    reconnect_interval: float = 5.0
    
    # ROS 2 topics
    cmd_vel_topic: str = "cmd_vel"
    rpm_topics: List[str] = None
    diagnostics_topic: str = "diagnostics"
    
    # Communication protocols
    json_buffer_size: int = 256
    command_queue_maxsize: int = 100
    status_timeout: float = 2.0
    
    # Performance monitoring
    max_command_rate: float = 50.0  # Hz
    max_status_age: float = 1.0     # seconds
    
    # Logging
    log_level: str = "INFO"
    log_file: Optional[str] = None
    
    def __post_init__(self):
        if self.rpm_topics is None:
            self.rpm_topics = [
                "wheel_rpm_left",    # Motor 1
                "wheel_rpm_right",   # Motor 2  
                "wheel_rpm_motor3",  # Motor 3
                "wheel_rpm_motor4"   # Motor 4
            ]


@dataclass 
class MotorStatus:
    """Motor status data structure"""
    target_duty: float = 0.0
    current_duty: float = 0.0
    rpm: float = 0.0
    last_update: float = 0.0
    
    def is_stale(self, max_age: float) -> bool:
        """Check if the status data is stale"""
        return (time.time() - self.last_update) > max_age


@dataclass
class SystemDiagnostics:
    """System diagnostics and performance metrics"""
    commands_sent: int = 0
    status_received: int = 0
    parse_errors: int = 0
    connection_errors: int = 0
    last_status_time: float = 0.0
    uptime_start: float = 0.0
    
    def __post_init__(self):
        self.uptime_start = time.time()
    
    @property
    def uptime(self) -> float:
        """Get system uptime in seconds"""
        return time.time() - self.uptime_start
    
    @property
    def command_rate(self) -> float:
        """Calculate command rate in Hz"""
        if self.uptime > 0:
            return self.commands_sent / self.uptime
        return 0.0


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def setup_logging(config: BridgeConfig) -> logging.Logger:
    """Configure logging with appropriate handlers and formatters"""
    logger = logging.getLogger("microros_bridge")
    logger.setLevel(getattr(logging, config.log_level.upper()))
    
    # Clear any existing handlers
    logger.handlers.clear()
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # File handler (if specified)
    if config.log_file:
        file_handler = logging.FileHandler(config.log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger


def find_serial_port() -> Optional[str]:
    """Automatically detect STM32 serial port"""
    ports = list_ports.comports()
    
    # Look for STM32 Nucleo boards
    stm32_identifiers = ["STM32", "NUCLEO", "STMicroelectronics"]
    
    for port in ports:
        port_description = (port.description or "").upper()
        port_manufacturer = (port.manufacturer or "").upper()
        
        for identifier in stm32_identifiers:
            if identifier in port_description or identifier in port_manufacturer:
                return port.device
    
    # Fallback to common port names
    common_ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for port in common_ports:
        if Path(port).exists():
            return port
    
    return None


def validate_motor_status(data: List[str]) -> bool:
    """Validate motor status data format and ranges"""
    if len(data) < 13:  # STATUS + 12 values (4 motors * 3 values each)
        return False
    
    try:
        for i in range(1, len(data)):
            value = float(data[i])
            # Basic range checking
            if abs(value) > 10000:  # Reasonable upper bound
                return False
    except ValueError:
        return False
    
    return True


# ============================================================================
# MAIN BRIDGE CLASS
# ============================================================================

class ProfessionalMicroROSBridge(Node):
    """
    Professional ROS 2 bridge for STM32 4-motor controller
    
    This class provides a robust, production-ready interface between
    the STM32 controller and ROS 2 with comprehensive error handling,
    monitoring, and diagnostics.
    """
    
    def __init__(self, config: BridgeConfig):
        """Initialize the bridge with the given configuration"""
        super().__init__('microros_bridge_professional')
        
        self.config = config
        self.logger = self.get_logger()
        
        # State management
        self.running = True
        self.serial_connection: Optional[serial.Serial] = None
        self.motor_status = [MotorStatus() for _ in range(4)]
        self.diagnostics = SystemDiagnostics()
        
        # Communication queues
        self.command_queue = Queue(maxsize=config.command_queue_maxsize)
        
        # Threading
        self.serial_lock = threading.Lock()
        self.status_lock = threading.Lock()
        
        # Initialize ROS 2 interfaces
        self._setup_ros_interfaces()
        
        # Start worker threads
        self._start_worker_threads()
        
        # Initialize serial connection
        self._connect_serial()
        
        self.logger.info("Professional MicroROS Bridge initialized successfully")
    
    def _setup_ros_interfaces(self):
        """Setup ROS 2 publishers, subscribers, and services"""
        # QoS profiles for reliable communication
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Velocity command subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.config.cmd_vel_topic, 
            self._cmd_vel_callback, reliable_qos
        )
        
        # RPM publishers for each motor
        self.rpm_publishers = []
        for topic in self.config.rpm_topics:
            pub = self.create_publisher(Float32, topic, reliable_qos)
            self.rpm_publishers.append(pub)
        
        # Diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.config.diagnostics_topic, reliable_qos
        )
        
        # Periodic diagnostics timer
        self.diagnostics_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )
    
    def _start_worker_threads(self):
        """Start background worker threads"""
        # Serial transmission thread
        self.tx_thread = threading.Thread(
            target=self._serial_tx_worker, 
            name="SerialTX",
            daemon=True
        )
        self.tx_thread.start()
        
        # Serial reception thread
        self.rx_thread = threading.Thread(
            target=self._serial_rx_worker,
            name="SerialRX", 
            daemon=True
        )
        self.rx_thread.start()
        
        self.logger.info("Background worker threads started")
    
    def _connect_serial(self) -> bool:
        """Establish serial connection to the STM32 board"""
        try:
            with self.serial_lock:
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                
                port = self.config.serial_port
                if port == "auto":
                    port = find_serial_port()
                    if not port:
                        self.logger.error("Could not auto-detect serial port")
                        return False
                
                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=self.config.baud_rate,
                    timeout=self.config.serial_timeout,
                    exclusive=True
                )
                
                # Wait for connection to stabilize
                time.sleep(2.0)
                
                self.logger.info(f"Serial connection established: {port} @ {self.config.baud_rate}")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to connect to serial port: {e}")
            self.diagnostics.connection_errors += 1
            return False
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        try:
            command_data = {
                "cmd": "CMD_VEL",
                "data": {
                    "linear_x": float(msg.linear.x),
                    "angular_z": float(msg.angular.z)
                }
            }
            
            json_command = json.dumps(command_data)
            self._queue_command(json_command)
            
        except Exception as e:
            self.logger.error(f"Error processing cmd_vel: {e}")
    
    def _queue_command(self, command: str):
        """Queue a command for transmission"""
        try:
            if not self.command_queue.full():
                self.command_queue.put(command + '\n', block=False)
                self.diagnostics.commands_sent += 1
            else:
                self.logger.warning("Command queue full, dropping command")
        except Exception as e:
            self.logger.error(f"Error queuing command: {e}")
    
    def _serial_tx_worker(self):
        """Serial transmission worker thread"""
        self.logger.info("Serial TX worker started")
        
        while self.running:
            try:
                # Get command from queue with timeout
                try:
                    command = self.command_queue.get(timeout=0.1)
                except Empty:
                    continue
                
                # Send command if connection is available
                with self.serial_lock:
                    if self.serial_connection and self.serial_connection.is_open:
                        self.serial_connection.write(command.encode('utf-8'))
                        self.serial_connection.flush()
                
                self.command_queue.task_done()
                
            except Exception as e:
                self.logger.error(f"Serial TX error: {e}")
                self._handle_connection_error()
                time.sleep(1.0)
    
    def _serial_rx_worker(self):
        """Serial reception worker thread"""
        self.logger.info("Serial RX worker started")
        buffer = ""
        
        while self.running:
            try:
                # Check connection status
                with self.serial_lock:
                    if not self.serial_connection or not self.serial_connection.is_open:
                        time.sleep(0.5)
                        continue
                    
                    # Read available data
                    if self.serial_connection.in_waiting > 0:
                        data = self.serial_connection.read(
                            self.serial_connection.in_waiting
                        ).decode('utf-8', errors='ignore')
                        buffer += data
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self._process_serial_message(line)
                
                time.sleep(0.01)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                self.logger.error(f"Serial RX error: {e}")
                self._handle_connection_error()
                time.sleep(1.0)
    
    def _process_serial_message(self, message: str):
        """Process incoming serial messages"""
        try:
            # Try JSON format first
            if message.startswith('{'):
                self._process_json_message(message)
            
            # Process STATUS format
            elif message.startswith('STATUS,'):
                self._process_status_message(message)
            
            # Process other message types
            elif message.startswith('ERROR:'):
                self.logger.warning(f"Controller error: {message}")
            elif message.startswith('SYSTEM_READY'):
                self.logger.info("Controller system ready")
            else:
                self.logger.debug(f"Unhandled message: {message}")
                
        except Exception as e:
            self.logger.error(f"Error processing message '{message}': {e}")
            self.diagnostics.parse_errors += 1
    
    def _process_json_message(self, message: str):
        """Process JSON format messages"""
        try:
            data = json.loads(message)
            
            # Handle RPM data
            for i, topic in enumerate(self.config.rpm_topics):
                rpm_key = f"rpm_motor{i+1}"
                if rpm_key in data:
                    self._publish_rpm(i, float(data[rpm_key]))
                    
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e}")
            self.diagnostics.parse_errors += 1
    
    def _process_status_message(self, message: str):
        """Process STATUS format messages"""
        try:
            parts = message.split(',')
            
            if not validate_motor_status(parts):
                self.logger.warning(f"Invalid status message format: {message}")
                return
            
            current_time = time.time()
            
            # Parse motor data (STATUS + 3 values per motor)
            for motor_idx in range(4):
                base_idx = 1 + (motor_idx * 3)  # Skip 'STATUS' and get motor data
                
                if base_idx + 2 < len(parts):
                    with self.status_lock:
                        self.motor_status[motor_idx].target_duty = float(parts[base_idx])
                        self.motor_status[motor_idx].current_duty = float(parts[base_idx + 1])
                        self.motor_status[motor_idx].rpm = float(parts[base_idx + 2])
                        self.motor_status[motor_idx].last_update = current_time
                    
                    # Publish RPM data
                    self._publish_rpm(motor_idx, self.motor_status[motor_idx].rpm)
            
            self.diagnostics.status_received += 1
            self.diagnostics.last_status_time = current_time
            
        except (ValueError, IndexError) as e:
            self.logger.error(f"Error parsing status message: {e}")
            self.diagnostics.parse_errors += 1
    
    def _publish_rpm(self, motor_index: int, rpm_value: float):
        """Publish RPM value for specified motor"""
        if 0 <= motor_index < len(self.rpm_publishers):
            msg = Float32()
            msg.data = float(rpm_value)
            self.rpm_publishers[motor_index].publish(msg)
    
    def _handle_connection_error(self):
        """Handle serial connection errors with automatic reconnection"""
        self.logger.warning("Connection error detected, attempting reconnection...")
        self.diagnostics.connection_errors += 1
        
        # Close existing connection
        try:
            with self.serial_lock:
                if self.serial_connection:
                    self.serial_connection.close()
                    self.serial_connection = None
        except Exception:
            pass
        
        # Attempt reconnection
        time.sleep(self.config.reconnect_interval)
        self._connect_serial()
    
    def _publish_diagnostics(self):
        """Publish system diagnostics"""
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Bridge status
            bridge_status = DiagnosticStatus()
            bridge_status.name = "microros_bridge"
            bridge_status.hardware_id = "stm32_controller"
            
            # Determine overall status
            is_connected = (
                self.serial_connection is not None and 
                self.serial_connection.is_open
            )
            status_age = time.time() - self.diagnostics.last_status_time
            
            if not is_connected:
                bridge_status.level = DiagnosticStatus.ERROR
                bridge_status.message = "Serial connection lost"
            elif status_age > self.config.max_status_age:
                bridge_status.level = DiagnosticStatus.WARN
                bridge_status.message = f"Status data stale ({status_age:.1f}s)"
            else:
                bridge_status.level = DiagnosticStatus.OK
                bridge_status.message = "Operating normally"
            
            # Add diagnostic values
            bridge_status.values = [
                KeyValue(key="uptime", value=f"{self.diagnostics.uptime:.1f}"),
                KeyValue(key="commands_sent", value=str(self.diagnostics.commands_sent)),
                KeyValue(key="status_received", value=str(self.diagnostics.status_received)),
                KeyValue(key="parse_errors", value=str(self.diagnostics.parse_errors)),
                KeyValue(key="connection_errors", value=str(self.diagnostics.connection_errors)),
                KeyValue(key="command_rate", value=f"{self.diagnostics.command_rate:.1f}"),
                KeyValue(key="status_age", value=f"{status_age:.2f}"),
            ]
            
            # Add motor status
            with self.status_lock:
                for i, motor in enumerate(self.motor_status):
                    motor_age = time.time() - motor.last_update
                    bridge_status.values.extend([
                        KeyValue(key=f"motor{i+1}_rpm", value=f"{motor.rpm:.1f}"),
                        KeyValue(key=f"motor{i+1}_duty", value=f"{motor.current_duty:.1f}"),
                        KeyValue(key=f"motor{i+1}_age", value=f"{motor_age:.2f}"),
                    ])
            
            diag_array.status.append(bridge_status)
            self.diagnostics_pub.publish(diag_array)
            
        except Exception as e:
            self.logger.error(f"Error publishing diagnostics: {e}")
    
    def shutdown(self):
        """Gracefully shutdown the bridge"""
        self.logger.info("Shutting down bridge...")
        
        self.running = False
        
        # Close serial connection
        try:
            with self.serial_lock:
                if self.serial_connection:
                    self.serial_connection.close()
                    self.serial_connection = None
        except Exception as e:
            self.logger.error(f"Error closing serial connection: {e}")
        
        # Wait for threads to finish
        if hasattr(self, 'tx_thread'):
            self.tx_thread.join(timeout=2.0)
        if hasattr(self, 'rx_thread'):
            self.rx_thread.join(timeout=2.0)
        
        self.logger.info("Bridge shutdown complete")


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def parse_arguments() -> argparse.Namespace:
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Professional ROS 2 Bridge for STM32 4-Motor Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s --port /dev/ttyACM0 --baud 115200
    %(prog)s --port auto --log-level DEBUG
    %(prog)s --config bridge_config.json
        """
    )
    
    parser.add_argument(
        "--port", "-p", 
        default="/dev/ttyACM0",
        help="Serial port (default: /dev/ttyACM0, use 'auto' for detection)"
    )
    
    parser.add_argument(
        "--baud", "-b",
        type=int, default=115200,
        help="Baud rate (default: 115200)"
    )
    
    parser.add_argument(
        "--log-level", "-l",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level (default: INFO)"
    )
    
    parser.add_argument(
        "--log-file", "-f",
        help="Log file path (optional)"
    )
    
    parser.add_argument(
        "--config", "-c",
        help="Configuration file path (JSON format)"
    )
    
    parser.add_argument(
        "--version", "-v",
        action="version",
        version="%(prog)s 2.0.0"
    )
    
    return parser.parse_args()


def load_config(args: argparse.Namespace) -> BridgeConfig:
    """Load configuration from arguments and optional config file"""
    config = BridgeConfig()
    
    # Load from config file if specified
    if args.config:
        try:
            with open(args.config, 'r') as f:
                config_data = json.load(f)
            
            # Update config with file data
            for key, value in config_data.items():
                if hasattr(config, key):
                    setattr(config, key, value)
                    
        except Exception as e:
            print(f"Warning: Could not load config file {args.config}: {e}")
    
    # Override with command line arguments
    if hasattr(args, 'port'):
        config.serial_port = args.port
    if hasattr(args, 'baud'):
        config.baud_rate = args.baud
    if hasattr(args, 'log_level'):
        config.log_level = args.log_level
    if hasattr(args, 'log_file') and args.log_file:
        config.log_file = args.log_file
    
    return config


def main():
    """Main application entry point"""
    # Parse arguments and load configuration
    args = parse_arguments()
    config = load_config(args)
    
    # Setup logging
    logger = setup_logging(config)
    logger.info("Starting Professional MicroROS Bridge v2.0.0")
    
    # Initialize ROS 2
    rclpy.init()
    
    bridge = None
    try:
        # Create and run bridge
        bridge = ProfessionalMicroROSBridge(config)
        logger.info("Bridge started successfully")
        
        # Spin the ROS 2 node
        rclpy.spin(bridge)
        
    except KeyboardInterrupt:
        logger.info("Received shutdown signal")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        logger.error(traceback.format_exc())
    finally:
        # Cleanup
        if bridge:
            bridge.shutdown()
        
        try:
            rclpy.shutdown()
        except Exception:
            pass
        
        logger.info("Application terminated")


if __name__ == "__main__":
    main()