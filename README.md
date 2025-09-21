# STM32 4-Motor Controller - Professional Robotics Platform

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](CHANGELOG.md)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/)
[![STM32](https://img.shields.io/badge/STM32-F446ZE-red.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f446ze.html)

A professional-grade, production-ready 4-motor controller system for robotics applications with real-time tachometer feedback, non-blocking control algorithms, and seamless ROS 2 integration.

## Key Features

### Motor Control System
- **4 Independent Motors**: PWM control with direction pins and acceleration/deceleration profiles
- **Real-time Tachometer Feedback**: Interrupt-driven RPM measurement with noise filtering
- **Non-blocking Control**: 50Hz control loop with configurable acceleration/deceleration rates
- **Safety Features**: Emergency stop, duty cycle limiting, and comprehensive validation

### Communication & Integration
- **ROS 2 Integration**: Professional bridge with diagnostics, logging, and monitoring
- **Dual Protocol Support**: JSON commands for structured data, text commands for debugging
- **Robust Error Handling**: Automatic reconnection, timeout detection, and graceful degradation
- **Performance Monitoring**: Real-time diagnostics and system health reporting

### Professional Features
- **Comprehensive Documentation**: API reference, deployment guides, and troubleshooting
- **Configuration Management**: JSON-based configuration with command-line overrides
- **Production Logging**: Multi-level logging with file output and rotation support
- **Quality Assurance**: Input validation, range checking, and error recovery

## Hardware Requirements

| Component | Specification | Quantity |
|-----------|---------------|----------|
| **Microcontroller** | STM32F446ZE Nucleo Board | 1 |
| **Motors** | DC Motors with Encoders/Tachometers | 4 |
| **Motor Drivers** | PWM + Direction Compatible | 4 |
| **Power Supply** | 12-24V DC (Motor voltage rating) | 1 |
| **Connections** | Jumper wires, connectors | As needed |

## Pin Configuration

### Motor Control Pins
| Motor | PWM Pin | DIR Pin | SPEED Pin | Timer Features |
|-------|---------|---------|-----------|----------------|
| **Motor 1** | PA6 | PB5 | PA8 | TIM3_CH1, TIM1_CH1 |
| **Motor 2** | PA7 | PB10 | PB6 | TIM3_CH2, TIM4_CH1 |
| **Motor 3** | PA0 | PC7 | PC6 | TIM2_CH1, TIM8_CH1 |
| **Motor 4** | PA1 | PC8 | PC9 | TIM2_CH2, TIM8_CH4 |

### Pin Assignment Rationale
- **PWM Pins**: Selected for hardware timer support ensuring precise PWM generation
- **SPEED Pins**: Configured with interrupt capability for real-time tachometer processing  
- **DIR Pins**: Standard GPIO for reliable direction control
- **LED_BUILTIN**: System heartbeat and status indication

## Installation & Setup

### Prerequisites

#### Development Environment
```bash
# Install PlatformIO Core
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py

# Install ROS 2 Humble (Ubuntu 22.04)
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y ros-humble-desktop
```

#### Python Dependencies
```bash
# Install required packages
pip install pyserial

# Install ROS 2 message packages
sudo apt install -y ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-diagnostic-msgs
```

### Firmware Installation

1. **Clone or Download Project**
   ```bash
   git clone <repository-url>
   cd firmware
   ```

2. **Build Firmware**
   ```bash
   # Build the project
   platformio run
   
   # Upload to board (ensure STM32 is connected)
   platformio run --target upload
   ```

3. **Verify Installation**
   ```bash
   # Monitor serial output
   platformio device monitor --baud 115200
   ```

Expected output:
```
=====================================
   STM32-4Motor-Controller v2.0.0
=====================================
Hardware: STM32F446ZE Nucleo Board
Features: 4-Motor Control + Tachometers
Commands: JSON, Text (M1:50, STOP, etc.)
Status: Ready for operation
=====================================
SYSTEM_READY
```

## Usage Guide

### Basic Motor Control

#### Direct Serial Commands (115200 baud)
```bash
# Individual motor control (duty cycle: -100 to +100)
M1:50    # Motor 1 forward at 50%
M2:-30   # Motor 2 reverse at 30%
M3:75    # Motor 3 forward at 75%
M4:-25   # Motor 4 reverse at 25%

# System commands
STOP     # Emergency stop all motors
STATUS   # Request immediate status report
VERSION  # Display firmware version
```

#### JSON Commands (Structured Communication)
```json
{
  "cmd": "CMD_VEL",
  "data": {
    "linear_x": 0.2,
    "angular_z": 0.1
  }
}
```

### ROS 2 Integration

#### Start the Professional Bridge
```bash
# Basic usage
cd tools
python3 microros_bridge.py

# With custom configuration
python3 microros_bridge.py --port /dev/ttyACM0 --baud 115200 --log-level DEBUG

# Auto-detect serial port
python3 microros_bridge.py --port auto

# With configuration file
python3 microros_bridge.py --config bridge_config.json
```

#### ROS 2 Topic Interface

**Command Topics (Subscribers):**
```bash
# Differential drive control (motors 1&2)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{
  linear: {x: 0.2, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.1}
}'
```

**Status Topics (Publishers):**
```bash
# Monitor individual motor RPM
ros2 topic echo /wheel_rpm_left      # Motor 1 RPM
ros2 topic echo /wheel_rpm_right     # Motor 2 RPM  
ros2 topic echo /wheel_rpm_motor3    # Motor 3 RPM
ros2 topic echo /wheel_rpm_motor4    # Motor 4 RPM

# System diagnostics
ros2 topic echo /diagnostics
```

### Advanced Configuration

#### Create Bridge Configuration File
```json
{
  "serial_port": "/dev/ttyACM0",
  "baud_rate": 115200,
  "log_level": "INFO",
  "log_file": "/var/log/microros_bridge.log",
  "cmd_vel_topic": "cmd_vel",
  "rpm_topics": [
    "wheel_rpm_left",
    "wheel_rpm_right", 
    "wheel_rpm_motor3",
    "wheel_rpm_motor4"
  ],
  "max_command_rate": 50.0,
  "status_timeout": 2.0
}
```

## System Monitoring

### Status Message Format
The firmware continuously reports system status every 200ms:
```
STATUS,m1_target,m1_current,m1_rpm,m2_target,m2_current,m2_rpm,m3_target,m3_current,m3_rpm,m4_target,m4_current,m4_rpm
```

Example:
```
STATUS,50.0,49.8,245.6,-30.0,-29.9,-156.2,0.0,0.0,0.0,25.0,24.7,128.4
```

### Diagnostics and Health Monitoring

#### View System Diagnostics
```bash
# Real-time diagnostics
ros2 topic echo /diagnostics

# Bridge performance metrics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

#### Performance Metrics
- **Command Rate**: Commands sent per second
- **Status Age**: Time since last status update
- **Error Counters**: Parse errors, connection failures
- **Motor Health**: Individual motor status and RPM

## Configuration Parameters

### Firmware Configuration (main.cpp)

#### Control System Parameters
```cpp
namespace ControlConfig {
    constexpr uint32_t UPDATE_INTERVAL_MS = 20;    // 50Hz control loop
    constexpr float ACCELERATION_RATE = 2.0f;       // %/update acceleration
    constexpr float DECELERATION_RATE = 3.0f;       // %/update deceleration
}
```

#### Tachometer Parameters
```cpp
namespace TachometerConfig {
    constexpr uint16_t PULSES_PER_REVOLUTION = 1000; // Encoder resolution
    constexpr uint32_t SAMPLE_INTERVAL_MS = 100;     // RPM calculation rate
    constexpr uint32_t SIGNAL_TIMEOUT_MS = 2000;     // Stop detection timeout
}
```

#### Differential Drive Parameters
```cpp
namespace DriveConfig {
    constexpr float WHEEL_BASE_M = 0.3f;             // Wheel separation
    constexpr float WHEEL_RADIUS_M = 0.05f;          // Wheel radius
}
```

### Bridge Configuration

See [Configuration Management](#advanced-configuration) section for complete parameter reference.

## Project Structure

```
firmware/
├── src/
│   ├── main.cpp                     # Professional firmware implementation
│   └── main_old.cpp                 # Legacy firmware (backup)
├── tools/
│   ├── microros_bridge.py           # Professional ROS 2 bridge
│   ├── microros_bridge_old.py       # Legacy bridge (backup)
│   ├── speed_logger.py              # Direct serial logging utility
│   └── config/
│       └── bridge_config.json       # Default bridge configuration
├── include/
│   └── README                       # Include directory information
├── lib/
│   └── README                       # Library directory information
├── test/
│   └── README                       # Test directory information
├── docs/
│   ├── API_REFERENCE.md             # Detailed API documentation
│   ├── DEPLOYMENT.md                # Production deployment guide
│   └── TROUBLESHOOTING.md           # Common issues and solutions
├── platformio.ini                   # Build configuration
├── README.md                        # This file
├── CHANGELOG.md                     # Version history
└── LICENSE                          # MIT License
```

## 🔬 API Reference

### Firmware API

#### Motor Control Functions
```cpp
void setMotorTarget(uint8_t motorIndex, float targetDutyCycle);
void updateMotorControl(uint8_t motorIndex);
void emergencyStop();
```

#### Tachometer Functions  
```cpp
float calculateRPM(uint8_t motorIndex);
bool isValidRPM(float rpm);
```

#### Communication Functions
```cpp
void processCommand(const String& command);
void sendStatusReport();
```

### Bridge API

#### Core Classes
```python
class ProfessionalMicroROSBridge(Node):
    """Main bridge class with ROS 2 integration"""
    
class BridgeConfig:
    """Configuration data structure"""
    
class SystemDiagnostics:
    """Performance monitoring and metrics"""
```

#### Key Methods
```python
def _cmd_vel_callback(self, msg: Twist):
    """Handle velocity commands"""
    
def _process_status_message(self, message: str):
    """Parse and publish motor status"""
    
def _publish_diagnostics(self):
    """Publish system health metrics"""
```

## 🐛 Troubleshooting

### Common Issues

#### Connection Problems
```bash
# Check available serial ports
ls /dev/tty*

# Verify port permissions
sudo usermod -a -G dialout $USER
# Logout/login required after group change

# Test connection manually
screen /dev/ttyACM0 115200
```

#### Motor Control Issues
- **No movement**: Verify motor driver connections and power supply
- **Inconsistent RPM**: Check tachometer wiring and signal quality
- **Unexpected behavior**: Review duty cycle limits and motor direction

#### ROS 2 Integration Issues
```bash
# Check ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify message packages
ros2 interface show geometry_msgs/msg/Twist

# Test topic communication
ros2 topic list
ros2 node list
```

### Diagnostic Commands

#### Firmware Diagnostics
```bash
# Version information
echo "VERSION" > /dev/ttyACM0

# Force status update
echo "STATUS" > /dev/ttyACM0

# Emergency stop test
echo "STOP" > /dev/ttyACM0
```

#### Bridge Diagnostics
```bash
# Enable debug logging
python3 microros_bridge.py --log-level DEBUG

# Monitor diagnostics
ros2 topic echo /diagnostics

# Performance analysis
ros2 run rqt_plot rqt_plot /wheel_rpm_left/data
```

### Error Codes and Messages

| Error Type | Description | Solution |
|------------|-------------|----------|
| `JSON_PARSE_ERROR` | Invalid JSON format | Check command syntax |
| `COMMAND_BUFFER_OVERFLOW` | Command too long | Reduce command size |
| `UNKNOWN_*_COMMAND` | Unrecognized command | Verify command format |
| `EMERGENCY_STOP_ACTIVATED` | Safety stop triggered | Send new commands to resume |

## 🔄 Development Workflow

### Building and Testing

#### Firmware Development
```bash
# Clean build
platformio run --target clean
platformio run

# Upload and monitor
platformio run --target upload
platformio device monitor

# Format code (if clang-format available)
platformio run --target format
```

#### Bridge Development
```bash
# Install development dependencies
pip install pytest flake8 black

# Code formatting
black microros_bridge.py

# Linting
flake8 microros_bridge.py

# Testing
python3 -m pytest tests/
```

### Performance Optimization

#### Firmware Optimization
- Adjust `ControlConfig::UPDATE_INTERVAL_MS` for different control frequencies
- Tune `ACCELERATION_RATE` and `DECELERATION_RATE` for smooth motion
- Modify `TachometerConfig::PULSES_PER_REVOLUTION` for your encoders

#### Bridge Optimization
- Configure `max_command_rate` based on your application needs
- Adjust `json_buffer_size` for complex command structures
- Set appropriate `status_timeout` for your system responsiveness

## Deployment

### Production Deployment

#### System Service (systemd)
```bash
# Create service file: /etc/systemd/system/microros-bridge.service
[Unit]
Description=MicroROS Bridge for STM32 Controller
After=network.target

[Service]
Type=simple
User=robot
ExecStart=/usr/bin/python3 /opt/robot/microros_bridge.py --config /etc/robot/bridge_config.json
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target

# Enable and start service
sudo systemctl enable microros-bridge
sudo systemctl start microros-bridge
```

#### Docker Deployment
```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-serial \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Copy application
COPY tools/microros_bridge.py /app/
COPY tools/config/ /app/config/

WORKDIR /app
CMD ["python3", "microros_bridge.py", "--config", "config/bridge_config.json"]
```

### Monitoring and Maintenance

#### Log Management
```bash
# Rotate logs (add to crontab)
0 0 * * * /usr/bin/find /var/log/robot/ -name "*.log" -mtime +7 -delete

# Monitor log files
tail -f /var/log/microros_bridge.log
```

#### Health Monitoring
```bash
# ROS 2 diagnostics aggregator
ros2 run diagnostic_aggregator aggregator_node

# Custom monitoring script
#!/bin/bash
# Check bridge status every minute
* * * * * ros2 topic echo /diagnostics --once | grep -q "OK" || systemctl restart microros-bridge
```

## 📝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### Development Setup
1. Fork the repository
2. Create a feature branch
3. Make your changes with appropriate tests
4. Submit a pull request

### Code Standards
- Follow existing code style and conventions
- Add comprehensive documentation for new features
- Include unit tests for new functionality
- Update documentation as needed

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **STM32 Community**: For excellent hardware platform and tools
- **ROS 2 Team**: For robust robotics middleware
- **PlatformIO**: For streamlined embedded development
- **Contributors**: Everyone who helped improve this project

## Support

- **Documentation**: Complete guides in `/docs` directory
- **Issues**: Report bugs via GitHub Issues
- **Discussions**: Join community discussions
- **Email**: Contact maintainers for enterprise support

---

**STM32 4-Motor Controller v2.0.0** - Professional Robotics Platform  
Built with dedication for the robotics community