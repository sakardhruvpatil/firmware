# Jetson Orin Nano Deployment Guide

## Quick Start Instructions

### 1. Deploy to Jetson
```bash
# On your development machine
chmod +x deploy_to_jetson.sh
./deploy_to_jetson.sh <jetson_ip_address>
```

### 2. Test Hardware System
```bash
# SSH to Jetson
ssh username@<jetson_ip_address>

# Run hardware test
cd ~/
python3 test_hardware_system.py
```

### 3. Launch Robot System
```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_workspace
source install/setup.bash

# Launch hardware system
ros2 launch ackermann_robot ackermann_robot_hardware.launch.py

# In another terminal, test with teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Hardware Connections

### STM32F446ZE
- **Connection**: USB cable to Jetson
- **Device**: `/dev/ttyACM0`
- **Baudrate**: 115200
- **Commands**:
  - `MOTOR <left_speed> <right_speed>` - Control motors (-100 to 100)
  - `STOP` - Emergency stop
  - `STATUS` - Get status

### Leadshine Servo Controller
- **Connection**: USB-to-RS485 adapter
- **Device**: `/dev/ttyUSB0`
- **Baudrate**: 9600
- **Protocol**: Modbus RTU
- **Slave ID**: 1

## System Architecture

```
teleop_twist_keyboard
        ↓ /cmd_vel
kinematics_node (converts twist to wheel speeds)
        ↓ /wheel_speeds
hardware_interface (communicates with STM32 & servo)
        ↓ Serial/Modbus
STM32 (motors) + Servo Controller (steering)
```

## Configuration Files

### Hardware Parameters (`config/hardware_params.yaml`)
- Vehicle dimensions and limits
- Hardware device paths and settings
- Control parameters and safety limits
- PID parameters for motor control

### Key Parameters to Adjust:
- `max_speed`: Maximum vehicle speed (m/s)
- `max_steering_angle`: Maximum steering angle (radians)
- `stm32_port`: STM32 device path
- `servo_port`: Servo controller device path

## Troubleshooting

### Common Issues

1. **Device not found**
   ```bash
   # Check connected devices
   ls /dev/tty*
   dmesg | tail  # Check for connection messages
   ```

2. **Permission denied**
   ```bash
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   # Logout and login again
   ```

3. **ROS 2 not found**
   ```bash
   # Source ROS 2
   source /opt/ros/humble/setup.bash
   ```

4. **Package not found**
   ```bash
   # Build workspace
   cd ~/ros2_workspace
   colcon build
   source install/setup.bash
   ```

### Testing Individual Components

1. **Test STM32 Communication**
   ```bash
   # Direct serial test
   python3 -c "
   import serial
   ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
   ser.write(b'STATUS\n')
   print(ser.read_all())
   ser.close()
   "
   ```

2. **Test Servo Communication**
   ```bash
   # Install modbus tools
   sudo apt install python3-pip
   pip3 install pymodbus

   # Test with python
   python3 -c "
   from pymodbus.client.sync import ModbusSerialClient
   client = ModbusSerialClient(method='rtu', port='/dev/ttyUSB0', baudrate=9600)
   print('Connected:', client.connect())
   client.close()
   "
   ```

3. **Test ROS 2 Nodes**
   ```bash
   # List available nodes
   ros2 node list

   # Check topics
   ros2 topic list

   # Monitor wheel speeds
   ros2 topic echo /wheel_speeds
   ```

## System Monitoring

### Performance Monitoring
```bash
# CPU usage
htop

# GPU usage (if using GPU acceleration)
nvidia-smi

# ROS 2 node performance
ros2 run rqt_graph rqt_graph
```

### Debug Output
- All nodes output to screen with emulate_tty for colored output
- Use `ros2 log` commands to check node logs
- Hardware interface publishes status messages for monitoring

## Safety Features

1. **Emergency Stop**: `STOP` command immediately halts all motors
2. **Command Timeout**: Stops if no commands received for 2 seconds
3. **Speed Limiting**: Configurable maximum safe speeds
4. **Deadband Control**: Ignores small inadvertent commands

## Next Steps

1. **Sensor Integration**: Add encoders, IMU, cameras
2. **Navigation**: Integrate with Nav2 stack
3. **Autonomous Operation**: Add mapping and path planning
4. **Remote Monitoring**: Set up remote visualization and control