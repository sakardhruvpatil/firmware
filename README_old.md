# 4-Motor Controller for STM32F446ZE

A complete ROS 2 4-motor robot controller for STM32F446ZE Nucleo board with individual motor control, tachometer feedback, and Python bridge integration.

## Hardware Requirements

- **Board**: STM32F446ZE Nucleo
- **Motors**: 4x DC motors with encoders/tachometers
- **Motor Drivers**: Compatible with PWM + Direction control

## Pin Configuration

| Function | Pin | Timer/Feature |
|----------|-----|---------------|
| Motor 1 PWM | PA6 | TIM3_CH1 |
| Motor 1 DIR | PB5 | GPIO |
| Motor 1 SPEED | PA8 | TIM1_CH1 (interrupt) |
| Motor 2 PWM | PA7 | TIM3_CH2 |
| Motor 2 DIR | PB10 | GPIO |
| Motor 2 SPEED | PB6 | TIM4_CH1 (interrupt) |
| Motor 3 PWM | PA0 | TIM2_CH1 |
| Motor 3 DIR | PC7 | GPIO |
| Motor 3 SPEED | PC6 | TIM8_CH1 (interrupt) |
| Motor 4 PWM | PA1 | TIM2_CH2 |
| Motor 4 DIR | PC8 | GPIO |
| Motor 4 SPEED | PC9 | TIM8_CH4 (interrupt) |

## Features

- **4-Motor Control**: Independent PWM control with acceleration/deceleration profiles
- **Tachometer Feedback**: Real-time RPM measurement with noise filtering for all motors
- **Differential Drive**: Converts linear/angular velocity to wheel speeds (motors 1&2)
- **Individual Control**: Direct control of motors 3&4 via M3:/M4: commands
- **ROS 2 Integration**: Python bridge for seamless ROS 2 communication
- **JSON Interface**: Structured command processing
- **Non-blocking**: Interrupt-driven design for real-time performance

## Building and Uploading

```bash
# Build the project
platformio run

# Upload to board
platformio run --target upload

# Monitor serial output (optional)
platformio device monitor
```

## ROS 2 Usage

### Start the bridge:
```bash
cd tools
python3 microros_bridge.py
```

### Control the robot:
```bash
# Send velocity commands (controls motors 1&2 for differential drive)
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'

# Monitor wheel RPM for all motors
ros2 topic echo /wheel_rpm_left     # Motor 1
ros2 topic echo /wheel_rpm_right    # Motor 2  
ros2 topic echo /wheel_rpm_motor3   # Motor 3
ros2 topic echo /wheel_rpm_motor4   # Motor 4
```

## Direct Serial Control

You can also control the motors directly via serial commands:

### JSON Commands:
```json
{"cmd": "CMD_VEL", "data": {"linear_x": 0.1, "angular_z": 0.0}}
```

### Text Commands:
```
M1:50    # Set motor 1 to 50% duty cycle
M2:-30   # Set motor 2 to -30% duty cycle  
M3:25    # Set motor 3 to 25% duty cycle
M4:-10   # Set motor 4 to -10% duty cycle
STOP     # Stop all motors
STATUS   # Request immediate status update
```

## Status Messages

The firmware continuously sends status updates every 200ms:
```
STATUS,motor1_target,motor1_current,motor1_rpm,motor2_target,motor2_current,motor2_rpm,motor3_target,motor3_current,motor3_rpm,motor4_target,motor4_current,motor4_rpm
```

## Robot Configuration

Default differential drive parameters for motors 1&2 (configurable in code):
- **Wheel Base**: 0.3m (distance between wheels)
- **Wheel Radius**: 0.05m
- **Pulses per Revolution**: 1000 (adjust for your encoders)

Motors 3&4 can be controlled independently for additional mechanisms (arms, conveyors, etc.)

## Files Structure

```
firmware/
├── src/main.cpp              # Main firmware code
├── tools/
│   ├── microros_bridge.py    # ROS 2 bridge script  
│   └── speed_logger.py       # Data logging utility
├── platformio.ini            # Build configuration
└── README.md                 # This file
```

## Troubleshooting

1. **No motor movement**: Check motor driver connections and power supply
2. **Inconsistent RPM readings**: Verify tachometer wiring and signal quality
3. **Bridge connection issues**: Ensure correct serial port and no other applications using it
4. **Build errors**: Run `platformio lib update` to refresh dependencies

## Dependencies

- **PlatformIO**: Build system and framework
- **ArduinoJson**: JSON parsing library
- **ROS 2 Humble**: Robot Operating System (host)
- **pyserial**: Python serial communication (bridge)