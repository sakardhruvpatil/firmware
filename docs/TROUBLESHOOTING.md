# Troubleshooting Guide - STM32 4-Motor Controller

Comprehensive troubleshooting guide for common issues and their solutions.

## Table of Contents

1. [Hardware Issues](#hardware-issues)
2. [Firmware Issues](#firmware-issues)
3. [Communication Problems](#communication-problems)
4. [ROS 2 Integration Issues](#ros-2-integration-issues)
5. [Performance Problems](#performance-problems)
6. [Development Issues](#development-issues)
7. [Diagnostic Tools](#diagnostic-tools)
8. [Frequently Asked Questions](#frequently-asked-questions)

## Hardware Issues

### Motor Not Moving

**Symptoms:**
- Motors receive commands but don't move
- Status shows duty cycle but no RPM
- No mechanical response

**Possible Causes & Solutions:**

1. **Power Supply Issues**
   ```bash
   # Check power supply voltage
   # Ensure motor supply voltage matches motor rating
   # Verify adequate current capacity
   ```
   - Solution: Use appropriate power supply (12-24V, sufficient current)
   - Check: Measure voltage at motor terminals

2. **Motor Driver Connections**
   ```
   STM32 Pin -> Motor Driver Pin
   PA6 (PWM)  -> PWM Input
   PB5 (DIR)  -> Direction Input
   GND        -> Ground
   ```
   - Solution: Verify all connections are secure
   - Check: Continuity with multimeter

3. **Motor Driver Configuration**
   - Some drivers require enable pins to be pulled high
   - Check driver datasheet for proper wiring
   - Verify logic voltage compatibility (3.3V vs 5V)

4. **Motor Issues**
   - Test motor directly with power supply
   - Check for mechanical binding
   - Verify motor specifications

### Inconsistent RPM Readings

**Symptoms:**
- RPM values jump erratically
- RPM doesn't correlate with motor speed
- Sporadic tachometer signals

**Solutions:**

1. **Tachometer Wiring**
   ```
   Encoder Output -> STM32 Pin (with pullup)
   PA8 (Motor 1) -> Speed signal
   PB6 (Motor 2) -> Speed signal
   PC6 (Motor 3) -> Speed signal
   PC9 (Motor 4) -> Speed signal
   ```

2. **Signal Quality Issues**
   - Add 10kΩ pullup resistors if not using INPUT_PULLUP
   - Use shielded cables for encoder signals
   - Keep encoder wires away from motor power wires
   - Add 100nF capacitors near encoder inputs for noise filtering

3. **Encoder Configuration**
   ```cpp
   // Adjust pulses per revolution in firmware
   constexpr uint16_t PULSES_PER_REVOLUTION = 1000; // Match your encoder
   ```

4. **Mechanical Issues**
   - Ensure encoder disc/wheel is properly attached
   - Check for loose coupling between motor and encoder
   - Verify encoder alignment

### Overheating Issues

**Symptoms:**
- Motor drivers getting hot
- Thermal shutdown of drivers
- Reduced performance over time

**Solutions:**
1. **Heat Dissipation**
   - Add heatsinks to motor drivers
   - Improve ventilation
   - Reduce ambient temperature

2. **Current Limiting**
   ```cpp
   // Reduce maximum duty cycle if needed
   constexpr float MAX_DUTY_CYCLE = 80.0f; // Reduce from 100%
   ```

3. **PWM Frequency**
   - Higher PWM frequencies can increase heat
   - Check motor driver specifications

## Firmware Issues

### Compilation Errors

**Common Error: Missing Libraries**
```
Fatal error: ArduinoJson.h: No such file or directory
```

**Solution:**
```bash
# Update library dependencies
platformio lib update

# Or install manually
platformio lib install "bblanchon/ArduinoJson@^7.0.0"
```

**Common Error: Board Definition**
```
Error: Unknown board 'nucleo_f446ze'
```

**Solution:**
```bash
# Update platform
platformio platform update ststm32

# Check available boards
platformio boards nucleo
```

### Runtime Issues

**Symptoms:**
- Firmware uploads but doesn't respond
- Serial communication not working
- Unexpected behavior

**Debugging Steps:**

1. **Serial Connection**
   ```bash
   # Check if device is detected
   ls /dev/tty*
   
   # Monitor serial output
   platformio device monitor --baud 115200
   ```

2. **Firmware Reset**
   - Press reset button on Nucleo board
   - Check for startup banner:
   ```
   =====================================
      STM32-4Motor-Controller v2.0.0
   =====================================
   ```

3. **Memory Issues**
   ```bash
   # Check memory usage
   platformio run --verbose
   
   # Look for:
   # RAM:   [          ]   X.X% (used XXXX bytes from XXXXXX bytes)
   # Flash: [=         ]   X.X% (used XXXX bytes from XXXXXX bytes)
   ```

### Configuration Issues

**Wrong Pin Assignments**
```cpp
// Verify pin definitions match your hardware
namespace MotorPins {
    constexpr uint8_t MOTOR1_PWM = PA6;   // Check this matches your wiring
    constexpr uint8_t MOTOR1_DIR = PB5;   // Check this matches your wiring
    // ...
}
```

**Timer Conflicts**
- Some pins share timers
- Check STM32F446ZE pinout for conflicts
- Use alternative pins if needed

## Communication Problems

### Serial Port Issues

**Permission Denied**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

**Port Not Found**
```bash
# List available ports
ls /dev/tty*
platformio device list

# Check USB connection
lsusb | grep STM
dmesg | tail
```

**Port in Use**
```bash
# Find process using port
sudo lsof /dev/ttyACM0

# Kill process or close other applications
# Arduino IDE, other monitors, etc.
```

### Command Processing Issues

**Commands Not Recognized**
```bash
# Test basic commands
echo "VERSION" > /dev/ttyACM0
echo "STATUS" > /dev/ttyACM0
echo "M1:50" > /dev/ttyACM0
```

**JSON Parse Errors**
```json
// Ensure proper JSON format
{
  "cmd": "CMD_VEL",
  "data": {
    "linear_x": 0.1,
    "angular_z": 0.0
  }
}
```

**Buffer Overflow**
- Commands too long (>128 characters)
- Send shorter commands
- Check command format

## ROS 2 Integration Issues

### Bridge Startup Problems

**ROS 2 Not Found**
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check ROS 2 installation
ros2 --help
```

**Missing Dependencies**
```bash
# Install required packages
sudo apt install -y ros-humble-geometry-msgs ros-humble-std-msgs ros-humble-diagnostic-msgs

# Install Python dependencies
pip install pyserial
```

**Bridge Connection Issues**
```python
# Test bridge manually
python3 microros_bridge.py --port /dev/ttyACM0 --log-level DEBUG
```

### Topic Communication

**Topics Not Visible**
```bash
# Check if bridge is running
ros2 node list

# List available topics
ros2 topic list

# Check topic types
ros2 topic info /cmd_vel
ros2 topic info /wheel_rpm_left
```

**No Data on Topics**
```bash
# Test topic publishing
ros2 topic echo /wheel_rpm_left

# Send test command
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

### Diagnostics Issues

**No Diagnostic Data**
```bash
# Check diagnostics topic
ros2 topic echo /diagnostics

# Use diagnostic tools
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## Performance Problems

### Slow Response

**High Latency**
- Check control loop frequency
- Reduce status reporting rate
- Optimize command processing

```cpp
// Adjust timing parameters
namespace ControlConfig {
    constexpr uint32_t UPDATE_INTERVAL_MS = 10;  // Increase frequency
}

namespace CommConfig {
    constexpr uint32_t STATUS_INTERVAL_MS = 500; // Reduce reporting rate
}
```

### Resource Usage

**High CPU Usage**
```bash
# Monitor CPU usage
top -p $(pgrep -f microros_bridge)

# Reduce logging level
python3 microros_bridge.py --log-level WARNING
```

**Memory Issues**
```bash
# Monitor memory usage
ps aux | grep microros_bridge

# Check for memory leaks
valgrind python3 microros_bridge.py
```

## Development Issues

### PlatformIO Problems

**Core Update Issues**
```bash
# Update PlatformIO Core
pip install --upgrade platformio

# Clear cache
platformio system prune
```

**Library Conflicts**
```bash
# Clean build
platformio run --target clean

# Update libraries
platformio lib update

# Rebuild
platformio run
```

### Code Modification Issues

**Changes Not Taking Effect**
```bash
# Ensure clean rebuild
platformio run --target clean
platformio run --target upload

# Verify upload success
platformio device monitor
```

**Debugging**
```cpp
// Add debug prints
Serial.println("Debug: Motor target set to " + String(target));

// Use conditional compilation
#ifdef DEBUG
    Serial.println("Debug info");
#endif
```

## Diagnostic Tools

### Firmware Diagnostics

```bash
# Version information
echo "VERSION" | socat - /dev/ttyACM0,b115200

# Force status update
echo "STATUS" | socat - /dev/ttyACM0,b115200

# Emergency stop test
echo "STOP" | socat - /dev/ttyACM0,b115200
```

### Hardware Testing

```bash
# Test individual motors
echo "M1:50" | socat - /dev/ttyACM0,b115200   # Motor 1 forward
echo "M1:-50" | socat - /dev/ttyACM0,b115200  # Motor 1 reverse
echo "M1:0" | socat - /dev/ttyACM0,b115200    # Motor 1 stop
```

### ROS 2 Diagnostics

```bash
# Node graph
rqt_graph

# Topic monitor
rqt_topic

# Plot data
rqt_plot /wheel_rpm_left/data

# Diagnostics viewer
rqt_robot_monitor
```

### Performance Analysis

```bash
# Command rate analysis
ros2 topic hz /cmd_vel

# Status rate analysis
ros2 topic hz /wheel_rpm_left

# Latency measurement
ros2 topic delay /cmd_vel
```

## Frequently Asked Questions

### Q: Motors respond to commands but RPM is always zero
**A:** Check tachometer wiring and encoder configuration. Ensure `PULSES_PER_REVOLUTION` matches your encoder specification.

### Q: Bridge connects but no ROS 2 topics appear
**A:** Verify ROS 2 environment is sourced and required message packages are installed.

### Q: High CPU usage with the bridge
**A:** Reduce logging level, increase status intervals, or optimize command rate.

### Q: Commands work in terminal but not through ROS 2
**A:** Check JSON command format and ensure bridge is processing messages correctly.

### Q: Firmware uploads but board doesn't respond
**A:** Check serial port permissions, baud rate, and try pressing reset button.

### Q: Inconsistent motor behavior
**A:** Verify power supply stability, check for loose connections, and ensure proper motor driver configuration.

### Q: Build fails with timer conflicts
**A:** Some pins share timer resources. Check STM32F446ZE documentation and use alternative pins.

### Q: Bridge crashes during operation
**A:** Enable debug logging to identify crash cause. Check for serial port issues or message parsing errors.

## Getting Additional Help

1. **Enable Debug Logging**
   ```bash
   python3 microros_bridge.py --log-level DEBUG
   ```

2. **Check System Logs**
   ```bash
   dmesg | tail
   journalctl -u microros-bridge.service
   ```

3. **Create Issue Report**
   Include:
   - Hardware configuration
   - Firmware version
   - Error messages
   - Steps to reproduce
   - Log files

4. **Community Support**
   - GitHub Issues
   - ROS 2 Community Forums
   - STM32 Community

This troubleshooting guide covers the most common issues encountered during development and deployment. For additional support, please refer to the project documentation or community forums.