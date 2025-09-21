# PS4 Joystick Control Guide

## Quick Start

### 1. Connect PS4 Controller
```bash
# Put controller in pairing mode (Share + PS button until light flashes)
sudo bluetoothctl
scan on
# Find controller MAC (e.g., XX:XX:XX:XX:XX:XX)
pair XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
exit
```

### 2. Test Controller
```bash
cd ~/ros2_workspace
python3 setup_ps4_controller.py
```

### 3. Launch Robot with Joystick
```bash
# Option 1: Use updated hardware launch file
source install/setup.bash
ros2 launch ackermann_robot ackermann_robot_hardware.launch.py use_joystick:=true

# Option 2: Use dedicated joystick launch file
ros2 launch ackermann_robot ackermann_robot_joystick.launch.py
```

## Controls

### Movement
- **Left Stick Vertical**: Forward/Backward movement
- **Left Stick Horizontal**: Steering (turning)

### Safety
- **L1 Button**: Enable movement (MUST HOLD for robot to move)
- **R1 Button**: Turbo mode (higher speeds)

### Button Layout
```
     Triangle
  Square   Circle
       X

L1  L2         R2  R1
    Share  PS  Options
      LS      RS
```

## Speed Settings

### Normal Mode (L1 only)
- Linear speed: 1.0 m/s max
- Angular speed: 1.5 rad/s max

### Turbo Mode (L1 + R1)
- Linear speed: 2.0 m/s max  
- Angular speed: 3.0 rad/s max

## Troubleshooting

### Controller Not Found
```bash
# Check if device exists
ls /dev/input/js*

# Test joystick directly
jstest /dev/input/js0
```

### ROS2 Not Receiving Commands
```bash
# Check joy messages
ros2 topic echo /joy

# Check cmd_vel output  
ros2 topic echo /cmd_vel

# List active nodes
ros2 node list
```

### Reconnecting Controller
```bash
# If controller disconnects
sudo bluetoothctl
connect XX:XX:XX:XX:XX:XX
```

## Advanced Configuration

Edit `/config/ps4_joy_config.yaml` to customize:
- Button mappings
- Speed scaling
- Deadzone settings
- Enable button assignment

## Launch Options

```bash
# Custom joystick device
ros2 launch ackermann_robot ackermann_robot_joystick.launch.py joy_device:=/dev/input/js1

# Custom config file
ros2 launch ackermann_robot ackermann_robot_joystick.launch.py joy_config_file:=/path/to/custom_config.yaml

# Disable joystick in hardware launch
ros2 launch ackermann_robot ackermann_robot_hardware.launch.py use_joystick:=false
```