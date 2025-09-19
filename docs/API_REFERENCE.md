# API Reference - STM32 4-Motor Controller

Comprehensive API documentation for the STM32 4-Motor Controller firmware and Python bridge.

## Firmware API (C++)

### Core Configuration

#### MotorPins Namespace
Defines hardware pin assignments for all 4 motors.

```cpp
namespace MotorPins {
    // Motor 1 (Left Wheel)
    constexpr uint8_t MOTOR1_PWM = PA6;     // TIM3_CH1
    constexpr uint8_t MOTOR1_DIR = PB5;     // GPIO
    constexpr uint8_t MOTOR1_SPEED = PA8;   // TIM1_CH1 (Interrupt)
    
    // Motor 2-4 similarly defined...
}
```

#### ControlConfig Namespace
Control system parameters for motor behavior.

```cpp
namespace ControlConfig {
    constexpr uint32_t UPDATE_INTERVAL_MS = 20;    // 50Hz control loop
    constexpr float ACCELERATION_RATE = 2.0f;       // %/update acceleration
    constexpr float DECELERATION_RATE = 3.0f;       // %/update deceleration
    constexpr float MAX_DUTY_CYCLE = 100.0f;        // Maximum duty cycle
    constexpr float MIN_DUTY_CYCLE = -100.0f;       // Minimum duty cycle
}
```

### Data Structures

#### MotorState
Represents the control state of a single motor.

```cpp
struct MotorState {
    float targetDutyCycle;      // Target duty cycle (-100 to +100)
    float currentDutyCycle;     // Current duty cycle (-100 to +100)
    uint32_t lastUpdateTime;    // Last update timestamp (ms)
};
```

#### TachometerData
Contains tachometer measurement data.

```cpp
struct TachometerData {
    volatile uint32_t pulseCount;       // Total pulse count
    volatile uint32_t lastPulseMicros;  // Last pulse timestamp (μs)
    volatile uint32_t lastPeriodMicros; // Period between pulses (μs)
    uint32_t lastUpdateTime;            // Last RPM calculation time (ms)
    float currentRPM;                   // Current RPM reading
};
```

### Motor Control Functions

#### setMotorTarget()
Sets the target duty cycle for a specific motor.

```cpp
void setMotorTarget(uint8_t motorIndex, float targetDutyCycle);
```

**Parameters:**
- `motorIndex`: Motor index (0-3)
- `targetDutyCycle`: Target duty cycle (-100.0 to +100.0)

**Example:**
```cpp
setMotorTarget(0, 50.0f);  // Motor 1 forward at 50%
setMotorTarget(1, -30.0f); // Motor 2 reverse at 30%
```

#### updateMotorControl()
Updates motor control with acceleration/deceleration limiting.

```cpp
void updateMotorControl(uint8_t motorIndex);
```

**Parameters:**
- `motorIndex`: Motor index (0-3)

**Behavior:**
- Applies rate limiting based on `ACCELERATION_RATE` and `DECELERATION_RATE`
- Updates motor output only if sufficient time has elapsed
- Automatically called in main loop for all motors

#### emergencyStop()
Immediately stops all motors and resets targets.

```cpp
void emergencyStop();
```

**Effects:**
- Sets all motor targets to 0
- Immediately updates all motor outputs to 0
- Sends "EMERGENCY_STOP_ACTIVATED" message

### Tachometer Functions

#### calculateRPM()
Calculates RPM for a specific motor from tachometer data.

```cpp
float calculateRPM(uint8_t motorIndex);
```

**Parameters:**
- `motorIndex`: Motor index (0-3)

**Returns:**
- RPM value (float), 0.0 if invalid or stopped

**Algorithm:**
- Uses period-based calculation for accuracy at low speeds
- Applies timeout detection for stopped motors
- Validates RPM within reasonable bounds

### Communication Functions

#### processCommand()
Parses and executes incoming serial commands.

```cpp
void processCommand(const String& command);
```

**Supported Commands:**

**Text Commands:**
- `M1:50` - Set motor 1 to 50% duty cycle
- `M2:-30` - Set motor 2 to -30% duty cycle
- `M3:75` - Set motor 3 to 75% duty cycle
- `M4:-25` - Set motor 4 to -25% duty cycle
- `STOP` - Emergency stop all motors
- `STATUS` - Force immediate status update
- `VERSION` - Display firmware version

**JSON Commands:**
```json
{
  "cmd": "CMD_VEL",
  "data": {
    "linear_x": 0.2,
    "angular_z": 0.1
  }
}
```

#### sendStatusReport()
Sends system status over serial.

```cpp
void sendStatusReport();
```

**Output Format:**
```
STATUS,m1_target,m1_current,m1_rpm,m2_target,m2_current,m2_rpm,m3_target,m3_current,m3_rpm,m4_target,m4_current,m4_rpm
```

**Example:**
```
STATUS,50.0,49.8,245.6,-30.0,-29.9,-156.2,0.0,0.0,0.0,25.0,24.7,128.4
```

### Utility Functions

#### constrainFloat()
Constrains a float value between min and max bounds.

```cpp
inline float constrainFloat(float value, float minVal, float maxVal);
```

#### dutyCycleToPWM()
Converts duty cycle percentage to PWM value.

```cpp
inline uint8_t dutyCycleToPWM(float dutyCycle);
```

#### isValidRPM()
Validates RPM reading for sanity.

```cpp
inline bool isValidRPM(float rpm);
```

## Python Bridge API

### Core Classes

#### ProfessionalMicroROSBridge
Main bridge class providing ROS 2 integration.

```python
class ProfessionalMicroROSBridge(Node):
    def __init__(self, config: BridgeConfig)
```

**Key Methods:**

##### _cmd_vel_callback()
Handles incoming velocity commands from ROS 2.

```python
def _cmd_vel_callback(self, msg: Twist) -> None
```

**Parameters:**
- `msg`: ROS 2 Twist message with linear and angular velocities

**Behavior:**
- Converts Twist message to JSON command
- Queues command for serial transmission
- Applies differential drive kinematics

##### _process_status_message()
Parses STATUS messages from firmware.

```python
def _process_status_message(self, message: str) -> None
```

**Parameters:**
- `message`: STATUS message string from firmware

**Behavior:**
- Validates message format
- Extracts motor data (target, current, RPM)
- Publishes RPM data to ROS 2 topics
- Updates internal motor status

##### _publish_diagnostics()
Publishes system health metrics.

```python
def _publish_diagnostics(self) -> None
```

**Published Data:**
- Connection status
- Command/status rates
- Error counters
- Motor health metrics
- System uptime

#### BridgeConfig
Configuration data structure.

```python
@dataclass
class BridgeConfig:
    serial_port: str = "/dev/ttyACM0"
    baud_rate: int = 115200
    serial_timeout: float = 0.1
    reconnect_interval: float = 5.0
    
    cmd_vel_topic: str = "cmd_vel"
    rpm_topics: List[str] = None
    diagnostics_topic: str = "diagnostics"
    
    max_command_rate: float = 50.0
    max_status_age: float = 1.0
    
    log_level: str = "INFO"
    log_file: Optional[str] = None
```

#### SystemDiagnostics
Performance monitoring and metrics.

```python
@dataclass
class SystemDiagnostics:
    commands_sent: int = 0
    status_received: int = 0
    parse_errors: int = 0
    connection_errors: int = 0
    last_status_time: float = 0.0
    uptime_start: float = 0.0
    
    @property
    def uptime(self) -> float
    
    @property
    def command_rate(self) -> float
```

### Utility Functions

#### setup_logging()
Configures logging with appropriate handlers.

```python
def setup_logging(config: BridgeConfig) -> logging.Logger
```

#### find_serial_port()
Automatically detects STM32 serial port.

```python
def find_serial_port() -> Optional[str]
```

#### validate_motor_status()
Validates motor status data format.

```python
def validate_motor_status(data: List[str]) -> bool
```

### Configuration Management

#### Command Line Arguments
```bash
python3 microros_bridge.py [OPTIONS]

Options:
  --port, -p TEXT        Serial port (default: /dev/ttyACM0)
  --baud, -b INTEGER     Baud rate (default: 115200)
  --log-level, -l LEVEL  Logging level (DEBUG|INFO|WARNING|ERROR)
  --log-file, -f PATH    Log file path (optional)
  --config, -c PATH      Configuration file path (JSON)
  --version, -v          Show version information
```

#### Configuration File Format
```json
{
  "serial_port": "/dev/ttyACM0",
  "baud_rate": 115200,
  "log_level": "INFO",
  "log_file": "/var/log/microros_bridge.log",
  "max_command_rate": 50.0,
  "status_timeout": 2.0
}
```

## Error Handling

### Firmware Error Messages

| Message | Description | Recovery |
|---------|-------------|----------|
| `JSON_PARSE_ERROR` | Invalid JSON format | Check command syntax |
| `COMMAND_BUFFER_OVERFLOW` | Command too long | Reduce command size |
| `UNKNOWN_*_COMMAND` | Unrecognized command | Verify command format |
| `EMERGENCY_STOP_ACTIVATED` | Safety stop triggered | Send new commands to resume |

### Bridge Error Handling

#### Connection Recovery
- Automatic serial reconnection on failure
- Configurable reconnection interval
- Graceful degradation during outages

#### Data Validation
- Range checking for all numeric values
- Format validation for status messages
- Error counting and reporting

#### Performance Monitoring
- Command rate limiting
- Status timeout detection
- Comprehensive diagnostics

## Integration Examples

### ROS 2 Launch File
```xml
<launch>
  <node pkg="your_package" exec="microros_bridge.py" name="motor_bridge">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baud_rate" value="115200"/>
    <param name="log_level" value="INFO"/>
  </node>
</launch>
```

### Custom Motor Control
```python
import rclpy
from geometry_msgs.msg import Twist

def send_velocity_command():
    node = rclpy.create_node('motor_controller')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    
    msg = Twist()
    msg.linear.x = 0.5  # m/s forward
    msg.angular.z = 0.2  # rad/s turn
    
    pub.publish(msg)
```

### Direct Serial Control
```python
import serial
import json

def send_motor_command(port, motor, duty_cycle):
    ser = serial.Serial(port, 115200)
    command = f"M{motor}:{duty_cycle}\n"
    ser.write(command.encode())
    ser.close()

# Example usage
send_motor_command('/dev/ttyACM0', 1, 50)  # Motor 1 at 50%
```

This API reference provides comprehensive coverage of all available functions, classes, and interfaces for both the firmware and Python bridge components.