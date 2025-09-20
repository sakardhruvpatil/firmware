/**
 * @file main.cpp
 * @brief Professional 4-Motor Controller with Tachometer Feedback and ROS 2 Integration
 * @version 2.0.0
 * @date 2025-09-19
 * @author STM32 Robotics Team
 * 
 * @details
 * This firmware provides a complete 4-motor control system for robotics applications
 * with real-time tachometer feedback, non-blocking control algorithms, and seamless
 * ROS 2 integration through a Python bridge interface.
 * 
  * Features:
 * - 4-channel motor control with PWM speed and direction control
 * - Tachometer feedback for each motor (RPM measurement)
 * - Acceleration/deceleration limiting for smooth operation
 * - Text command interface for motor control
 * - Serial status reporting with real-time motor data
 * - Emergency stop functionality
 * 
 * Hardware Platform: STM32F446ZE Nucleo Board
 * Framework: Arduino with PlatformIO
 * 
 * Pin Configuration:
 * ┌────────┬──────┬──────┬─────────┬─────────────────┐
 * │ Motor  │ PWM  │ DIR  │ SPEED   │ Timer/Feature   │
 * ├────────┼──────┼──────┼─────────┼─────────────────┤
 * │ Motor1 │ PA6  │ PB5  │ PA8     │ TIM3_CH1/TIM1   │
 * │ Motor2 │ PA7  │ PB10 │ PB6     │ TIM3_CH2/TIM4   │
 * │ Motor3 │ PA0  │ PC7  │ PC6     │ TIM2_CH1/TIM8   │
 * │ Motor4 │ PA1  │ PC8  │ PC9     │ TIM2_CH2/TIM8   │
 * └────────┴──────┴──────┴─────────┴─────────────────┘
 * 
 * Communication Protocol:
 * - Serial: 115200 baud, 8N1
 * - Text Commands: M1:50, M2:-30, M3:25, M4:-10, STOP, STATUS
 * - Status Format: STATUS,m1_tgt,m1_cur,m1_rpm,m2_tgt,m2_cur,m2_rpm,m3_tgt,m3_cur,m3_rpm,m4_tgt,m4_cur,m4_rpm
 * 
 * @copyright Copyright (c) 2025 STM32 Robotics Team
 * @license MIT License
 */

#include <Arduino.h>

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================

// Firmware version
#define FIRMWARE_VERSION "2.0.0"
#define FIRMWARE_NAME "STM32-4Motor-Controller"

// Mathematical constants
#ifndef PI
#define PI 3.14159265359f
#endif

// Hardware Configuration - Motor Control Pins
namespace MotorPins {
    // Motor 1 (Left Wheel - Differential Drive)
    constexpr uint8_t MOTOR1_PWM = PA6;   // TIM3_CH1
    constexpr uint8_t MOTOR1_DIR = PB5;   // GPIO
    constexpr uint8_t MOTOR1_SPEED = PA8; // TIM1_CH1 (Interrupt)
    
    // Motor 2 (Right Wheel - Differential Drive)
    constexpr uint8_t MOTOR2_PWM = PA7;   // TIM3_CH2
    constexpr uint8_t MOTOR2_DIR = PB10;  // GPIO
    constexpr uint8_t MOTOR2_SPEED = PB6; // TIM4_CH1 (Interrupt)
    
    // Motor 3 (Auxiliary Motor)
    constexpr uint8_t MOTOR3_PWM = PA0;   // TIM2_CH1
    constexpr uint8_t MOTOR3_DIR = PC7;   // GPIO
    constexpr uint8_t MOTOR3_SPEED = PC6; // TIM8_CH1 (Interrupt)
    
    // Motor 4 (Auxiliary Motor)
    constexpr uint8_t MOTOR4_PWM = PA1;   // TIM2_CH2
    constexpr uint8_t MOTOR4_DIR = PC8;   // GPIO
    constexpr uint8_t MOTOR4_SPEED = PC9; // TIM8_CH4 (Interrupt)
}

// Control System Configuration
namespace ControlConfig {
    constexpr uint32_t UPDATE_INTERVAL_MS = 20;    // 50Hz control loop
    constexpr float ACCELERATION_RATE = 2.0f;       // %/update for acceleration
    constexpr float DECELERATION_RATE = 3.0f;       // %/update for deceleration
    constexpr float MAX_DUTY_CYCLE = 100.0f;        // Maximum duty cycle percentage
    constexpr float MIN_DUTY_CYCLE = -100.0f;       // Minimum duty cycle percentage
}

// Tachometer Configuration
namespace TachometerConfig {
    constexpr uint16_t PULSES_PER_REVOLUTION = 1000; // Encoder pulses per motor revolution
    constexpr uint32_t SAMPLE_INTERVAL_MS = 100;     // RPM calculation interval
    constexpr uint32_t SIGNAL_TIMEOUT_MS = 2000;     // Timeout for detecting stopped motor
    constexpr float MIN_VALID_RPM = 0.1f;            // Minimum RPM considered valid
    constexpr float MAX_VALID_RPM = 10000.0f;        // Maximum RPM considered valid
}

// Communication Configuration
namespace CommConfig {
    constexpr uint32_t SERIAL_BAUD_RATE = 115200;   // Serial communication speed
    constexpr uint32_t STATUS_INTERVAL_MS = 200;     // Status reporting interval
    constexpr size_t CMD_BUFFER_SIZE = 128;          // Command buffer size
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Motor control state structure
 */
struct MotorState {
    float targetDutyCycle;      ///< Target duty cycle (-100 to +100)
    float currentDutyCycle;     ///< Current duty cycle (-100 to +100)
    uint32_t lastUpdateTime;    ///< Last update timestamp (ms)
    
    // Constructor
    MotorState() : targetDutyCycle(0.0f), currentDutyCycle(0.0f), lastUpdateTime(0) {}
};

/**
 * @brief Tachometer measurement structure
 */
struct TachometerData {
    volatile uint32_t pulseCount;       ///< Total pulse count
    volatile uint32_t lastPulseMicros;  ///< Timestamp of last pulse (microseconds)
    volatile uint32_t lastPeriodMicros; ///< Period between last two pulses (microseconds)
    uint32_t lastUpdateTime;            ///< Last RPM calculation time (ms)
    float currentRPM;                   ///< Current RPM reading
    
    // Constructor
    TachometerData() : pulseCount(0), lastPulseMicros(0), lastPeriodMicros(0), 
                      lastUpdateTime(0), currentRPM(0.0f) {}
};

/**
 * @brief System status structure
 */
struct SystemStatus {
    bool initialized;                   ///< System initialization status
    uint32_t lastStatusTime;           ///< Last status report time
    uint32_t lastHeartbeatTime;        ///< Last heartbeat LED toggle time
    bool heartbeatState;               ///< Current heartbeat LED state
    uint32_t uptime;                   ///< System uptime in milliseconds
    
    // Constructor
    SystemStatus() : initialized(false), lastStatusTime(0), lastHeartbeatTime(0), 
                    heartbeatState(false), uptime(0) {}
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Motor control states
MotorState g_motors[4];

// Tachometer data
TachometerData g_tachometers[4];

// System status
SystemStatus g_systemStatus;

// Communication buffer
String g_commandBuffer;

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

/**
 * @brief Generic tachometer pulse interrupt handler
 * @param tachoData Reference to tachometer data structure
 */
void handleTachometerPulse(volatile TachometerData& tachoData) {
    const uint32_t currentTime = micros();
    tachoData.pulseCount++;
    
    if (tachoData.lastPulseMicros > 0) {
        tachoData.lastPeriodMicros = currentTime - tachoData.lastPulseMicros;
    }
    tachoData.lastPulseMicros = currentTime;
}

// Individual ISR functions
void motor1SpeedISR() { handleTachometerPulse(g_tachometers[0]); }
void motor2SpeedISR() { handleTachometerPulse(g_tachometers[1]); }
void motor3SpeedISR() { handleTachometerPulse(g_tachometers[2]); }
void motor4SpeedISR() { handleTachometerPulse(g_tachometers[3]); }

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Constrain a float value between min and max bounds
 * @param value Input value
 * @param minVal Minimum allowed value
 * @param maxVal Maximum allowed value
 * @return Constrained value
 */
inline float constrainFloat(float value, float minVal, float maxVal) {
    return (value < minVal) ? minVal : (value > maxVal) ? maxVal : value;
}

/**
 * @brief Convert duty cycle percentage to PWM value (0-255)
 * @param dutyCycle Duty cycle percentage (-100 to +100)
 * @return PWM value (0-255)
 */
inline uint8_t dutyCycleToPWM(float dutyCycle) {
    const float absDuty = abs(dutyCycle);
    return static_cast<uint8_t>(constrainFloat(absDuty * 255.0f / 100.0f, 0.0f, 255.0f));
}

/**
 * @brief Validate RPM reading for sanity
 * @param rpm RPM value to validate
 * @return true if RPM is within valid range
 */
inline bool isValidRPM(float rpm) {
    return (rpm >= TachometerConfig::MIN_VALID_RPM && rpm <= TachometerConfig::MAX_VALID_RPM);
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Set motor PWM and direction based on duty cycle
 * @param motorIndex Motor index (0-3)
 * @param dutyCycle Duty cycle percentage (-100 to +100)
 */
void setMotorOutput(uint8_t motorIndex, float dutyCycle) {
    if (motorIndex >= 4) return; // Safety check
    
    dutyCycle = constrainFloat(dutyCycle, ControlConfig::MIN_DUTY_CYCLE, ControlConfig::MAX_DUTY_CYCLE);
    
    // Pin arrays for efficient indexing
    static const uint8_t pwmPins[] = {MotorPins::MOTOR1_PWM, MotorPins::MOTOR2_PWM, 
                                     MotorPins::MOTOR3_PWM, MotorPins::MOTOR4_PWM};
    static const uint8_t dirPins[] = {MotorPins::MOTOR1_DIR, MotorPins::MOTOR2_DIR, 
                                     MotorPins::MOTOR3_DIR, MotorPins::MOTOR4_DIR};
    
    const bool isForward = dutyCycle >= 0.0f;
    const uint8_t pwmValue = dutyCycleToPWM(dutyCycle);
    
    // Set direction
    digitalWrite(dirPins[motorIndex], isForward ? HIGH : LOW);
    
    // Set PWM
    analogWrite(pwmPins[motorIndex], pwmValue);
}

/**
 * @brief Set target duty cycle for a motor
 * @param motorIndex Motor index (0-3)
 * @param targetDutyCycle Target duty cycle percentage (-100 to +100)
 */
void setMotorTarget(uint8_t motorIndex, float targetDutyCycle) {
    if (motorIndex >= 4) return; // Safety check
    
    g_motors[motorIndex].targetDutyCycle = constrainFloat(targetDutyCycle, 
                                                         ControlConfig::MIN_DUTY_CYCLE, 
                                                         ControlConfig::MAX_DUTY_CYCLE);
}

/**
 * @brief Update motor control with acceleration/deceleration limiting
 * @param motorIndex Motor index (0-3)
 */
void updateMotorControl(uint8_t motorIndex) {
    if (motorIndex >= 4) return; // Safety check
    
    MotorState& motor = g_motors[motorIndex];
    const uint32_t currentTime = millis();
    
    // Check if it's time for an update
    if (currentTime - motor.lastUpdateTime < ControlConfig::UPDATE_INTERVAL_MS) {
        return;
    }
    
    const float difference = motor.targetDutyCycle - motor.currentDutyCycle;
    const float maxChange = (difference > 0) ? ControlConfig::ACCELERATION_RATE : ControlConfig::DECELERATION_RATE;
    
    // Apply rate limiting
    if (abs(difference) <= maxChange) {
        motor.currentDutyCycle = motor.targetDutyCycle;
    } else {
        motor.currentDutyCycle += (difference > 0) ? maxChange : -maxChange;
    }
    
    // Update motor output
    setMotorOutput(motorIndex, motor.currentDutyCycle);
    motor.lastUpdateTime = currentTime;
}

/**
 * @brief Emergency stop all motors
 */
void emergencyStop() {
    for (uint8_t i = 0; i < 4; i++) {
        setMotorTarget(i, 0.0f);
        g_motors[i].currentDutyCycle = 0.0f;
        setMotorOutput(i, 0.0f);
    }
    Serial.println("EMERGENCY_STOP_ACTIVATED");
}

// ============================================================================
// TACHOMETER FUNCTIONS
// ============================================================================

/**
 * @brief Calculate RPM from tachometer data
 * @param motorIndex Motor index (0-3)
 * @return Calculated RPM value
 */
float calculateRPM(uint8_t motorIndex) {
    if (motorIndex >= 4) return 0.0f; // Safety check
    
    TachometerData& tacho = g_tachometers[motorIndex];
    const uint32_t currentTime = millis();
    
    // Check if enough time has passed for a valid reading
    if (currentTime - tacho.lastUpdateTime < TachometerConfig::SAMPLE_INTERVAL_MS) {
        return tacho.currentRPM; // Return cached value
    }
    
    // Check for signal timeout (motor stopped)
    if ((micros() - tacho.lastPulseMicros) > (TachometerConfig::SIGNAL_TIMEOUT_MS * 1000UL)) {
        tacho.currentRPM = 0.0f;
        tacho.lastUpdateTime = currentTime;
        return 0.0f;
    }
    
    // Calculate RPM using period-based method for better accuracy at low speeds
    if (tacho.lastPeriodMicros > 0) {
        const float frequency = 1000000.0f / static_cast<float>(tacho.lastPeriodMicros);
        const float rpm = (frequency * 60.0f) / TachometerConfig::PULSES_PER_REVOLUTION;
        
        // Validate and update
        if (isValidRPM(rpm)) {
            tacho.currentRPM = rpm;
        } else {
            tacho.currentRPM = 0.0f; // Invalid reading
        }
    } else {
        tacho.currentRPM = 0.0f; // No valid period data
    }
    
    tacho.lastUpdateTime = currentTime;
    return tacho.currentRPM;
}

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================


/**
 * @brief Parse and execute text command
 * @param command Text command string
 * @return true if command was successfully executed
 */
/**
 * @brief Process a single motor command or system command
 * @param command Single command string (e.g., "M1:50", "STOP", "STATUS")
 * @return true if command was successfully executed
 */
bool processSingleMotorCommand(const String& command);

/**
 * @brief Process text-based command input (supports single and multi-motor commands)
 * @param command Command string to process
 * @return true if command was successfully executed
 */
bool processTextCommand(const String& command) {
    String cmd = command;
    cmd.trim();
    
    // Check if this is a multi-motor command (contains commas)
    if (cmd.indexOf(',') > 0) {
        // Parse comma-separated motor commands: M1:50,M2:30,M3:25,M4:10
        bool allSuccess = true;
        int startPos = 0;
        
        while (startPos < cmd.length()) {
            int commaPos = cmd.indexOf(',', startPos);
            if (commaPos == -1) commaPos = cmd.length();
            
            String singleCmd = cmd.substring(startPos, commaPos);
            singleCmd.trim();
            
            if (singleCmd.length() > 0) {
                bool success = processSingleMotorCommand(singleCmd);
                if (!success) allSuccess = false;
            }
            
            startPos = commaPos + 1;
        }
        return allSuccess;
    }
    else {
        // Single command processing
        return processSingleMotorCommand(cmd);
    }
}

/**
 * @brief Process a single motor command or system command
 * @param command Single command string (e.g., "M1:50", "STOP", "STATUS")
 * @return true if command was successfully executed
 */
bool processSingleMotorCommand(const String& command) {
    String cmd = command;
    cmd.trim();
    
    if (cmd.startsWith("M") && cmd.indexOf(':') > 0) {
        // Motor command: M1:50, M2:-30, etc.
        const int motorNum = cmd.substring(1, cmd.indexOf(':')).toInt();
        const float dutyCycle = cmd.substring(cmd.indexOf(':') + 1).toFloat();
        
        if (motorNum >= 1 && motorNum <= 4) {
            setMotorTarget(motorNum - 1, dutyCycle); // Convert to 0-based index
            return true;
        }
    }
    else if (cmd.equals("STOP")) {
        emergencyStop();
        return true;
    }
    else if (cmd.equals("STATUS")) {
        g_systemStatus.lastStatusTime = 0; // Force immediate status update
        return true;
    }
    else if (cmd.equals("VERSION")) {
        Serial.print("FIRMWARE: ");
        Serial.print(FIRMWARE_NAME);
        Serial.print(" v");
        Serial.println(FIRMWARE_VERSION);
        return true;
    }
    
    Serial.print("UNKNOWN_TEXT_COMMAND: ");
    Serial.println(cmd);
    return false;
}

/**
 * @brief Process incoming serial command
 * @param command Command string to process
 */
void processCommand(const String& command) {
    if (command.length() == 0) return;
    
    const bool success = processTextCommand(command);
    
    if (!success) {
        Serial.println("COMMAND_FAILED");
    }
}

/**
 * @brief Send system status over serial
 */
void sendStatusReport() {
    const uint32_t currentTime = millis();
    
    if (currentTime - g_systemStatus.lastStatusTime < CommConfig::STATUS_INTERVAL_MS) {
        return;
    }
    
    // Update all RPM readings
    for (uint8_t i = 0; i < 4; i++) {
        calculateRPM(i);
    }
    
    // Send status in CSV format
    Serial.print("STATUS");
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(",");
        Serial.print(g_motors[i].targetDutyCycle, 1);
        Serial.print(",");
        Serial.print(g_motors[i].currentDutyCycle, 1);
        Serial.print(",");
        Serial.print(g_tachometers[i].currentRPM, 1);
    }
    Serial.println();
    
    g_systemStatus.lastStatusTime = currentTime;
}

// ============================================================================
// SYSTEM FUNCTIONS
// ============================================================================

/**
 * @brief Initialize hardware pins and peripherals
 */
void initializeHardware() {
    // Configure motor control pins
    const uint8_t pwmPins[] = {MotorPins::MOTOR1_PWM, MotorPins::MOTOR2_PWM, 
                              MotorPins::MOTOR3_PWM, MotorPins::MOTOR4_PWM};
    const uint8_t dirPins[] = {MotorPins::MOTOR1_DIR, MotorPins::MOTOR2_DIR, 
                              MotorPins::MOTOR3_DIR, MotorPins::MOTOR4_DIR};
    
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(pwmPins[i], OUTPUT);
        pinMode(dirPins[i], OUTPUT);
        setMotorOutput(i, 0.0f); // Initialize to stopped
    }
    
    // Configure tachometer input pins
    const uint8_t speedPins[] = {MotorPins::MOTOR1_SPEED, MotorPins::MOTOR2_SPEED, 
                                MotorPins::MOTOR3_SPEED, MotorPins::MOTOR4_SPEED};
    void (*isrFunctions[])() = {motor1SpeedISR, motor2SpeedISR, motor3SpeedISR, motor4SpeedISR};
    
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(speedPins[i], INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(speedPins[i]), isrFunctions[i], RISING);
    }
    
    // Configure heartbeat LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
}

/**
 * @brief Print startup banner and system information
 */
void printStartupBanner() {
    Serial.println("=====================================");
    Serial.print("   ");
    Serial.print(FIRMWARE_NAME);
    Serial.print(" v");
    Serial.println(FIRMWARE_VERSION);
    Serial.println("=====================================");
    Serial.println("Hardware: STM32F446ZE Nucleo Board");
    Serial.println("Features: 4-Motor Control + Tachometers");
    Serial.println("Commands: Text (M1:50, STOP, etc.)");
    Serial.println("Status: Ready for operation");
    Serial.println("=====================================");
}

/**
 * @brief Update heartbeat LED
 */
void updateHeartbeat() {
    const uint32_t currentTime = millis();
    
    if (currentTime - g_systemStatus.lastHeartbeatTime >= 500) {
        g_systemStatus.heartbeatState = !g_systemStatus.heartbeatState;
        digitalWrite(LED_BUILTIN, g_systemStatus.heartbeatState);
        g_systemStatus.lastHeartbeatTime = currentTime;
    }
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================

/**
 * @brief Arduino setup function
 */
void setup() {
    // Initialize serial communication
    Serial.begin(CommConfig::SERIAL_BAUD_RATE);
    while (!Serial && millis() < 3000) {
        delay(10); // Wait for serial with timeout
    }
    
    // Initialize hardware
    initializeHardware();
    
    // Print startup information
    printStartupBanner();
    
    // Mark system as initialized
    g_systemStatus.initialized = true;
    g_systemStatus.uptime = millis();
    
    Serial.println("SYSTEM_READY");
}

/**
 * @brief Arduino main loop function
 */
void loop() {
    const uint32_t currentTime = millis();
    g_systemStatus.uptime = currentTime;
    
    // Update motor control loops
    for (uint8_t i = 0; i < 4; i++) {
        updateMotorControl(i);
    }
    
    // Process incoming serial commands
    while (Serial.available()) {
        const char receivedChar = Serial.read();
        
        if (receivedChar == '\n' || receivedChar == '\r') {
            if (g_commandBuffer.length() > 0) {
                processCommand(g_commandBuffer);
                g_commandBuffer = "";
            }
        } else if (g_commandBuffer.length() < CommConfig::CMD_BUFFER_SIZE - 1) {
            g_commandBuffer += receivedChar;
        } else {
            // Buffer overflow protection
            g_commandBuffer = "";
            Serial.println("COMMAND_BUFFER_OVERFLOW");
        }
    }
    
    // Send periodic status updates
    sendStatusReport();
    
    // Update system heartbeat
    updateHeartbeat();
    
    // Small delay to prevent overwhelming the system
    delay(1);
}