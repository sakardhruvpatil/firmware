# motor_driver.py
import time
import struct
import serial
import math

def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

def build_command(slave_id, function_code, address, data):
    command = [slave_id, function_code, address >> 8, address & 0xFF, data >> 8, data & 0xFF]
    crc = calculate_crc(command)
    command.append(crc & 0xFF)
    command.append((crc >> 8) & 0xFF)
    return bytes(command)

def int_to_bytes(value, length):
    return value.to_bytes(length, byteorder='big', signed=True)

class MotorController:
    def __init__(self, port, slave_id=1, baudrate=38400, timeout=0.1):
        """Create a motor controller.

        port may be either a string device path OR an existing serial.Serial instance
        so multiple MotorController objects can share one bus without reopening
        the device file (which can cause contention / OS-level locking).
        """
        if isinstance(port, serial.Serial):
            self.serial = port
        else:
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.slave_id = slave_id
        if not self.serial or not self.serial.is_open:
            raise Exception(f"Failed to open serial port {port}")
        # Optional external logger callback (set by caller)
        self.logger = None

    def send_command(self, command):
        self.serial.write(command)
        time.sleep(0.01)  # Reduced from 50ms to 10ms
        response = self.serial.read(8)
        return response

    # New low-level helpers
    def _write_single_register(self, address, value):
        cmd = build_command(self.slave_id, 6, address, value)
        return self.send_command(cmd)

    def read_register(self, address, quantity=1):
        """Read holding register(s) using Modbus function 0x03. Returns list of values or single int."""
        function_code = 3
        hi = (address >> 8) & 0xFF
        lo = address & 0xFF
        q_hi = (quantity >> 8) & 0xFF
        q_lo = quantity & 0xFF
        frame = [self.slave_id, function_code, hi, lo, q_hi, q_lo]
        crc = calculate_crc(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        self.serial.write(bytes(frame))
        time.sleep(0.01)  # Reduced from 50ms to 10ms
        # Expected response: slave, func, byte_count, data..., crc_lo, crc_hi
        header = self.serial.read(3)
        if len(header) < 3:
            return None
        byte_count = header[2]
        data_bytes = self.serial.read(byte_count + 2)  # includes CRC
        if len(data_bytes) < byte_count + 2:
            return None
        values = []
        for i in range(0, byte_count, 2):
            values.append((data_bytes[i] << 8) | data_bytes[i+1])
        return values[0] if quantity == 1 else values

    def write_register(self, address, value):
        """Write single holding register using Modbus function 0x06"""
        command = build_command(self.slave_id, 6, address, value)
        return self.send_command(command)

    def set_mode_position_control(self):
        """Set motor to position control mode"""
        command = build_command(self.slave_id, 6, 0x6200, 0x0001)  # Position control mode
        self.send_command(command)

    def set_angle_degrees(self, angle_degrees):
        """
        Set the steering angle in degrees (±26°)
        Uses PR0 absolute positioning mode for precise steering control
        """
        # Clamp to safety limits
        MAX_ANGLE = 26.0
        MIN_ANGLE = -26.0
        angle = max(MIN_ANGLE, min(MAX_ANGLE, angle_degrees))
        
        # Convert to motor position using parameters from GUI
        PULSES_PER_REVOLUTION = 10000
        GEARBOX_RATIO = 30
        EFFECTIVE_PULSES_PER_REVOLUTION = PULSES_PER_REVOLUTION * GEARBOX_RATIO  # 300,000
        
        target_position = int((angle / 360.0) * EFFECTIVE_PULSES_PER_REVOLUTION)
        pos32 = target_position & 0xFFFFFFFF
        hi = (pos32 >> 16) & 0xFFFF
        lo = pos32 & 0xFFFF
        
        # Stage PR0 parameters
        self.write_register(0x6200, 1)        # PR0 mode
        self.write_register(0x6201, hi)       # Position high
        self.write_register(0x6202, lo)       # Position low
        self.write_register(0x6203, 200)      # Velocity RPM
        self.write_register(0x6204, 100)      # Acceleration
        self.write_register(0x6205, 100)      # Deceleration
        self.write_register(0x6206, 0)        # Pause
        self.write_register(0x6207, 0x0010)   # Trigger setup
        
        # Execute the move
        self.write_register(0x6002, 0x0010)   # Start motion
        
        if self.logger:
            self.logger(f"[SID {self.slave_id}] Steering to {angle:.1f}° (pos: {target_position})")

    def stop_motion(self):
        command = build_command(self.slave_id, 6, 0x6002, 0x0040)
        self.send_command(command)

    def close(self):
        if hasattr(self, 'serial') and self.serial and self.serial.is_open:
            self.serial.close()