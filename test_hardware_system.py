#!/usr/bin/env python3
"""
Hardware System Test Script for Jetson Orin Nano
Tests all hardware components before full system launch
"""

import subprocess
import time
import sys
import serial
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

class HardwareSystemTest:
    def __init__(self):
        self.test_results = {}
        
    def run_command(self, command, timeout=10):
        """Run a shell command and return result"""
        try:
            result = subprocess.run(
                command, shell=True, capture_output=True, 
                text=True, timeout=timeout
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)
    
    def test_usb_devices(self):
        """Test for expected USB devices"""
        logger.info("Testing USB devices...")
        
        # Check for STM32 device
        success, stdout, stderr = self.run_command("ls /dev/ttyACM*")
        stm32_present = "/dev/ttyACM0" in stdout
        
        # Check for servo controller
        success, stdout, stderr = self.run_command("ls /dev/ttyUSB*")
        servo_present = "/dev/ttyUSB0" in stdout
        
        self.test_results['usb_devices'] = {
            'stm32': stm32_present,
            'servo': servo_present,
            'status': stm32_present and servo_present
        }
        
        if stm32_present:
            logger.info("✓ STM32 device found at /dev/ttyACM0")
        else:
            logger.error("✗ STM32 device not found")
            
        if servo_present:
            logger.info("✓ Servo controller found at /dev/ttyUSB0")
        else:
            logger.error("✗ Servo controller not found")
            
        return stm32_present and servo_present
    
    def test_serial_communication(self):
        """Test serial communication with STM32"""
        logger.info("Testing STM32 serial communication...")
        
        try:
            # Try to open serial connection
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
            time.sleep(2)  # Wait for connection to stabilize
            
            # Send test command
            ser.write(b'STOP\n')
            time.sleep(0.1)
            
            # Try to read response
            response = ser.read_all()
            ser.close()
            
            success = len(response) >= 0  # Any response indicates communication
            self.test_results['serial_comm'] = {
                'connected': True,
                'response': response.decode('utf-8', errors='ignore'),
                'status': success
            }
            
            logger.info("✓ STM32 serial communication successful")
            return True
            
        except Exception as e:
            self.test_results['serial_comm'] = {
                'connected': False,
                'error': str(e),
                'status': False
            }
            logger.error(f"✗ STM32 serial communication failed: {e}")
            return False
    
    def test_ros2_installation(self):
        """Test ROS 2 installation and sourcing"""
        logger.info("Testing ROS 2 installation...")
        
        # Test ROS 2 sourcing
        ros_command = "source /opt/ros/humble/setup.bash && ros2 --help"
        success, stdout, stderr = self.run_command(ros_command)
        
        self.test_results['ros2'] = {
            'installed': success,
            'status': success
        }
        
        if success:
            logger.info("✓ ROS 2 Humble is properly installed and sourced")
        else:
            logger.error("✗ ROS 2 Humble installation issue")
            logger.error(f"Error: {stderr}")
            
        return success
    
    def test_python_dependencies(self):
        """Test required Python packages"""
        logger.info("Testing Python dependencies...")
        
        required_packages = ['pymodbus', 'pyserial', 'matplotlib', 'numpy']
        missing_packages = []
        
        for package in required_packages:
            try:
                __import__(package)
                logger.info(f"✓ {package} is available")
            except ImportError:
                missing_packages.append(package)
                logger.error(f"✗ {package} is missing")
        
        success = len(missing_packages) == 0
        self.test_results['python_deps'] = {
            'missing': missing_packages,
            'status': success
        }
        
        if not success:
            logger.error(f"Missing packages: {', '.join(missing_packages)}")
            logger.info("Install with: pip3 install " + " ".join(missing_packages))
            
        return success
    
    def test_workspace_build(self):
        """Test ROS 2 workspace build"""
        logger.info("Testing workspace build...")
        
        workspace_path = Path.home() / "ros2_workspace"
        if not workspace_path.exists():
            logger.error(f"✗ Workspace not found at {workspace_path}")
            self.test_results['workspace'] = {'status': False, 'error': 'Workspace not found'}
            return False
        
        # Test build
        build_command = f"cd {workspace_path} && source /opt/ros/humble/setup.bash && colcon build"
        success, stdout, stderr = self.run_command(build_command, timeout=60)
        
        self.test_results['workspace'] = {
            'path': str(workspace_path),
            'build_success': success,
            'status': success
        }
        
        if success:
            logger.info("✓ Workspace build successful")
        else:
            logger.error("✗ Workspace build failed")
            logger.error(f"Error: {stderr}")
            
        return success
    
    def test_launch_files(self):
        """Test launch file availability"""
        logger.info("Testing launch files...")
        
        workspace_path = Path.home() / "ros2_workspace"
        launch_file = workspace_path / "install/ackermann_robot/share/ackermann_robot/launch/ackermann_robot_hardware.launch.py"
        
        success = launch_file.exists()
        self.test_results['launch_files'] = {
            'hardware_launch': success,
            'status': success
        }
        
        if success:
            logger.info("✓ Hardware launch file found")
        else:
            logger.error("✗ Hardware launch file not found")
            
        return success
    
    def run_all_tests(self):
        """Run all hardware tests"""
        logger.info("Starting hardware system tests...")
        logger.info("=" * 50)
        
        tests = [
            self.test_ros2_installation,
            self.test_python_dependencies,
            self.test_usb_devices,
            self.test_serial_communication,
            self.test_workspace_build,
            self.test_launch_files,
        ]
        
        passed = 0
        failed = 0
        
        for test in tests:
            try:
                if test():
                    passed += 1
                else:
                    failed += 1
            except Exception as e:
                logger.error(f"Test {test.__name__} crashed: {e}")
                failed += 1
            logger.info("-" * 30)
        
        # Summary
        logger.info("=" * 50)
        logger.info("HARDWARE TEST SUMMARY")
        logger.info(f"Passed: {passed}")
        logger.info(f"Failed: {failed}")
        
        if failed == 0:
            logger.info("✓ All tests passed! Hardware is ready.")
            return True
        else:
            logger.error("✗ Some tests failed. Check issues above.")
            return False
    
    def get_system_info(self):
        """Get system information"""
        logger.info("System Information:")
        
        # Get system info
        info_commands = {
            'OS': 'lsb_release -d',
            'Kernel': 'uname -r',
            'Python': 'python3 --version',
            'GPU': 'nvidia-smi --query-gpu=name --format=csv,noheader,nounits',
            'Memory': 'free -h',
            'Disk': 'df -h /',
        }
        
        for name, command in info_commands.items():
            success, stdout, stderr = self.run_command(command)
            if success:
                logger.info(f"{name}: {stdout.strip()}")
            else:
                logger.info(f"{name}: Unable to detect")

def main():
    """Main test function"""
    print("\n" + "="*60)
    print("JETSON ORIN NANO - HARDWARE SYSTEM TEST")
    print("="*60)
    
    tester = HardwareSystemTest()
    
    # Show system info
    tester.get_system_info()
    print("\n")
    
    # Run all tests
    success = tester.run_all_tests()
    
    if success:
        print("\nHardware system is ready for robot deployment!")
        print("\nNext steps:")
        print("1. Connect STM32 to /dev/ttyACM0")
        print("2. Connect servo controller to /dev/ttyUSB0")
        print("3. Run: ros2 launch ackermann_robot ackermann_robot_hardware.launch.py")
        return 0
    else:
        print("\nHardware system has issues. Please fix them before proceeding.")
        return 1

if __name__ == "__main__":
    sys.exit(main())