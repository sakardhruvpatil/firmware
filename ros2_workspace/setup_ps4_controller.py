#!/usr/bin/env python3
"""
PS4 Controller Setup and Test Script for Ackermann Robot

This script helps set up and test the PS4 controller connection.
"""

import subprocess
import time
import os
import sys

def run_command(command, timeout=10):
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

def check_ps4_controller():
    """Check if PS4 controller is connected"""
    print("Checking for PS4 controller...")
    
    # Check for joystick devices
    success, stdout, stderr = run_command("ls /dev/input/js* 2>/dev/null")
    if not success or not stdout.strip():
        print("ERROR: No joystick devices found at /dev/input/js*")
        print("\nTo connect PS4 controller:")
        print("1. Put controller in pairing mode (Share + PS button)")
        print("2. Run: sudo bluetoothctl")
        print("3. In bluetoothctl: scan on")
        print("4. Find controller MAC address")
        print("5. pair <MAC_ADDRESS>")
        print("6. connect <MAC_ADDRESS>")
        print("7. trust <MAC_ADDRESS>")
        return False
    
    print(f"SUCCESS: Joystick devices found:\n{stdout}")
    
    # Test joystick input
    print("Testing joystick input (press Ctrl+C to stop)...")
    try:
        subprocess.run(["jstest", "/dev/input/js0"], timeout=5)
    except subprocess.TimeoutExpired:
        print("SUCCESS: Joystick test completed")
    except FileNotFoundError:
        print("WARNING: jstest not found. Install with: sudo apt install joystick")
    except KeyboardInterrupt:
        print("SUCCESS: Joystick test stopped")
    
    return True

def test_ros2_joy():
    """Test ROS2 joy node"""
    print("\nTesting ROS2 joy node...")
    
    # Source ROS2
    env = os.environ.copy()
    env.update({
        'ROS_DOMAIN_ID': '0',
        'RMW_IMPLEMENTATION': 'rmw_cyclonedx_cpp'
    })
    
    # Check if joy node can start
    try:
        print("Starting joy node for 5 seconds...")
        proc = subprocess.Popen([
            'bash', '-c', 
            'source /opt/ros/humble/setup.bash && ros2 run joy joy_node'
        ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        time.sleep(5)
        proc.terminate()
        proc.wait(timeout=2)
        
        print("SUCCESS: Joy node test completed")
        return True
        
    except Exception as e:
        print(f"ERROR: Joy node test failed: {e}")
        return False

def show_controller_mapping():
    """Show PS4 controller button/axis mapping"""
    print("\nPS4 Controller Mapping:")
    print("=" * 50)
    print("MOVEMENT CONTROLS:")
    print("  Left Stick Vertical   → Forward/Backward")
    print("  Left Stick Horizontal → Steering")
    print("")
    print("SAFETY CONTROLS:")
    print("  L1 Button → Enable movement (must hold)")
    print("  R1 Button → Turbo mode (higher speeds)")
    print("")
    print("BUTTON MAPPING:")
    print("  X Button      → Button 0")
    print("  Circle Button → Button 1") 
    print("  Triangle      → Button 2")
    print("  Square        → Button 3")
    print("  L1            → Button 4 (Enable)")
    print("  R1            → Button 5 (Turbo)")
    print("  L2            → Button 6")
    print("  R2            → Button 7")
    print("  Share         → Button 8")
    print("  Options       → Button 9")
    print("  PS Button     → Button 10")
    print("  L3 (stick)    → Button 11")
    print("  R3 (stick)    → Button 12")

def main():
    """Main function"""
    print("PS4 Controller Setup for Ackermann Robot")
    print("=" * 50)
    
    # Check controller connection
    if not check_ps4_controller():
        return 1
    
    # Test ROS2 integration
    test_ros2_joy()
    
    # Show controller mapping
    show_controller_mapping()
    
    print(f"\nSUCCESS: Setup complete!")
    print(f"\nTo launch with joystick control:")
    print(f"cd ~/ros2_workspace")
    print(f"source install/setup.bash")
    print(f"ros2 launch ackermann_robot ackermann_robot_hardware.launch.py use_joystick:=true")
    print(f"\nOr use the dedicated joystick launch file:")
    print(f"ros2 launch ackermann_robot ackermann_robot_joystick.launch.py")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())