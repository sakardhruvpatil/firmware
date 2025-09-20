#!/bin/bash

# Ackermann Robot - Jetson Orin Nano Deployment Script
# This script sets up the robot system on Jetson hardware

echo "🚀 Ackermann Robot - Jetson Orin Nano Deployment"
echo "================================================"

# Configuration
JETSON_IP="${1:-192.168.1.100}"  # Default IP, override with argument
JETSON_USER="${2:-jetson}"       # Default username
PROJECT_NAME="ackermann_robot"

echo "Target Jetson: $JETSON_USER@$JETSON_IP"
echo ""

# Step 1: Check SSH connection
echo "📡 Testing SSH connection..."
if ssh -q -o BatchMode=yes -o ConnectTimeout=5 $JETSON_USER@$JETSON_IP exit; then
    echo "✅ SSH connection successful"
else
    echo "❌ SSH connection failed. Please check:"
    echo "   - Jetson IP address: $JETSON_IP"
    echo "   - Username: $JETSON_USER"
    echo "   - SSH keys are set up"
    echo ""
    echo "Usage: $0 <jetson_ip> <username>"
    echo "Example: $0 192.168.1.100 jetson"
    exit 1
fi

# Step 2: Check/Install ROS 2 on Jetson
echo ""
echo "🤖 Checking ROS 2 installation on Jetson..."
ssh $JETSON_USER@$JETSON_IP "
    if command -v ros2 &> /dev/null; then
        echo '✅ ROS 2 is already installed'
        ros2 --version
    else
        echo '📦 Installing ROS 2 Humble...'
        sudo apt update
        sudo apt install -y software-properties-common
        sudo add-apt-repository universe
        sudo apt update && sudo apt install -y curl
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install -y ros-humble-desktop
        sudo apt install -y python3-colcon-common-extensions
        echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
        echo '✅ ROS 2 Humble installed successfully'
    fi
"

# Step 3: Install additional dependencies
echo ""
echo "📦 Installing required dependencies..."
ssh $JETSON_USER@$JETSON_IP "
    sudo apt update
    sudo apt install -y python3-pip python3-serial python3-matplotlib python3-numpy
    pip3 install pymodbus pyserial
    echo '✅ Dependencies installed'
"

# Step 4: Create workspace on Jetson
echo ""
echo "📁 Creating workspace on Jetson..."
ssh $JETSON_USER@$JETSON_IP "
    mkdir -p ~/robot_ws/src
    echo '✅ Workspace created'
"

# Step 5: Transfer package files
echo ""
echo "📤 Transferring robot package to Jetson..."
cd ../ros2_workspace

# Create temporary clean package without build artifacts
rm -rf /tmp/ackermann_robot_deploy
mkdir -p /tmp/ackermann_robot_deploy
cp -r src/ackermann_robot /tmp/ackermann_robot_deploy/

# Transfer the package
scp -r /tmp/ackermann_robot_deploy/ackermann_robot $JETSON_USER@$JETSON_IP:~/robot_ws/src/

echo "✅ Package transferred successfully"

# Step 6: Build on Jetson
echo ""
echo "🔨 Building package on Jetson..."
ssh $JETSON_USER@$JETSON_IP "
    cd ~/robot_ws
    source /opt/ros/humble/setup.bash
    colcon build --packages-select ackermann_robot
    echo '✅ Package built successfully'
"

# Step 7: Create hardware launch script
echo ""
echo "⚙️ Creating hardware launch script..."
ssh $JETSON_USER@$JETSON_IP "cat > ~/robot_ws/launch_hardware.sh << 'EOF'
#!/bin/bash
# Hardware launch script for Jetson

echo '🤖 Starting Ackermann Robot Hardware System'
cd ~/robot_ws
source install/setup.bash

# Check hardware connections
echo '🔌 Checking hardware connections...'
if [ -e /dev/ttyACM0 ]; then
    echo '✅ STM32 controller found at /dev/ttyACM0'
else
    echo '❌ STM32 controller not found at /dev/ttyACM0'
fi

if [ -e /dev/ttyUSB0 ]; then
    echo '✅ Servo controller found at /dev/ttyUSB0'
else
    echo '❌ Servo controller not found at /dev/ttyUSB0'
fi

echo ''
echo '🚀 Launching robot system...'
ros2 launch ackermann_robot ackermann_robot_hardware.launch.py
EOF

chmod +x ~/robot_ws/launch_hardware.sh
echo '✅ Hardware launch script created'
"

echo ""
echo "🎉 Deployment Complete!"
echo ""
echo "📋 Next Steps:"
echo "1. SSH to Jetson: ssh $JETSON_USER@$JETSON_IP"
echo "2. Connect hardware:"
echo "   - STM32 controller via USB (should appear as /dev/ttyACM0)"
echo "   - Servo controllers via USB-RS485 (should appear as /dev/ttyUSB0)"
echo "3. Launch system: ~/robot_ws/launch_hardware.sh"
echo "4. Control robot: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "🔧 Hardware Configuration:"
echo "   - STM32: /dev/ttyACM0, 115200 baud"
echo "   - Servos: /dev/ttyUSB0, 38400 baud, Modbus IDs 2,3"
echo ""