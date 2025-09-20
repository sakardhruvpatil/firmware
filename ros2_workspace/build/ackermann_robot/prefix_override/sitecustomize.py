import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sakar_gaurd/firmware/ros2_workspace/install/ackermann_robot'
