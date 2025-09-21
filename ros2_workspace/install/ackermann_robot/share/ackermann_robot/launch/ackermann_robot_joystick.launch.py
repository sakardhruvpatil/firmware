#!/usr/bin/env python3
"""
Ackermann Robot Hardware Launch with PS4 Joystick Control

This launch file starts the complete Ackermann robot system with PS4 controller support:
- Hardware interface for motor control
- Kinematics node for motion planning  
- PS4 joystick driver and teleop
- Static transforms for robot frames

Controls:
- Left stick: Forward/backward and steering
- L1 button: Enable movement (safety)
- R1 button: Turbo mode (higher speeds)

Author: Ackermann Robot Team
License: MIT
Version: 1.0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate launch description for Ackermann robot with joystick control"""
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ackermann_robot'),
            'config',
            'hardware_params.yaml'
        ]),
        description='Path to hardware configuration file'
    )
    
    joy_config_file_arg = DeclareLaunchArgument(
        'joy_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ackermann_robot'),
            'config',
            'ps4_joy_config.yaml'
        ]),
        description='Path to joystick configuration file'
    )
    
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Whether to start joystick nodes'
    )
    
    joy_device_arg = DeclareLaunchArgument(
        'joy_device',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    joy_config_file = LaunchConfiguration('joy_config_file')
    use_joystick = LaunchConfiguration('use_joystick')
    joy_device = LaunchConfiguration('joy_device')
    
    # Core hardware nodes
    kinematics_node = Node(
        package='ackermann_robot',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    hardware_interface_node = Node(
        package='ackermann_robot',
        executable='hardware_interface',
        name='hardware_interface',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Joystick nodes (conditional)
    joystick_group = GroupAction(
        condition=IfCondition(use_joystick),
        actions=[
            # Joy driver node
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[
                    joy_config_file,
                    {'device_id': 0}  # Use device_id instead of device_name
                ],
                output='screen',
                emulate_tty=True,
            ),
            
            # Teleop twist joy node
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[joy_config_file],
                remappings=[
                    ('cmd_vel', 'cmd_vel'),
                    ('joy', 'joy'),
                ],
                output='screen',
                emulate_tty=True,
            ),
        ]
    )
    
    # Static transform publishers
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['--x', '0', '--y', '0', '--z', '0.1', 
                  '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                  '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
    )
    
    # Wheel frame transforms with proper Ackermann geometry
    wheel_transforms = []
    wheel_positions = [
        ('left_front_wheel', '0.32', '0.3', '0.05'),
        ('left_rear_wheel', '-0.32', '0.3', '0.05'),
        ('right_front_wheel', '0.32', '-0.3', '0.05'),
        ('right_rear_wheel', '-0.32', '-0.3', '0.05'),
    ]
    
    for wheel_name, x, y, z in wheel_positions:
        wheel_transforms.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{wheel_name}_transform',
                arguments=['--x', x, '--y', y, '--z', z,
                          '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                          '--frame-id', 'base_link', '--child-frame-id', wheel_name]
            )
        )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        joy_config_file_arg,
        use_joystick_arg,
        joy_device_arg,
        
        # Core hardware nodes
        kinematics_node,
        hardware_interface_node,
        
        # Joystick control group
        joystick_group,
        
        # Transform publishers
        base_link_to_base_footprint,
        *wheel_transforms,
    ])