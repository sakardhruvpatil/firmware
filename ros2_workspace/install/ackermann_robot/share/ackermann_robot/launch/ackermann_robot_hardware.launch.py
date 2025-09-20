#!/usr/bin/env python3
"""
Professional Ackermann Robot Hardware Launch File

This launch file starts the complete Ackermann robot hardware system including:
- Kinematics node for motion planning
- Hardware interface for motor control
- Static transforms for robot frames

Author: Ackermann Robot Team
License: MIT
Version: 1.0.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Ackermann robot hardware system"""
    
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
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    
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
    
    # Static transform publishers using new-style arguments
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
        
        # Core hardware nodes
        kinematics_node,
        hardware_interface_node,
        
        # Transform publishers
        base_link_to_base_footprint,
        *wheel_transforms,
    ])