#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch the Ackermann robot hardware system on Jetson Orin Nano
    """
    
    # Launch arguments
    use_visualization_arg = DeclareLaunchArgument(
        'use_visualization', 
        default_value='false',
        description='Enable visualization window (disable for headless operation)'
    )
    
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
    use_visualization = LaunchConfiguration('use_visualization')
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
    
    # Optional visualization node (for debugging)
    visualization_node = Node(
        package='ackermann_robot',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_visualization)
    )
    
    # Essential transform publishers
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    # Wheel frame transforms for navigation
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
                arguments=[x, y, z, '0', '0', '0', 'base_link', wheel_name]
            )
        )
    
    return LaunchDescription([
        # Launch arguments
        use_visualization_arg,
        config_file_arg,
        
        # Core hardware nodes
        kinematics_node,
        hardware_interface_node,
        visualization_node,
        
        # Transform publishers
        base_link_to_base_footprint,
        *wheel_transforms,
    ])