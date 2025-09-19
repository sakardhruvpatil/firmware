#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch the Ackermann robot simulation with visualization
    """
    
    # Launch arguments
    use_visualization_arg = DeclareLaunchArgument(
        'use_visualization', 
        default_value='true',
        description='Enable visualization window'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ackermann_robot'),
            'config',
            'robot_params.yaml'
        ]),
        description='Path to robot configuration file'
    )
    
    # Get launch configurations
    use_visualization = LaunchConfiguration('use_visualization')
    config_file = LaunchConfiguration('config_file')
    
    # Core simulation nodes
    kinematics_node = Node(
        package='ackermann_robot',
        executable='kinematics_node',
        name='kinematics_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    simulation_node = Node(
        package='ackermann_robot',
        executable='simulation_node',
        name='simulation_node',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Visualization node
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
    
    return LaunchDescription([
        # Launch arguments
        use_visualization_arg,
        config_file_arg,
        
        # Core nodes
        kinematics_node,
        simulation_node,
        visualization_node,
        
        # Essential transforms
        base_link_to_base_footprint,
    ])