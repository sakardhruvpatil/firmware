#!/usr/bin/env python3
"""
Simulation Node for Ackermann Robot

Provides physics simulation when hardware is not available.
Implements the exact kinematics as described in the requirements.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from dataclasses import dataclass
import time

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

@dataclass 
class SimulationState:
    """Complete simulation state"""
    # World position and orientation
    x: float = 0.0
    y: float = 0.0
    psi: float = 0.0  # heading angle
    
    # Robot-frame velocities
    v_x: float = 0.0
    v_y: float = 0.0
    omega: float = 0.0
    
    # Steering disk angles (commanded)
    theta_L: float = 0.0  # Left steering disk
    theta_R: float = 0.0  # Right steering disk
    
    # Individual wheel velocities (m/s)
    v_LF: float = 0.0  # Left Front
    v_LR: float = 0.0  # Left Rear
    v_RF: float = 0.0  # Right Front  
    v_RR: float = 0.0  # Right Rear
    
    # Individual wheel steering angles
    delta_LF: float = 0.0
    delta_LR: float = 0.0
    delta_RF: float = 0.0
    delta_RR: float = 0.0

class AckermannSimulation:
    """
    Physics simulation implementing the exact kinematics described:
    - All 4 wheels independently driven
    - Each side has steering disk controlling front/rear wheels
    - Front wheels steer inward, rear wheels steer outward
    """
    
    def __init__(self, wheelbase=0.64, track_width=0.6, max_steer_angle=28.0):
        # Robot geometry
        self.L = wheelbase      # Wheelbase (m)
        self.T = track_width    # Track width (m)
        self.max_steer_rad = math.radians(max_steer_angle)
        
        # Simulation state
        self.state = SimulationState()
        
        # Physics parameters
        self.dt = 0.02  # 50 Hz simulation
        self.noise_enabled = False
        self.noise_std = {'velocity': 0.01, 'steering': 0.1}
        
    def compute_wheel_steering_angles(self, theta_L: float, theta_R: float):
        """
        Compute individual wheel steering angles from steering disk angles
        
        As specified:
        - Front wheel steers inward (toward robot centerline)
        - Rear wheel steers outward (away from centerline)  
        - Both have equal magnitude, opposite signs
        """
        # Left side
        delta_LF = +theta_L  # Left front steers inward (positive)
        delta_LR = -theta_L  # Left rear steers outward (negative)
        
        # Right side  
        delta_RF = +theta_R  # Right front steers inward (positive)
        delta_RR = -theta_R  # Right rear steers outward (negative)
        
        return delta_LF, delta_LR, delta_RF, delta_RR
        
    def forward_kinematics(self):
        """
        Compute robot velocity from wheel velocities using the specified method
        """
        # Get individual wheel steering angles
        delta_LF, delta_LR, delta_RF, delta_RR = self.compute_wheel_steering_angles(
            self.state.theta_L, self.state.theta_R
        )
        
        # Store for visualization
        self.state.delta_LF = delta_LF
        self.state.delta_LR = delta_LR
        self.state.delta_RF = delta_RF
        self.state.delta_RR = delta_RR
        
        # Decompose each wheel's velocity into robot-frame x,y components
        # and average over wheels to get robot velocity
        v_x_robot = (
            self.state.v_LF * np.cos(delta_LF) +
            self.state.v_LR * np.cos(delta_LR) + 
            self.state.v_RF * np.cos(delta_RF) +
            self.state.v_RR * np.cos(delta_RR)
        ) / 4.0
        
        v_y_robot = (
            self.state.v_LF * np.sin(delta_LF) +
            self.state.v_LR * np.sin(delta_LR) +
            self.state.v_RF * np.sin(delta_RF) + 
            self.state.v_RR * np.sin(delta_RR)
        ) / 4.0
        
        # Angular velocity computation (simplified approach)
        # Could be enhanced with more sophisticated methods
        omega_robot = self._compute_angular_velocity(delta_LF, delta_LR, delta_RF, delta_RR)
        
        self.state.v_x = v_x_robot
        self.state.v_y = v_y_robot
        self.state.omega = omega_robot
        
    def _compute_angular_velocity(self, delta_LF, delta_LR, delta_RF, delta_RR):
        """Compute angular velocity from wheel steering angles and velocities"""
        
        # Method 1: Use front wheels for primary turning calculation
        # This assumes front wheels are primary steering wheels
        if abs(delta_LF) > 1e-6 or abs(delta_RF) > 1e-6:
            # Average front wheel steering contribution
            front_contribution = (
                self.state.v_LF * np.sin(delta_LF) + 
                self.state.v_RF * np.sin(delta_RF)
            ) / 2.0
            
            omega = front_contribution / (self.L / 2.0)  # Approximate
        else:
            omega = 0.0
            
        return omega
        
    def integrate_motion(self):
        """Integrate robot motion using current velocities"""
        
        # Compute forward kinematics first
        self.forward_kinematics()
        
        # Transform robot-frame velocity to world frame
        cos_psi = np.cos(self.state.psi)
        sin_psi = np.sin(self.state.psi)
        
        v_x_world = self.state.v_x * cos_psi - self.state.v_y * sin_psi
        v_y_world = self.state.v_x * sin_psi + self.state.v_y * cos_psi
        
        # Integrate position and heading (Euler integration)
        self.state.x += v_x_world * self.dt
        self.state.y += v_y_world * self.dt
        self.state.psi += self.state.omega * self.dt
        
        # Normalize heading to [-pi, pi]
        self.state.psi = np.arctan2(np.sin(self.state.psi), np.cos(self.state.psi))
        
        # Add noise if enabled
        if self.noise_enabled:
            self.state.x += np.random.normal(0, self.noise_std['velocity'] * self.dt)
            self.state.y += np.random.normal(0, self.noise_std['velocity'] * self.dt)
            self.state.psi += np.random.normal(0, self.noise_std['steering'] * self.dt)
            
    def set_wheel_commands(self, velocities, steering_angles):
        """
        Set commanded wheel velocities and steering disk angles
        
        Args:
            velocities: [v_LF, v_LR, v_RF, v_RR] in m/s
            steering_angles: [theta_L, theta_R] in radians
        """
        if len(velocities) >= 4:
            self.state.v_LF = velocities[0]
            self.state.v_LR = velocities[1]
            self.state.v_RF = velocities[2]
            self.state.v_RR = velocities[3]
            
        if len(steering_angles) >= 2:
            # Apply steering limits
            self.state.theta_L = np.clip(steering_angles[0], -self.max_steer_rad, self.max_steer_rad)
            self.state.theta_R = np.clip(steering_angles[1], -self.max_steer_rad, self.max_steer_rad)
            
            # Apply sign correction if needed (uncomment if steering is inverted)
            # self.state.theta_L = -self.state.theta_L
            # self.state.theta_R = -self.state.theta_R
            
    def get_state(self):
        """Get current simulation state"""
        return self.state
        
    def reset(self, x=0.0, y=0.0, psi=0.0):
        """Reset simulation to initial state"""
        self.state = SimulationState()
        self.state.x = x
        self.state.y = y  
        self.state.psi = psi

class SimulationNode(Node):
    """ROS 2 simulation node"""
    
    def __init__(self):
        super().__init__('simulation_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('simulation.enable', True),
                ('simulation.physics_rate', 50.0),
                ('robot.wheelbase', 0.64),
                ('robot.track_width', 0.6),
                ('robot.max_steering_angle', 28.0),
                ('simulation.add_noise', False),
                ('simulation.noise_std_dev.velocity', 0.01),
                ('simulation.noise_std_dev.steering', 0.1),
            ]
        )
        
        # Get parameters
        self.simulation_enabled = self.get_parameter('simulation.enable').value
        self.physics_rate = self.get_parameter('simulation.physics_rate').value
        wheelbase = self.get_parameter('robot.wheelbase').value
        track_width = self.get_parameter('robot.track_width').value
        max_steer_angle = self.get_parameter('robot.max_steering_angle').value
        
        if not self.simulation_enabled:
            self.get_logger().info('Simulation disabled')
            return
            
        # Initialize simulation
        self.sim = AckermannSimulation(wheelbase, track_width, max_steer_angle)
        self.sim.noise_enabled = self.get_parameter('simulation.add_noise').value
        self.sim.noise_std['velocity'] = self.get_parameter('simulation.noise_std_dev.velocity').value
        self.sim.noise_std['steering'] = self.get_parameter('simulation.noise_std_dev.steering').value
        self.sim.dt = 1.0 / self.physics_rate
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.wheel_state_pub = self.create_publisher(Float64MultiArray, 'wheel_states', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        
        # Subscribers  
        self.wheel_cmd_sub = self.create_subscription(
            Float64MultiArray, 'wheel_commands', self.wheel_command_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Simulation timer
        self.sim_timer = self.create_timer(1.0/self.physics_rate, self.simulation_step)
        
        # Visualization timer
        self.viz_timer = self.create_timer(0.1, self.publish_visualization)
        
        self.get_logger().info(f'Simulation node started at {self.physics_rate} Hz')
        
    def wheel_command_callback(self, msg: Float64MultiArray):
        """Handle wheel command messages"""
        try:
            if len(msg.data) >= 6:
                # Extract velocities and steering angles
                velocities = list(msg.data[0:4])
                steering_angles = list(msg.data[4:6])
                
                self.sim.set_wheel_commands(velocities, steering_angles)
                
        except Exception as e:
            self.get_logger().error(f'Wheel command callback error: {e}')
            
    def simulation_step(self):
        """Execute one simulation step"""
        try:
            # Integrate motion
            self.sim.integrate_motion()
            
            # Publish state
            self.publish_odometry()
            self.publish_wheel_states()
            self.publish_joint_states()
            self.publish_tf()
            
        except Exception as e:
            self.get_logger().error(f'Simulation step error: {e}')
            
    def publish_odometry(self):
        """Publish odometry message"""
        state = self.sim.get_state()
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from heading)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(state.psi / 2.0)
        odom.pose.pose.orientation.w = np.cos(state.psi / 2.0)
        
        # Velocity in robot frame
        odom.twist.twist.linear.x = state.v_x
        odom.twist.twist.linear.y = state.v_y
        odom.twist.twist.angular.z = state.omega
        
        self.odom_pub.publish(odom)
        
    def publish_wheel_states(self):
        """Publish current wheel states"""
        state = self.sim.get_state()
        
        msg = Float64MultiArray()
        
        # Format: [v_LF, v_LR, v_RF, v_RR, theta_L, theta_R]
        msg.data = [
            state.v_LF, state.v_LR, state.v_RF, state.v_RR,
            state.theta_L, state.theta_R
        ]
        
        self.wheel_state_pub.publish(msg)
        
    def publish_joint_states(self):
        """Publish joint states for visualization"""
        state = self.sim.get_state()
        
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names
        joint_state.name = [
            'left_front_wheel_joint',
            'left_rear_wheel_joint',
            'right_front_wheel_joint', 
            'right_rear_wheel_joint',
            'left_front_steer_joint',
            'left_rear_steer_joint',
            'right_front_steer_joint',
            'right_rear_steer_joint'
        ]
        
        # Wheel positions (integrated from velocities)
        # For simplicity, just use current velocities
        wheel_radius = 0.1  # meters
        wheel_positions = [
            0.0, 0.0, 0.0, 0.0  # Could integrate velocity to get position
        ]
        
        # Steering angles
        steering_positions = [
            state.delta_LF, state.delta_LR, 
            state.delta_RF, state.delta_RR
        ]
        
        joint_state.position = wheel_positions + steering_positions
        
        # Velocities
        joint_state.velocity = [
            state.v_LF / wheel_radius,
            state.v_LR / wheel_radius,
            state.v_RF / wheel_radius,
            state.v_RR / wheel_radius,
            0.0, 0.0, 0.0, 0.0  # Steering velocities
        ]
        
        self.joint_state_pub.publish(joint_state)
        
    def publish_tf(self):
        """Publish transform from odom to base_link"""
        state = self.sim.get_state()
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(state.psi / 2.0)
        t.transform.rotation.w = np.cos(state.psi / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
    def publish_visualization(self):
        """Publish visualization markers"""
        state = self.sim.get_state()
        
        marker_array = MarkerArray()
        
        # Robot body marker
        body_marker = Marker()
        body_marker.header.frame_id = 'base_link'
        body_marker.header.stamp = self.get_clock().now().to_msg()
        body_marker.ns = 'robot_body'
        body_marker.id = 0
        body_marker.type = Marker.CUBE
        body_marker.action = Marker.ADD
        
        body_marker.pose.position.x = 0.0
        body_marker.pose.position.y = 0.0
        body_marker.pose.position.z = 0.1
        body_marker.pose.orientation.w = 1.0
        
        body_marker.scale.x = self.sim.L
        body_marker.scale.y = self.sim.T
        body_marker.scale.z = 0.2
        
        body_marker.color.r = 0.2
        body_marker.color.g = 0.5
        body_marker.color.b = 0.8
        body_marker.color.a = 0.7
        
        marker_array.markers.append(body_marker)
        
        # Wheel markers with steering visualization
        wheel_positions = [
            (self.sim.L/2, self.sim.T/2, 'LF'),    # Left Front
            (-self.sim.L/2, self.sim.T/2, 'LR'),   # Left Rear
            (self.sim.L/2, -self.sim.T/2, 'RF'),   # Right Front  
            (-self.sim.L/2, -self.sim.T/2, 'RR')   # Right Rear
        ]
        
        steering_angles = [state.delta_LF, state.delta_LR, state.delta_RF, state.delta_RR]
        
        for i, ((x, y, name), angle) in enumerate(zip(wheel_positions, steering_angles)):
            wheel_marker = Marker()
            wheel_marker.header.frame_id = 'base_link'
            wheel_marker.header.stamp = self.get_clock().now().to_msg()
            wheel_marker.ns = 'wheels'
            wheel_marker.id = i
            wheel_marker.type = Marker.CYLINDER
            wheel_marker.action = Marker.ADD
            
            wheel_marker.pose.position.x = x
            wheel_marker.pose.position.y = y
            wheel_marker.pose.position.z = 0.05
            
            # Apply steering angle
            wheel_marker.pose.orientation.x = 0.0
            wheel_marker.pose.orientation.y = 0.0
            wheel_marker.pose.orientation.z = np.sin(angle / 2.0)
            wheel_marker.pose.orientation.w = np.cos(angle / 2.0)
            
            wheel_marker.scale.x = 0.2
            wheel_marker.scale.y = 0.2  
            wheel_marker.scale.z = 0.1
            
            wheel_marker.color.r = 0.1
            wheel_marker.color.g = 0.1
            wheel_marker.color.b = 0.1
            wheel_marker.color.a = 1.0
            
            marker_array.markers.append(wheel_marker)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    node = SimulationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()