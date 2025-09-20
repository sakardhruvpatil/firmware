#!/usr/bin/env python3
"""
Four-Wheel Independent Drive Ackermann Robot Kinematics

This node handles the complex kinematics for a robot where:
- All 4 wheels are independently driven
- Each side has a steering disk controlling front/rear wheels
- Front wheel steers inward, rear wheel steers outward (equal magnitude, opposite signs)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
import math
from dataclasses import dataclass
from typing import Tuple

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

@dataclass
class RobotGeometry:
    """Robot physical dimensions and constraints"""
    wheelbase: float = 0.64      # L - front to rear wheel centers (m)
    track_width: float = 0.6     # T - left to right wheel centers (m)  
    wheel_radius: float = 0.1    # Wheel radius (m)
    max_steer_angle: float = 0.488  # ±28° in radians
    
@dataclass 
class WheelState:
    """Individual wheel state"""
    velocity: float = 0.0      # Linear velocity (m/s)
    steering_angle: float = 0.0  # Steering angle (radians)
    rpm: float = 0.0           # Motor RPM

@dataclass
class RobotState:
    """Complete robot state"""
    # World coordinates
    x: float = 0.0
    y: float = 0.0  
    heading: float = 0.0  # psi (radians)
    
    # Robot-frame velocities
    v_x: float = 0.0      # Forward velocity (m/s)
    v_y: float = 0.0      # Lateral velocity (m/s)
    omega: float = 0.0    # Angular velocity (rad/s)
    
    # Steering disk angles
    theta_L: float = 0.0  # Left steering disk angle (radians)
    theta_R: float = 0.0  # Right steering disk angle (radians)
    
    # Individual wheel states
    left_front: WheelState = None
    left_rear: WheelState = None
    right_front: WheelState = None
    right_rear: WheelState = None
    
    def __post_init__(self):
        if self.left_front is None:
            self.left_front = WheelState()
        if self.left_rear is None:
            self.left_rear = WheelState()
        if self.right_front is None:
            self.right_front = WheelState()
        if self.right_rear is None:
            self.right_rear = WheelState()

class AckermannKinematics:
    """
    Handles forward and inverse kinematics for 4-wheel independent drive
    with dual steering disk configuration
    """
    
    def __init__(self, geometry: RobotGeometry):
        self.geo = geometry
        
        # Wheel positions relative to robot center
        self.wheel_positions = {
            'LF': np.array([self.geo.wheelbase/2, self.geo.track_width/2]),    # Left Front
            'LR': np.array([-self.geo.wheelbase/2, self.geo.track_width/2]),   # Left Rear  
            'RF': np.array([self.geo.wheelbase/2, -self.geo.track_width/2]),   # Right Front
            'RR': np.array([-self.geo.wheelbase/2, -self.geo.track_width/2])   # Right Rear
        }
        
    def compute_wheel_steering_angles(self, theta_L: float, theta_R: float) -> Tuple[float, float, float, float]:
        """
        Compute individual wheel steering angles from steering disk angles
        
        Args:
            theta_L: Left steering disk angle (radians)
            theta_R: Right steering disk angle (radians)
            
        Returns:
            Tuple of (delta_LF, delta_LR, delta_RF, delta_RR) steering angles
        """
        # Steering linkage: front wheels steer inward, rear wheels steer outward
        # with equal magnitude but opposite signs
        delta_LF = +theta_L   # Left front steers inward (positive)
        delta_LR = -theta_L   # Left rear steers outward (negative)
        delta_RF = +theta_R   # Right front steers inward (positive) 
        delta_RR = -theta_R   # Right rear steers outward (negative)
        
        return delta_LF, delta_LR, delta_RF, delta_RR
        
    def forward_kinematics(self, robot_state: RobotState) -> Tuple[float, float, float]:
        """
        Compute robot velocity from individual wheel velocities and steering angles
        Using the exact kinematic model formulas
        
        Args:
            robot_state: Current robot state with wheel velocities and steering
            
        Returns:
            Tuple of (v_x_robot, v_y_robot, omega_robot) in robot frame
        """
        # Get wheel steering angles from steering disks
        delta_LF, delta_LR, delta_RF, delta_RR = self.compute_wheel_steering_angles(
            robot_state.theta_L, robot_state.theta_R
        )
        
        # Wheel velocities  
        v_LF = robot_state.left_front.velocity
        v_LR = robot_state.left_rear.velocity
        v_RF = robot_state.right_front.velocity
        v_RR = robot_state.right_rear.velocity
        
        # Robot's forward (x) and lateral (y) velocities using exact formulas:
        # vx_robot = (1/4) * Σ(vi * cos(δi))
        # vy_robot = (1/4) * Σ(vi * sin(δi))
        v_x_robot = 0.25 * (v_LF * np.cos(delta_LF) + 
                           v_LR * np.cos(delta_LR) + 
                           v_RF * np.cos(delta_RF) + 
                           v_RR * np.cos(delta_RR))
                    
        v_y_robot = 0.25 * (v_LF * np.sin(delta_LF) + 
                           v_LR * np.sin(delta_LR) + 
                           v_RF * np.sin(delta_RF) + 
                           v_RR * np.sin(delta_RR))
        
        # Angular velocity using exact formula:
        # ω = [(vy,LF + vy,RF) - (vy,LR + vy,RR)] / (2 × L)
        omega_robot = self._compute_angular_velocity(
            [v_LF, v_LR, v_RF, v_RR],
            [delta_LF, delta_LR, delta_RF, delta_RR]
        )
        
        return v_x_robot, v_y_robot, omega_robot
        
    def _compute_angular_velocity(self, velocities: list, steering_angles: list) -> float:
        """
        Compute angular velocity using the exact formula from the kinematic model:
        ω = [(vy,LF + vy,RF) - (vy,LR + vy,RR)] / (2 × L)
        """
        # Extract individual wheel velocities
        v_LF, v_LR, v_RF, v_RR = velocities
        delta_LF, delta_LR, delta_RF, delta_RR = steering_angles
        
        # Compute lateral (y) velocity components for each wheel
        vy_LF = v_LF * np.sin(delta_LF)
        vy_LR = v_LR * np.sin(delta_LR)
        vy_RF = v_RF * np.sin(delta_RF)
        vy_RR = v_RR * np.sin(delta_RR)
        
        # Apply the exact angular velocity formula
        omega = ((vy_LF + vy_RF) - (vy_LR + vy_RR)) / (2.0 * self.geo.wheelbase)
        
        return omega
        
    def inverse_kinematics(self, v_x: float, v_y: float, omega: float) -> RobotState:
        """
        Compute required wheel velocities and steering angles for desired motion
        Using full Ackermann kinematics model
        
        Args:
            v_x: Desired forward velocity (m/s)
            v_y: Desired lateral velocity (m/s) - should be minimal for Ackermann  
            omega: Desired angular velocity (rad/s)
            
        Returns:
            RobotState with computed wheel velocities and steering angles
        """
        robot_state = RobotState()
        
        if abs(omega) < 1e-6:
            # Straight line motion: θ_L = θ_R = 0, all wheels parallel
            robot_state.theta_L = 0.0
            robot_state.theta_R = 0.0
            
            # All wheels same velocity for straight motion
            # Account for left motor direction inversion
            robot_state.left_front.velocity = -v_x    # Invert left motors
            robot_state.left_rear.velocity = -v_x     # Invert left motors
            robot_state.right_front.velocity = v_x    # Right motors normal
            robot_state.right_rear.velocity = v_x     # Right motors normal
            
        elif abs(v_x) < 1e-6:
            # Pure rotation in place: θ_L = -θ_R ≈ max angle, v_L = -v_R
            max_steer = self.geo.max_steer_angle
            
            if omega > 0:  # Left turn
                robot_state.theta_L = max_steer    # Left disk positive
                robot_state.theta_R = -max_steer   # Right disk negative
            else:  # Right turn
                robot_state.theta_L = -max_steer   # Left disk negative  
                robot_state.theta_R = max_steer    # Right disk positive
                
            # Compute tangential velocity for rotation
            # Use track_width/2 as the radius for rotation about center
            tangential_vel = abs(omega) * self.geo.track_width / 2.0
            
            if omega > 0:  # Left turn
                # For left turn: left wheels backward, right wheels forward
                # Account for left motor direction inversion
                robot_state.left_front.velocity = tangential_vel   # Left backward becomes positive due to motor inversion
                robot_state.left_rear.velocity = tangential_vel    # Left backward becomes positive due to motor inversion  
                robot_state.right_front.velocity = tangential_vel  # Right forward stays positive
                robot_state.right_rear.velocity = tangential_vel   # Right forward stays positive
            else:  # Right turn
                # For right turn: left wheels forward, right wheels backward
                # Account for left motor direction inversion
                robot_state.left_front.velocity = -tangential_vel  # Left forward becomes negative due to motor inversion
                robot_state.left_rear.velocity = -tangential_vel   # Left forward becomes negative due to motor inversion
                robot_state.right_front.velocity = -tangential_vel # Right backward stays negative
                robot_state.right_rear.velocity = -tangential_vel  # Right backward stays negative
                
        else:
            # Forward motion with turning: Full Ackermann geometry
            # Compute turning radius from desired motion
            R = v_x / omega  # Turning radius (can be negative)
            abs_R = abs(R)
            
            # Compute Ackermann steering angles based on turning radius
            # Both sides steer toward the turn center
            if omega > 0:  # Left turn (positive omega)
                # Left side is inner (smaller radius), right side is outer
                if abs_R > self.geo.track_width/2:
                    robot_state.theta_L = np.arctan(self.geo.wheelbase / (abs_R - self.geo.track_width/2))
                    robot_state.theta_R = np.arctan(self.geo.wheelbase / (abs_R + self.geo.track_width/2))
                else:
                    # Very tight turn, use maximum angles
                    robot_state.theta_L = self.geo.max_steer_angle
                    robot_state.theta_R = self.geo.max_steer_angle * 0.5
            else:  # Right turn (negative omega)
                # Right side is inner (smaller radius), left side is outer
                if abs_R > self.geo.track_width/2:
                    robot_state.theta_L = -np.arctan(self.geo.wheelbase / (abs_R + self.geo.track_width/2))
                    robot_state.theta_R = -np.arctan(self.geo.wheelbase / (abs_R - self.geo.track_width/2))
                else:
                    # Very tight turn, use maximum angles
                    robot_state.theta_L = -self.geo.max_steer_angle * 0.5
                    robot_state.theta_R = -self.geo.max_steer_angle
                    
            # Clamp steering angles to limits
            robot_state.theta_L = np.clip(robot_state.theta_L, -self.geo.max_steer_angle, self.geo.max_steer_angle)
            robot_state.theta_R = np.clip(robot_state.theta_R, -self.geo.max_steer_angle, self.geo.max_steer_angle)
            
            # Compute wheel velocities based on Ackermann geometry
            # Each wheel's distance from the instantaneous center of rotation
            if omega > 0:  # Left turn
                R_left = abs_R - self.geo.track_width/2.0
                R_right = abs_R + self.geo.track_width/2.0
            else:  # Right turn
                R_left = abs_R + self.geo.track_width/2.0  
                R_right = abs_R - self.geo.track_width/2.0
                
            # Compute angular velocity for each side
            omega_magnitude = abs(omega)
            
            # Left side wheels
            if R_left > 1e-6:
                v_left = omega_magnitude * R_left * np.sign(R)
            else:
                v_left = 0.0
                
            # Right side wheels  
            if R_right > 1e-6:
                v_right = omega_magnitude * R_right * np.sign(R)
            else:
                v_right = 0.0
                
            robot_state.left_front.velocity = -v_left   # Invert left motors
            robot_state.left_rear.velocity = -v_left    # Invert left motors
            robot_state.right_front.velocity = v_right  # Right motors normal
            robot_state.right_rear.velocity = v_right   # Right motors normal
            
        return robot_state

class KinematicsNode(Node):
    """ROS 2 node for robot kinematics computation and odometry"""
    
    def __init__(self):
        super().__init__('kinematics_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot.wheelbase', 0.64),
                ('robot.track_width', 0.6),
                ('robot.wheel_radius', 0.1),
                ('robot.max_steering_angle', 28.0),
                ('control.update_rate', 50.0),
            ]
        )
        
        # Get parameters
        self.geometry = RobotGeometry(
            wheelbase=self.get_parameter('robot.wheelbase').value,
            track_width=self.get_parameter('robot.track_width').value,
            wheel_radius=self.get_parameter('robot.wheel_radius').value,
            max_steer_angle=math.radians(self.get_parameter('robot.max_steering_angle').value)
        )
        
        self.update_rate = self.get_parameter('control.update_rate').value
        
        # Initialize kinematics
        self.kinematics = AckermannKinematics(self.geometry)
        self.robot_state = RobotState()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, 'wheel_commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.wheel_state_sub = self.create_subscription(
            Float64MultiArray, 'wheel_states', self.wheel_state_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for odometry updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_odometry)
        
        self.get_logger().info('Ackermann Kinematics Node initialized')
        
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands and compute wheel commands"""
        try:
            # Extract desired velocities
            v_x = msg.linear.x
            v_y = msg.linear.y  # Should be minimal for Ackermann
            omega = msg.angular.z
            
            # Apply sign correction if needed (uncomment if steering is inverted)
            # omega = -omega  # Invert if left command makes robot turn right
            
            # Compute required wheel states
            target_state = self.kinematics.inverse_kinematics(v_x, v_y, omega)
            
            # Publish wheel commands
            self.publish_wheel_commands(target_state)
            
            # Log for debugging
            self.get_logger().debug(
                f'Cmd: vx={v_x:.2f}, vy={v_y:.2f}, ω={omega:.2f} -> '
                f'θL={math.degrees(target_state.theta_L):.1f}°, '
                f'θR={math.degrees(target_state.theta_R):.1f}°'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel callback: {e}')
            
    def wheel_state_callback(self, msg: Float64MultiArray):
        """Update robot state from wheel feedback"""
        try:
            if len(msg.data) >= 8:  # 4 velocities + 4 steering angles (or RPMs)
                # Update wheel velocities (convert from RPM to m/s if needed)
                self.robot_state.left_front.velocity = msg.data[0]
                self.robot_state.left_rear.velocity = msg.data[1] 
                self.robot_state.right_front.velocity = msg.data[2]
                self.robot_state.right_rear.velocity = msg.data[3]
                
                # Update steering angles if provided
                if len(msg.data) >= 6:
                    self.robot_state.theta_L = msg.data[4]
                    self.robot_state.theta_R = msg.data[5]
                    
        except Exception as e:
            self.get_logger().error(f'Error in wheel_state callback: {e}')
            
    def publish_wheel_commands(self, target_state: RobotState):
        """Publish computed wheel commands"""
        msg = Float64MultiArray()
        
        # Format: [v_LF, v_LR, v_RF, v_RR, theta_L, theta_R]
        msg.data = [
            target_state.left_front.velocity,
            target_state.left_rear.velocity,
            target_state.right_front.velocity, 
            target_state.right_rear.velocity,
            target_state.theta_L,
            target_state.theta_R
        ]
        
        self.wheel_cmd_pub.publish(msg)
        
    def update_odometry(self):
        """Compute and publish odometry"""
        try:
            # Compute robot velocity from current wheel states
            v_x, v_y, omega = self.kinematics.forward_kinematics(self.robot_state)
            
            # Update robot state
            dt = 1.0 / self.update_rate
            self.robot_state.v_x = v_x
            self.robot_state.v_y = v_y
            self.robot_state.omega = omega
            
            # Integrate to get pose (simple Euler integration)
            cos_h = np.cos(self.robot_state.heading)
            sin_h = np.sin(self.robot_state.heading)
            
            # Transform robot-frame velocity to world frame
            v_x_world = v_x * cos_h - v_y * sin_h
            v_y_world = v_x * sin_h + v_y * cos_h
            
            # Update pose
            self.robot_state.x += v_x_world * dt
            self.robot_state.y += v_y_world * dt
            self.robot_state.heading += omega * dt
            
            # Normalize heading
            self.robot_state.heading = np.arctan2(
                np.sin(self.robot_state.heading),
                np.cos(self.robot_state.heading)
            )
            
            # Publish odometry
            self.publish_odometry()
            self.publish_tf()
            self.publish_joint_states()
            
        except Exception as e:
            self.get_logger().error(f'Error updating odometry: {e}')
            
    def publish_odometry(self):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.robot_state.x
        odom.pose.pose.position.y = self.robot_state.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from heading)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(self.robot_state.heading / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.robot_state.heading / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = self.robot_state.v_x
        odom.twist.twist.linear.y = self.robot_state.v_y
        odom.twist.twist.angular.z = self.robot_state.omega
        
        self.odom_pub.publish(odom)
        
    def publish_tf(self):
        """Publish transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.robot_state.x
        t.transform.translation.y = self.robot_state.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(self.robot_state.heading / 2.0)
        t.transform.rotation.w = np.cos(self.robot_state.heading / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
        
    def publish_joint_states(self):
        """Publish joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Wheel joint names
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
        
        # Get wheel steering angles
        delta_LF, delta_LR, delta_RF, delta_RR = self.kinematics.compute_wheel_steering_angles(
            self.robot_state.theta_L, self.robot_state.theta_R
        )
        
        # Wheel positions (integration of velocity)
        # This is simplified - in real system would come from encoders
        dt = 1.0 / self.update_rate
        wheel_positions = [
            0.0, 0.0, 0.0, 0.0  # Placeholder wheel positions
        ]
        
        joint_state.position = wheel_positions + [delta_LF, delta_LR, delta_RF, delta_RR]
        
        # Wheel velocities
        joint_state.velocity = [
            self.robot_state.left_front.velocity / self.geometry.wheel_radius,
            self.robot_state.left_rear.velocity / self.geometry.wheel_radius,
            self.robot_state.right_front.velocity / self.geometry.wheel_radius, 
            self.robot_state.right_rear.velocity / self.geometry.wheel_radius,
            0.0, 0.0, 0.0, 0.0  # Steering velocities
        ]
        
        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    
    node = KinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()