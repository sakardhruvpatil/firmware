#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import threading
from collections import deque


class RobotVisualization:
    """
    Comprehensive 2D visualization for 4-wheel independent drive Ackermann robot
    Shows chassis, wheels, steering angles, RPMs, and real-time telemetry
    """
    
    def __init__(self):
        # Robot physical parameters (from config)
        self.wheelbase = 0.64  # meters
        self.track_width = 0.6  # meters
        self.wheel_radius = 0.1  # meters
        self.max_steering_angle = 28.0  # degrees
        
        # Visualization parameters
        self.chassis_length = self.wheelbase + 0.2
        self.chassis_width = self.track_width + 0.1
        
        # Robot state
        self.position = [0.0, 0.0, 0.0]  # x, y, theta
        self.velocity = [0.0, 0.0]  # linear, angular
        self.wheel_rpms = [0.0, 0.0, 0.0, 0.0]  # LF, LR, RF, RR
        self.steering_angles = [0.0, 0.0]  # left_side, right_side
        self.current_cmd = [0.0, 0.0]  # cmd_vel linear, angular
        
        # Trajectory tracking
        self.trajectory = deque(maxlen=500)
        self.trajectory.append([0.0, 0.0])
        
        # Turning center for Ackermann geometry
        self.turning_center = None
        self.wheel_vectors = []
        
        # Setup matplotlib
        self.setup_visualization()
        
    def setup_visualization(self):
        """Initialize matplotlib figure and subplots"""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('Ackermann Robot Visualization - Ready for Hardware Validation', fontsize=16, fontweight='bold')
        
        # Main robot view (2D top-down)
        self.ax_main = plt.subplot2grid((3, 4), (0, 0), colspan=2, rowspan=2)
        self.ax_main.set_xlim(-3, 3)
        self.ax_main.set_ylim(-3, 3)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_title('Robot Top-Down View', fontweight='bold')
        self.ax_main.set_xlabel('X (meters)')
        self.ax_main.set_ylabel('Y (meters)')
        
        # Telemetry displays
        self.ax_telemetry = plt.subplot2grid((3, 4), (0, 2), colspan=2)
        self.ax_telemetry.axis('off')
        self.ax_telemetry.set_title('Real-Time Telemetry', fontweight='bold')
        
        # Wheel RPM display
        self.ax_wheels = plt.subplot2grid((3, 4), (1, 2), colspan=2)
        self.ax_wheels.set_title('Wheel RPMs & Steering Angles', fontweight='bold')
        self.ax_wheels.set_ylim(-3000, 3000)
        
        # Command and geometry display
        self.ax_geometry = plt.subplot2grid((3, 4), (2, 0), colspan=4)
        self.ax_geometry.axis('off')
        self.ax_geometry.set_title('Ackermann Geometry & Commands', fontweight='bold')
        
        # Initialize plot elements
        self.init_plot_elements()
        
    def init_plot_elements(self):
        """Initialize all visual elements"""
        # Robot chassis
        self.chassis_patch = patches.Rectangle(
            (-self.chassis_length/2, -self.chassis_width/2),
            self.chassis_length, self.chassis_width,
            linewidth=2, edgecolor='blue', facecolor='lightblue', alpha=0.7
        )
        self.ax_main.add_patch(self.chassis_patch)
        
        # Wheels (4 drive wheels)
        self.wheel_patches = []
        self.wheel_positions = [
            (self.wheelbase/2, self.track_width/2),    # Left Front
            (-self.wheelbase/2, self.track_width/2),   # Left Rear  
            (self.wheelbase/2, -self.track_width/2),   # Right Front
            (-self.wheelbase/2, -self.track_width/2)   # Right Rear
        ]
        
        wheel_names = ['LF', 'LR', 'RF', 'RR']
        colors = ['red', 'orange', 'green', 'purple']
        
        for i, (pos, name, color) in enumerate(zip(self.wheel_positions, wheel_names, colors)):
            wheel = patches.Circle(pos, self.wheel_radius, 
                                 linewidth=2, edgecolor=color, 
                                 facecolor=color, alpha=0.6)
            self.ax_main.add_patch(wheel)
            self.wheel_patches.append(wheel)
            
            # Wheel labels
            self.ax_main.text(pos[0], pos[1], name, ha='center', va='center', 
                            fontweight='bold', fontsize=8)
        
        # Steering indicators (left and right side)
        self.steering_indicators = []
        steering_positions = [
            (0, self.track_width/2 + 0.15),   # Left side steering
            (0, -self.track_width/2 - 0.15)   # Right side steering
        ]
        
        for pos in steering_positions:
            indicator = patches.FancyArrow(pos[0], pos[1], 0.2, 0, 
                                         width=0.05, head_width=0.1, 
                                         head_length=0.05, fc='magenta', ec='magenta')
            self.ax_main.add_patch(indicator)
            self.steering_indicators.append(indicator)
        
        # Trajectory line
        self.trajectory_line, = self.ax_main.plot([], [], 'b--', alpha=0.5, linewidth=1, label='Trajectory')
        
        # Velocity vector
        self.velocity_arrow = patches.FancyArrow(0, 0, 0, 0, width=0.02, head_width=0.08, 
                                               head_length=0.1, fc='red', ec='red', alpha=0.8)
        self.ax_main.add_patch(self.velocity_arrow)
        
        # Turning center and geometry
        self.turning_center_point, = self.ax_main.plot([], [], 'ko', markersize=8, label='Turning Center')
        self.wheel_vector_lines = []
        for _ in range(4):
            line, = self.ax_main.plot([], [], 'g-', linewidth=2, alpha=0.7)
            self.wheel_vector_lines.append(line)
        
        # RPM bars
        self.rpm_bars = self.ax_wheels.bar(range(4), [0, 0, 0, 0], 
                                          color=['red', 'orange', 'green', 'purple'], alpha=0.7)
        self.ax_wheels.set_xticks(range(4))
        self.ax_wheels.set_xticklabels(['LF', 'LR', 'RF', 'RR'])
        self.ax_wheels.set_ylabel('RPM')
        self.ax_wheels.grid(True, alpha=0.3)
        
        # Legend
        self.ax_main.legend(loc='upper right')
        
    def update_robot_state(self, position=None, velocity=None, wheel_rpms=None, 
                          steering_angles=None, current_cmd=None):
        """Update robot state data"""
        if position is not None:
            self.position = position
            self.trajectory.append([position[0], position[1]])
            
        if velocity is not None:
            self.velocity = velocity
            
        if wheel_rpms is not None:
            self.wheel_rpms = wheel_rpms
            
        if steering_angles is not None:
            self.steering_angles = steering_angles
            
        if current_cmd is not None:
            self.current_cmd = current_cmd
    
    def calculate_ackermann_geometry(self):
        """Calculate Ackermann steering geometry"""
        if abs(self.steering_angles[0]) > 0.1 or abs(self.steering_angles[1]) > 0.1:
            # Average steering angle for geometry calculation
            avg_steering = (self.steering_angles[0] + self.steering_angles[1]) / 2.0
            
            if abs(avg_steering) > 0.1:  # Only if significant steering
                # Calculate turning radius
                turning_radius = self.wheelbase / math.tan(math.radians(avg_steering))
                
                # Turning center position
                center_x = self.position[0] - turning_radius * math.sin(self.position[2])
                center_y = self.position[1] + turning_radius * math.cos(self.position[2])
                self.turning_center = [center_x, center_y]
                
                # Calculate wheel vectors
                self.wheel_vectors = []
                for i, (wheel_x, wheel_y) in enumerate(self.wheel_positions):
                    # Transform wheel position to global coordinates
                    global_x = self.position[0] + wheel_x * math.cos(self.position[2]) - wheel_y * math.sin(self.position[2])
                    global_y = self.position[1] + wheel_x * math.sin(self.position[2]) + wheel_y * math.cos(self.position[2])
                    
                    # Vector from wheel to turning center
                    vec_x = center_x - global_x
                    vec_y = center_y - global_y
                    vec_length = math.sqrt(vec_x**2 + vec_y**2)
                    
                    if vec_length > 0:
                        # Normalize and scale for visualization
                        vec_x = (vec_x / vec_length) * 0.3
                        vec_y = (vec_y / vec_length) * 0.3
                        
                    self.wheel_vectors.append([(global_x, global_y), (global_x + vec_x, global_y + vec_y)])
            else:
                self.turning_center = None
                self.wheel_vectors = []
        else:
            self.turning_center = None
            self.wheel_vectors = []
    
    def update_visualization(self):
        """Update all visual elements"""
        # Update robot position and orientation
        x, y, theta = self.position
        
        # Update chassis
        self.chassis_patch.set_xy((x - self.chassis_length/2, y - self.chassis_width/2))
        transform = plt.matplotlib.transforms.Affine2D().rotate_around(x, y, theta) + self.ax_main.transData
        self.chassis_patch.set_transform(transform)
        
        # Update wheels
        for i, wheel in enumerate(self.wheel_patches):
            wheel_x, wheel_y = self.wheel_positions[i]
            global_x = x + wheel_x * math.cos(theta) - wheel_y * math.sin(theta)
            global_y = y + wheel_x * math.sin(theta) + wheel_y * math.cos(theta)
            wheel.center = (global_x, global_y)
            
            # Color intensity based on RPM
            rpm = self.wheel_rpms[i]
            intensity = min(abs(rpm) / 2500.0, 1.0)  # Normalize to max RPM
            colors = ['red', 'orange', 'green', 'purple']
            wheel.set_alpha(0.3 + 0.7 * intensity)
        
        # Update steering indicators
        left_angle = math.radians(self.steering_angles[0])
        right_angle = math.radians(self.steering_angles[1])
        
        for i, (indicator, angle) in enumerate(zip(self.steering_indicators, [left_angle, right_angle])):
            indicator.remove()
            
        # Recreate steering indicators with new angles
        steering_positions = [
            (x, y + self.track_width/2 + 0.15),   # Left side
            (x, y - self.track_width/2 - 0.15)    # Right side
        ]
        
        self.steering_indicators.clear()
        for i, (pos, angle) in enumerate(zip(steering_positions, [left_angle, right_angle])):
            arrow_x = 0.2 * math.cos(theta + angle)
            arrow_y = 0.2 * math.sin(theta + angle)
            
            indicator = patches.FancyArrow(pos[0], pos[1], arrow_x, arrow_y,
                                         width=0.05, head_width=0.1, head_length=0.05,
                                         fc='magenta', ec='magenta')
            self.ax_main.add_patch(indicator)
            self.steering_indicators.append(indicator)
        
        # Update trajectory
        if len(self.trajectory) > 1:
            traj_x, traj_y = zip(*self.trajectory)
            self.trajectory_line.set_data(traj_x, traj_y)
        
        # Update velocity vector
        if abs(self.velocity[0]) > 0.1:
            vel_scale = 0.5
            vel_x = vel_scale * self.velocity[0] * math.cos(theta)
            vel_y = vel_scale * self.velocity[0] * math.sin(theta)
            
            self.velocity_arrow.remove()
            self.velocity_arrow = patches.FancyArrow(x, y, vel_x, vel_y,
                                                   width=0.02, head_width=0.08, head_length=0.1,
                                                   fc='red', ec='red', alpha=0.8)
            self.ax_main.add_patch(self.velocity_arrow)
        
        # Update Ackermann geometry
        self.calculate_ackermann_geometry()
        
        if self.turning_center:
            self.turning_center_point.set_data([self.turning_center[0]], [self.turning_center[1]])
        else:
            self.turning_center_point.set_data([], [])
        
        # Update wheel vectors
        for i, line in enumerate(self.wheel_vector_lines):
            if i < len(self.wheel_vectors):
                start, end = self.wheel_vectors[i]
                line.set_data([start[0], end[0]], [start[1], end[1]])
            else:
                line.set_data([], [])
        
        # Update RPM bars
        for i, bar in enumerate(self.rpm_bars):
            bar.set_height(self.wheel_rpms[i])
            
        # Update telemetry text
        self.ax_telemetry.clear()
        self.ax_telemetry.axis('off')
        self.ax_telemetry.set_title('Real-Time Telemetry', fontweight='bold')
        
        telemetry_text = f"""
Position: X={x:.2f}m, Y={y:.2f}m, θ={math.degrees(theta):.1f}°
Velocity: Linear={self.velocity[0]:.2f}m/s, Angular={self.velocity[1]:.2f}rad/s
Current Cmd: Linear={self.current_cmd[0]:.2f}m/s, Angular={self.current_cmd[1]:.2f}rad/s

Wheel RPMs:
  Left Front: {self.wheel_rpms[0]:+.0f} RPM
  Left Rear:  {self.wheel_rpms[1]:+.0f} RPM
  Right Front:{self.wheel_rpms[2]:+.0f} RPM
  Right Rear: {self.wheel_rpms[3]:+.0f} RPM

Steering Angles:
  Left Side:  {self.steering_angles[0]:+.1f}°
  Right Side: {self.steering_angles[1]:+.1f}°
        """
        
        self.ax_telemetry.text(0.05, 0.95, telemetry_text, transform=self.ax_telemetry.transAxes,
                              fontsize=10, verticalalignment='top', fontfamily='monospace')
        
        # Update geometry info
        self.ax_geometry.clear()
        self.ax_geometry.axis('off')
        self.ax_geometry.set_title('Ackermann Geometry & Commands', fontweight='bold')
        
        if self.turning_center:
            turning_radius = math.sqrt((self.turning_center[0] - x)**2 + (self.turning_center[1] - y)**2)
            geometry_text = f"Turning Radius: {turning_radius:.2f}m  |  Turning Center: ({self.turning_center[0]:.2f}, {self.turning_center[1]:.2f})"
        else:
            geometry_text = "Straight Line Motion - No Turning Center"
        
        geometry_text += f"\n\nUse 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' to control the robot"
        geometry_text += f"\nSystem Status: {'🟢 READY FOR HARDWARE' if max(abs(rpm) for rpm in self.wheel_rpms) > 0 else '🟡 WAITING FOR COMMANDS'}"
        
        self.ax_geometry.text(0.5, 0.5, geometry_text, transform=self.ax_geometry.transAxes,
                             fontsize=11, ha='center', va='center', fontweight='bold')
        
        # Auto-scale main view to follow robot
        margin = 2.0
        self.ax_main.set_xlim(x - margin, x + margin)
        self.ax_main.set_ylim(y - margin, y + margin)


class RobotVisualizationNode(Node):
    """ROS 2 Node for robot visualization"""
    
    def __init__(self):
        super().__init__('robot_visualization')
        
        # Create visualization
        self.viz = RobotVisualization()
        
        # ROS 2 subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        # Animation timer
        self.animation = FuncAnimation(self.viz.fig, self.update_plot, interval=50, blit=False)
        
        self.get_logger().info('Robot Visualization Node started - Ready for teleop!')
        
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.viz.update_robot_state(current_cmd=[msg.linear.x, msg.angular.z])
        
    def odom_callback(self, msg):
        """Handle odometry data"""
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # Convert quaternion to euler angle (yaw)
        theta = 2.0 * math.atan2(orient.z, orient.w)
        
        position = [pos.x, pos.y, theta]
        velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        
        self.viz.update_robot_state(position=position, velocity=velocity)
        
    def joint_states_callback(self, msg):
        """Handle joint states (steering angles and wheel velocities)"""
        if len(msg.position) >= 8:  # 4 wheel positions + 4 steering angles
            # Extract steering angles for left and right sides
            # Steering angles are at indices 4-7: [LF, LR, RF, RR]
            delta_LF = msg.position[4]  # Left front steering
            delta_LR = msg.position[5]  # Left rear steering  
            delta_RF = msg.position[6]  # Right front steering
            delta_RR = msg.position[7]  # Right rear steering
            
            # Average steering angles for each side (convert to degrees)
            left_side_angle = math.degrees((delta_LF - delta_LR) / 2.0)  # Average of front inward + rear outward
            right_side_angle = math.degrees((delta_RF - delta_RR) / 2.0)  # Average of front inward + rear outward
            
            steering_angles = [left_side_angle, right_side_angle]
            self.viz.update_robot_state(steering_angles=steering_angles)
            
        # Extract wheel velocities and convert to RPMs
        if len(msg.velocity) >= 4:
            # First 4 velocities are wheel velocities (rad/s)
            wheel_rpms = [vel * 9.549 for vel in msg.velocity[:4]]  # Convert rad/s to RPM
            self.viz.update_robot_state(wheel_rpms=wheel_rpms)
            
    def wheel_commands_callback(self, msg):
        """Handle wheel command data (RPMs) - simplified"""
        # Get RPMs from joint states instead
        pass
    
    def update_plot(self, frame):
        """Animation update function"""
        self.viz.update_visualization()
        return []


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotVisualizationNode()
    
    # Run ROS 2 in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Show the visualization
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()