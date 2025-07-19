#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from typing import List, Optional


class RobotController(Node):
    """
    Main robot controller node for Universal Robots.
    Provides high-level control interface for robot movements.
    """
    
    def __init__(self):
        super().__init__('robot_controller')
        
        # Parameters
        self.declare_parameter('robot_model', 'ur5e')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('control_frequency', 50.0)
        
        self.robot_model = self.get_parameter('robot_model').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # Joint names for UR robots
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Publishers
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.pose_target_publisher = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current state
        self.current_joint_positions: Optional[List[float]] = None
        self.current_joint_velocities: Optional[List[float]] = None
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )
        
        self.get_logger().info(f'Robot Controller initialized for {self.robot_model}')
        self.get_logger().info(f'Simulation mode: {self.simulation_mode}')
    
    def joint_state_callback(self, msg: JointState):
        """Callback for joint state updates."""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])
            self.current_joint_velocities = list(msg.velocity[:6]) if msg.velocity else [0.0] * 6
    
    def control_loop(self):
        """Main control loop - override in derived classes."""
        pass
    
    def move_to_joint_positions(self, positions: List[float], duration: float = 3.0):
        """
        Move robot to specified joint positions.
        
        Args:
            positions: List of 6 joint positions in radians
            duration: Time to complete the movement
        """
        if len(positions) != 6:
            self.get_logger().error('Joint positions must contain exactly 6 values')
            return
        
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = rclpy.duration.Duration(seconds=duration).to_msg()
        
        trajectory.points = [point]
        
        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info(f'Moving to joint positions: {positions}')
    
    def move_to_pose(self, position: List[float], orientation: List[float]):
        """
        Move robot to specified Cartesian pose.
        
        Args:
            position: [x, y, z] in meters
            orientation: [x, y, z, w] quaternion
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'base_link'
        
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        
        self.pose_target_publisher.publish(pose_stamped)
        self.get_logger().info(f'Moving to pose: pos={position}, ori={orientation}')
    
    def get_current_joint_positions(self) -> Optional[List[float]]:
        """Get current joint positions."""
        return self.current_joint_positions
    
    def home_position(self):
        """Move robot to home position."""
        home_joints = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
        self.move_to_joint_positions(home_joints, 5.0)


def main(args=None):
    rclpy.init(args=args)
    
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Robot controller shutting down...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
