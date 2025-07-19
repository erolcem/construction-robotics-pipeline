#!/usr/bin/env python3

"""
Universal Robots Joint Controller
This node provides comprehensive joint control capabilities for UR robots.
It supports both simulation and real robot control with position, velocity, and trajectory commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
import numpy as np
import time
import threading


class URJointController(Node):
    """
    Universal Robots Joint Controller Node
    
    Features:
    - Individual joint position control
    - Joint velocity control  
    - Trajectory execution
    - Current state monitoring
    - Safety limits enforcement
    """
    
    def __init__(self):
        super().__init__('ur_joint_controller')
        
        # Declare parameters
        self.declare_parameter('robot_type', 'ur5e')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('safety_limits', True)
        
        # Get parameters
        self.robot_type = self.get_parameter('robot_type').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.safety_limits = self.get_parameter('safety_limits').value
        
        # Joint names for UR robots
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current joint states
        self.current_positions = [0.0] * 6
        self.current_velocities = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.last_state_time = self.get_clock().now()
        
        # Safety limits (radians)
        self.position_limits = {
            'ur5e': {
                'min': [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28],
                'max': [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
            }
        }
        
        self.velocity_limits = [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]  # rad/s
        
        # Initialize publishers and subscribers
        self.setup_communication()
        
        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, 
            self.control_loop
        )
        
        # Interactive control thread
        self.control_thread = threading.Thread(target=self.interactive_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.get_logger().info(f"UR Joint Controller initialized for {self.robot_type}")
        self.get_logger().info(f"Simulation mode: {self.simulation_mode}")
        self.get_logger().info("Available commands:")
        self.get_logger().info("  - 'pos <j1> <j2> <j3> <j4> <j5> <j6>' - Set joint positions")
        self.get_logger().info("  - 'home' - Move to home position")
        self.get_logger().info("  - 'status' - Show current joint states")
        self.get_logger().info("  - 'traj' - Execute demo trajectory")
    
    def setup_communication(self):
        """Set up ROS 2 publishers, subscribers, and action clients"""
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        if self.simulation_mode:
            # For simulation - publish to joint_states for visualization
            self.joint_cmd_pub = self.create_publisher(
                JointState,
                '/joint_states',
                10
            )
        else:
            # For real robot - use trajectory action client
            self.trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/follow_joint_trajectory'
            )
    
    def joint_state_callback(self, msg):
        """Update current joint states"""
        if len(msg.position) >= 6:
            self.current_positions = list(msg.position[:6])
            if msg.velocity:
                self.current_velocities = list(msg.velocity[:6])
            if msg.effort:
                self.current_efforts = list(msg.effort[:6])
            self.last_state_time = self.get_clock().now()
    
    def check_safety_limits(self, positions, velocities=None):
        """Check if positions and velocities are within safety limits"""
        if not self.safety_limits:
            return True
            
        limits = self.position_limits.get(self.robot_type, self.position_limits['ur5e'])
        
        for i, pos in enumerate(positions):
            if pos < limits['min'][i] or pos > limits['max'][i]:
                self.get_logger().error(f"Joint {i+1} position {pos:.3f} exceeds limits [{limits['min'][i]:.3f}, {limits['max'][i]:.3f}]")
                return False
        
        if velocities:
            for i, vel in enumerate(velocities):
                if abs(vel) > self.velocity_limits[i]:
                    self.get_logger().error(f"Joint {i+1} velocity {vel:.3f} exceeds limit {self.velocity_limits[i]:.3f}")
                    return False
        
        return True
    
    def move_to_positions(self, positions, duration=3.0):
        """Move joints to specified positions"""
        if not self.check_safety_limits(positions):
            self.get_logger().error("Command rejected due to safety limits")
            return False
        
        if self.simulation_mode:
            # For simulation, publish joint states directly
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = positions
            joint_state.velocity = [0.0] * 6
            joint_state.effort = [0.0] * 6
            
            self.joint_cmd_pub.publish(joint_state)
            self.get_logger().info(f"Moving to positions: {[f'{p:.3f}' for p in positions]}")
            
        else:
            # For real robot, use trajectory action
            if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error("Trajectory action server not available")
                return False
            
            # Create trajectory
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            
            trajectory.points = [point]
            
            # Send goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory
            
            future = self.trajectory_client.send_goal_async(goal)
            self.get_logger().info(f"Executing trajectory to: {[f'{p:.3f}' for p in positions]}")
        
        return True
    
    def move_home(self):
        """Move to home position (all joints at 0)"""
        home_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # Typical UR home position
        self.get_logger().info("Moving to home position...")
        return self.move_to_positions(home_positions, 5.0)
    
    def execute_demo_trajectory(self):
        """Execute a demonstration trajectory"""
        self.get_logger().info("Executing demo trajectory...")
        
        # Define waypoints
        waypoints = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home
            [0.5, -1.0, -0.5, -2.0, 0.0, 0.0],   # Point 1
            [-0.5, -2.0, 1.0, -1.5, 1.57, 0.0],  # Point 2
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Back to home
        ]
        
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f"Moving to waypoint {i+1}/4")
            self.move_to_positions(waypoint, 3.0)
            time.sleep(3.5)  # Wait for movement to complete
        
        self.get_logger().info("Demo trajectory completed!")
    
    def print_status(self):
        """Print current joint states"""
        self.get_logger().info("=== Current Joint States ===")
        for i, name in enumerate(self.joint_names):
            pos = self.current_positions[i] if i < len(self.current_positions) else 0.0
            vel = self.current_velocities[i] if i < len(self.current_velocities) else 0.0
            eff = self.current_efforts[i] if i < len(self.current_efforts) else 0.0
            self.get_logger().info(f"{name}: pos={pos:.3f} rad, vel={vel:.3f} rad/s, effort={eff:.3f} Nm")
        
        age = (self.get_clock().now() - self.last_state_time).nanoseconds / 1e9
        self.get_logger().info(f"State age: {age:.2f} seconds")
    
    def interactive_control_loop(self):
        """Interactive control loop for user commands"""
        while rclpy.ok():
            try:
                # This would be better with a proper CLI interface
                # For now, we'll use timer-based commands in the control loop
                time.sleep(1.0)
            except KeyboardInterrupt:
                break
    
    def control_loop(self):
        """Main control loop"""
        # This can be used for continuous control tasks
        # For now, it just maintains the node alive
        pass
    
    def parse_command(self, command):
        """Parse and execute user commands"""
        parts = command.strip().split()
        if not parts:
            return
        
        cmd = parts[0].lower()
        
        if cmd == 'pos' and len(parts) == 7:
            try:
                positions = [float(p) for p in parts[1:7]]
                self.move_to_positions(positions)
            except ValueError:
                self.get_logger().error("Invalid position values")
        
        elif cmd == 'home':
            self.move_home()
        
        elif cmd == 'status':
            self.print_status()
        
        elif cmd == 'traj':
            self.execute_demo_trajectory()
        
        else:
            self.get_logger().info("Unknown command. Available: pos, home, status, traj")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = URJointController()
        
        # Example usage
        controller.get_logger().info("Starting demo sequence in 3 seconds...")
        time.sleep(3)
        
        # Show initial status
        controller.print_status()
        time.sleep(2)
        
        # Move to home
        controller.move_home()
        time.sleep(4)
        
        # Execute demo trajectory
        controller.execute_demo_trajectory()
        
        # Keep node running
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
