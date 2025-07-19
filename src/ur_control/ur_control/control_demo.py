#!/usr/bin/env python3

"""
ROS 2 Universal Robots Control Demonstration
This script demonstrates comprehensive robot control capabilities including:
- Joint position control
- Trajectory execution
- State monitoring
- Interactive control
- Safety monitoring
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import math
import threading
import sys


class URControlDemo(Node):
    """
    Comprehensive UR Robot Control Demonstration
    
    This node showcases various control patterns and monitoring capabilities
    that can be used as a foundation for more complex robotics applications.
    """
    
    def __init__(self):
        super().__init__('ur_control_demo')
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/joint_command', 10
        )
        self.demo_status_pub = self.create_publisher(
            String, '/demo_status', 10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Current state
        self.current_joint_positions = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Demo control
        self.demo_running = False
        self.demo_thread = None
        
        # Create timer for publishing commands
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("UR Control Demo Node Started")
        self.get_logger().info("Available demonstrations:")
        self.get_logger().info("  1. Basic joint movements")
        self.get_logger().info("  2. Sinusoidal trajectories") 
        self.get_logger().info("  3. Pick and place simulation")
        self.get_logger().info("  4. Continuous monitoring")
    
    def joint_state_callback(self, msg):
        """Update current joint states from robot"""
        if len(msg.position) >= 6:
            self.current_joint_positions = list(msg.position[:6])
        if len(msg.velocity) >= 6:
            self.current_joint_velocities = list(msg.velocity[:6])
    
    def publish_joint_command(self, positions, velocities=None):
        """Publish joint command to robot"""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.joint_names
        cmd.position = positions
        cmd.velocity = velocities if velocities else [0.0] * 6
        
        self.joint_cmd_pub.publish(cmd)
    
    def publish_status(self, message):
        """Publish demo status message"""
        status_msg = String()
        status_msg.data = message
        self.demo_status_pub.publish(status_msg)
        self.get_logger().info(f"Demo Status: {message}")
    
    def wait_for_robot_ready(self, timeout=5.0):
        """Wait for robot state to be available"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if any(abs(pos) > 0.001 for pos in self.current_joint_positions):
                return True
            time.sleep(0.1)
        return False
    
    def demo_1_basic_movements(self):
        """Demonstration 1: Basic joint movements"""
        self.publish_status("Starting Demo 1: Basic Joint Movements")
        
        # Define key positions
        positions = {
            'home': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            'point_1': [0.5, -1.0, -0.5, -2.0, 0.0, 0.0],
            'point_2': [-0.5, -2.0, 1.0, -1.5, 1.57, 0.0],
            'point_3': [1.0, -0.5, -1.0, -2.5, -1.57, 0.0],
        }
        
        for name, pos in positions.items():
            self.publish_status(f"Moving to {name}")
            self.publish_joint_command(pos)
            time.sleep(3.0)  # Wait for movement
            
            # Log current position
            self.get_logger().info(f"Reached {name}: {[f'{p:.3f}' for p in self.current_joint_positions]}")
        
        self.publish_status("Demo 1 completed")
    
    def demo_2_sinusoidal_trajectory(self):
        """Demonstration 2: Sinusoidal trajectories"""
        self.publish_status("Starting Demo 2: Sinusoidal Trajectories")
        
        duration = 10.0  # seconds
        frequency = 0.2  # Hz
        amplitude = 0.5  # radians
        
        start_time = time.time()
        
        while time.time() - start_time < duration and self.demo_running:
            t = time.time() - start_time
            
            # Create sinusoidal motion for different joints
            positions = [
                amplitude * math.sin(2 * math.pi * frequency * t),  # shoulder_pan
                -1.57 + 0.3 * math.sin(2 * math.pi * frequency * t * 2),  # shoulder_lift
                amplitude * 0.5 * math.sin(2 * math.pi * frequency * t * 1.5),  # elbow
                -1.57 + 0.3 * math.sin(2 * math.pi * frequency * t * 0.8),  # wrist_1
                amplitude * 0.8 * math.sin(2 * math.pi * frequency * t * 3),  # wrist_2
                amplitude * 0.6 * math.sin(2 * math.pi * frequency * t * 2.5),  # wrist_3
            ]
            
            self.publish_joint_command(positions)
            time.sleep(0.05)  # 20 Hz control rate
        
        self.publish_status("Demo 2 completed")
    
    def demo_3_pick_and_place(self):
        """Demonstration 3: Pick and place simulation"""
        self.publish_status("Starting Demo 3: Pick and Place Simulation")
        
        # Define pick and place sequence
        sequence = [
            ('approach_pick', [0.3, -1.2, -0.8, -1.8, 1.57, 0.0]),
            ('pick_position', [0.3, -1.5, -0.5, -2.0, 1.57, 0.0]),
            ('lift_object', [0.3, -1.2, -0.8, -1.8, 1.57, 0.0]),
            ('move_to_place', [-0.3, -1.2, -0.8, -1.8, 1.57, 0.0]),
            ('place_position', [-0.3, -1.5, -0.5, -2.0, 1.57, 0.0]),
            ('release_object', [-0.3, -1.2, -0.8, -1.8, 1.57, 0.0]),
            ('return_home', [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]),
        ]
        
        for step_name, positions in sequence:
            self.publish_status(f"Pick & Place: {step_name}")
            self.publish_joint_command(positions)
            time.sleep(2.5)
            
            # Simulate gripper actions
            if 'pick' in step_name:
                self.publish_status("Gripper: Closing")
            elif 'release' in step_name:
                self.publish_status("Gripper: Opening")
        
        self.publish_status("Demo 3 completed")
    
    def demo_4_continuous_monitoring(self):
        """Demonstration 4: Continuous state monitoring"""
        self.publish_status("Starting Demo 4: Continuous Monitoring")
        
        duration = 15.0
        start_time = time.time()
        
        while time.time() - start_time < duration and self.demo_running:
            # Calculate joint velocities and accelerations
            velocities = self.current_joint_velocities
            
            # Log detailed state information
            self.get_logger().info("=== Robot State Monitor ===")
            for i, name in enumerate(self.joint_names):
                pos = self.current_joint_positions[i]
                vel = velocities[i] if i < len(velocities) else 0.0
                self.get_logger().info(f"{name}: {pos:.4f} rad ({math.degrees(pos):.2f}°), vel: {vel:.4f} rad/s")
            
            # Check for joint limits (example)
            for i, pos in enumerate(self.current_joint_positions):
                if abs(pos) > 3.14:  # Simple limit check
                    self.get_logger().warn(f"Joint {i+1} near limits: {pos:.3f} rad")
            
            time.sleep(1.0)  # 1 Hz monitoring
        
        self.publish_status("Demo 4 completed")
    
    def run_all_demos(self):
        """Run all demonstrations in sequence"""
        self.demo_running = True
        
        try:
            self.publish_status("Starting complete demonstration sequence")
            
            # Wait for robot to be ready
            if not self.wait_for_robot_ready():
                self.get_logger().error("Robot not ready - no joint states received")
                return
            
            # Run all demos
            self.demo_1_basic_movements()
            time.sleep(2)
            
            self.demo_2_sinusoidal_trajectory()
            time.sleep(2)
            
            self.demo_3_pick_and_place()
            time.sleep(2)
            
            self.demo_4_continuous_monitoring()
            
            self.publish_status("All demonstrations completed successfully!")
            
        except Exception as e:
            self.get_logger().error(f"Demo error: {e}")
            self.publish_status(f"Demo failed: {e}")
        finally:
            self.demo_running = False
    
    def start_demo_sequence(self):
        """Start the demo sequence in a separate thread"""
        if self.demo_thread and self.demo_thread.is_alive():
            self.get_logger().info("Demo already running")
            return
        
        self.demo_thread = threading.Thread(target=self.run_all_demos)
        self.demo_thread.start()
    
    def control_loop(self):
        """Main control loop - can be used for continuous control"""
        # This could implement closed-loop control, safety monitoring, etc.
        pass


def main(args=None):
    rclpy.init(args=args)
    
    try:
        demo_node = URControlDemo()
        
        # Start demo automatically after 3 seconds
        def delayed_start():
            time.sleep(3)
            demo_node.start_demo_sequence()
        
        start_thread = threading.Thread(target=delayed_start)
        start_thread.start()
        
        # Spin the node
        rclpy.spin(demo_node)
        
    except KeyboardInterrupt:
        demo_node.get_logger().info("Demo interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
