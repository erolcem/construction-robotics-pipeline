#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_msgs.msg import Float64MultiArray, String
import json
import time
from typing import Dict, List, Optional


class StateMonitor(Node):
    """
    Robot state monitoring node that tracks and logs robot status.
    Provides real-time monitoring of joint states, pose, forces, and system status.
    """
    
    def __init__(self):
        super().__init__('state_monitor')
        
        # Parameters
        self.declare_parameter('log_frequency', 10.0)
        self.declare_parameter('save_logs', True)
        self.declare_parameter('log_file_path', '/workspace/logs/robot_state.log')
        
        self.log_frequency = self.get_parameter('log_frequency').value
        self.save_logs = self.get_parameter('save_logs').value
        self.log_file_path = self.get_parameter('log_file_path').value
        
        # State variables
        self.joint_states: Optional[JointState] = None
        self.current_pose: Optional[PoseStamped] = None
        self.force_torque: Optional[WrenchStamped] = None
        self.robot_mode: str = 'UNKNOWN'
        self.safety_status: str = 'UNKNOWN'
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.pose_callback,
            10
        )
        
        self.ft_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor',
            self.force_torque_callback,
            10
        )
        
        self.robot_mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.robot_mode_callback,
            10
        )
        
        self.safety_status_sub = self.create_subscription(
            String,
            '/safety_status',
            self.safety_status_callback,
            10
        )
        
        # Publishers for processed data
        self.state_summary_pub = self.create_publisher(
            String,
            '/robot_state_summary',
            10
        )
        
        self.joint_velocities_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_velocities_filtered',
            10
        )
        
        # Monitoring timer
        self.monitor_timer = self.create_timer(
            1.0 / self.log_frequency,
            self.monitor_callback
        )
        
        # Data storage for analysis
        self.state_history: List[Dict] = []
        self.max_history_size = 1000
        
        self.get_logger().info('State Monitor initialized')
        self.get_logger().info(f'Logging frequency: {self.log_frequency} Hz')
        if self.save_logs:
            self.get_logger().info(f'Saving logs to: {self.log_file_path}')
    
    def joint_state_callback(self, msg: JointState):
        """Process joint state updates."""
        self.joint_states = msg
        
        # Publish filtered velocities if needed
        if msg.velocity:
            velocity_msg = Float64MultiArray()
            velocity_msg.data = list(msg.velocity)
            self.joint_velocities_pub.publish(velocity_msg)
    
    def pose_callback(self, msg: PoseStamped):
        """Process current pose updates."""
        self.current_pose = msg
    
    def force_torque_callback(self, msg: WrenchStamped):
        """Process force/torque sensor data."""
        self.force_torque = msg
    
    def robot_mode_callback(self, msg: String):
        """Process robot mode updates."""
        self.robot_mode = msg.data
    
    def safety_status_callback(self, msg: String):
        """Process safety status updates."""
        self.safety_status = msg.data
    
    def monitor_callback(self):
        """Main monitoring loop."""
        current_time = time.time()
        
        # Collect current state
        state_data = {
            'timestamp': current_time,
            'robot_mode': self.robot_mode,
            'safety_status': self.safety_status
        }
        
        # Add joint data if available
        if self.joint_states:
            state_data['joint_positions'] = list(self.joint_states.position)
            state_data['joint_velocities'] = list(self.joint_states.velocity) if self.joint_states.velocity else []
            state_data['joint_efforts'] = list(self.joint_states.effort) if self.joint_states.effort else []
        
        # Add pose data if available
        if self.current_pose:
            pose = self.current_pose.pose
            state_data['position'] = [pose.position.x, pose.position.y, pose.position.z]
            state_data['orientation'] = [
                pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w
            ]
        
        # Add force/torque data if available
        if self.force_torque:
            wrench = self.force_torque.wrench
            state_data['force'] = [wrench.force.x, wrench.force.y, wrench.force.z]
            state_data['torque'] = [wrench.torque.x, wrench.torque.y, wrench.torque.z]
        
        # Store in history
        self.state_history.append(state_data)
        if len(self.state_history) > self.max_history_size:
            self.state_history.pop(0)
        
        # Create and publish summary
        summary = self.create_state_summary(state_data)
        summary_msg = String()
        summary_msg.data = json.dumps(summary)
        self.state_summary_pub.publish(summary_msg)
        
        # Log to file if enabled
        if self.save_logs:
            self.log_to_file(state_data)
        
        # Check for anomalies
        self.check_anomalies(state_data)
    
    def create_state_summary(self, state_data: Dict) -> Dict:
        """Create a summary of the current robot state."""
        summary = {
            'timestamp': state_data['timestamp'],
            'robot_mode': state_data['robot_mode'],
            'safety_status': state_data['safety_status'],
            'is_moving': self.is_robot_moving(),
            'joint_count': len(state_data.get('joint_positions', [])),
        }
        
        # Add position info if available
        if 'position' in state_data:
            summary['end_effector_position'] = state_data['position']
        
        # Add velocity info if available
        if 'joint_velocities' in state_data and state_data['joint_velocities']:
            max_velocity = max(abs(v) for v in state_data['joint_velocities'])
            summary['max_joint_velocity'] = max_velocity
        
        return summary
    
    def is_robot_moving(self) -> bool:
        """Check if robot is currently moving."""
        if not self.joint_states or not self.joint_states.velocity:
            return False
        
        velocity_threshold = 0.01  # rad/s
        return any(abs(v) > velocity_threshold for v in self.joint_states.velocity)
    
    def check_anomalies(self, state_data: Dict):
        """Check for potential issues or anomalies."""
        warnings = []
        
        # Check joint velocities
        if 'joint_velocities' in state_data and state_data['joint_velocities']:
            max_velocity = max(abs(v) for v in state_data['joint_velocities'])
            if max_velocity > 3.0:  # rad/s
                warnings.append(f"High joint velocity detected: {max_velocity:.2f} rad/s")
        
        # Check force/torque
        if 'force' in state_data:
            max_force = max(abs(f) for f in state_data['force'])
            if max_force > 100.0:  # N
                warnings.append(f"High force detected: {max_force:.2f} N")
        
        # Check safety status
        if self.safety_status not in ['NORMAL', 'UNKNOWN']:
            warnings.append(f"Safety issue: {self.safety_status}")
        
        # Log warnings
        for warning in warnings:
            self.get_logger().warn(warning)
    
    def log_to_file(self, state_data: Dict):
        """Log state data to file."""
        try:
            with open(self.log_file_path, 'a') as f:
                f.write(json.dumps(state_data) + '\n')
        except Exception as e:
            self.get_logger().error(f"Failed to write log file: {e}")
    
    def get_state_history(self, duration_seconds: float = 60.0) -> List[Dict]:
        """Get recent state history."""
        current_time = time.time()
        cutoff_time = current_time - duration_seconds
        
        return [state for state in self.state_history if state['timestamp'] > cutoff_time]


def main(args=None):
    rclpy.init(args=args)
    
    monitor = StateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('State monitor shutting down...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
