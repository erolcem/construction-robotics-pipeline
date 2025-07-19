# Usage Guide

This guide provides detailed information on how to use the Universal Robots ROS 2 control system.

## Basic Operations

### Starting the System

#### Simulation Mode
```bash
# Using the convenience script
./scripts/run_simulation.sh

# With Docker
./scripts/run_simulation.sh --docker

# Direct launch
ros2 launch launch/ur_sim_launch.py robot_model:=ur5e
```

#### Real Robot Mode
```bash
# Make sure robot is in remote control mode first
ros2 launch launch/ur_real_launch.py robot_ip:=192.168.1.100 robot_model:=ur5e
```

### Robot Control Commands

#### Basic Movement Commands
```python
# Example Python script to control the robot
import rclpy
from ur_control.robot_controller import RobotController

rclpy.init()
controller = RobotController()

# Move to home position
controller.home_position()

# Move to specific joint positions (radians)
joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
controller.move_to_joint_positions(joint_positions, duration=3.0)

# Move to Cartesian pose
position = [0.5, 0.2, 0.4]  # x, y, z in meters
orientation = [0.0, 0.0, 0.0, 1.0]  # quaternion
controller.move_to_pose(position, orientation)
```

#### Command Line Interface
```bash
# Move to home position
ros2 service call /move_to_home std_srvs/srv/Empty

# Get current joint states
ros2 topic echo /joint_states

# Monitor robot status
ros2 topic echo /robot_state_summary
```

## Advanced Usage

### Custom Trajectory Execution
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
    
    def execute_custom_trajectory(self):
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Define waypoints
        waypoints = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home
            [0.5, -1.0, -0.5, -2.0, 0.0, 0.0],   # Intermediate
            [1.0, -1.57, 0.0, -1.57, 0.0, 0.0]   # Final
        ]
        
        for i, positions in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 6
            point.time_from_start = rclpy.duration.Duration(seconds=(i+1)*2.0).to_msg()
            trajectory.points.append(point)
        
        self.pub.publish(trajectory)
```

### State Monitoring and Logging
```bash
# Real-time state monitoring
ros2 run ur_state_monitor state_monitor.py

# View logged data
tail -f /workspace/logs/robot_state.log

# Analyze joint velocities
ros2 topic echo /joint_velocities_filtered
```

### Force/Torque Monitoring
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class ForceMonitor(Node):
    def __init__(self):
        super().__init__('force_monitor')
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor',
            self.force_callback,
            10
        )
    
    def force_callback(self, msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        
        # Calculate magnitude
        force_magnitude = (force.x**2 + force.y**2 + force.z**2)**0.5
        torque_magnitude = (torque.x**2 + torque.y**2 + torque.z**2)**0.5
        
        self.get_logger().info(f'Force: {force_magnitude:.2f}N, Torque: {torque_magnitude:.2f}Nm')
        
        # Safety check
        if force_magnitude > 50.0:  # 50N threshold
            self.get_logger().warn('High force detected - stopping robot')
            # Add emergency stop logic here
```

## MoveIt Integration

### Using MoveIt for Motion Planning
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItRequest
from geometry_msgs.msg import PoseStamped

class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')
        # MoveIt integration code here
        
    def plan_to_pose(self, target_pose):
        # Use MoveIt to plan path to target pose
        # This would integrate with the MoveIt motion planning framework
        pass
```

### Motion Planning with Obstacles
```bash
# Start MoveIt with planning scene
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e

# In RViz, add obstacles to planning scene
# Then plan and execute motions that avoid obstacles
```

## Configuration and Customization

### Modifying Robot Parameters
Edit `config/robot_config.yaml`:
```yaml
# Adjust joint limits
joint_limits:
  shoulder_pan_joint: [-3.14, 3.14]  # Restrict range

# Modify control gains
pid_gains:
  shoulder_pan_joint: {p: 150.0, i: 0.2, d: 15.0}
```

### Custom Safety Limits
```yaml
# In robot_config.yaml
safety:
  max_joint_velocity: 2.0  # Reduce from default 3.0
  max_cartesian_velocity: 0.5  # Reduce from default 1.0
  emergency_stop:
    joint_position_error: 0.2  # More strict
```

### Simulation Environment Customization
Edit `config/simulation_config.yaml`:
```yaml
# Add custom objects
environment:
  obstacles:
    - name: "custom_table"
      type: "box"
      position: [0.8, 0.0, 0.4]
      size: [0.6, 1.2, 0.8]
      color: [0.8, 0.4, 0.2, 1.0]
```

## Debugging and Diagnostics

### Checking System Status
```bash
# View all active nodes
ros2 node list

# Check node health
ros2 node info /robot_controller

# Monitor system resources
ros2 run rqt_top rqt_top

# View computation graph
ros2 run rqt_graph rqt_graph
```

### Log Analysis
```bash
# View ROS 2 logs
ros2 log view

# Filter by node
ros2 log view --node robot_controller

# Monitor specific topics
ros2 topic hz /joint_states
ros2 topic bw /joint_states
```

### Performance Monitoring
```python
#!/usr/bin/env python3
# Performance monitoring script
import psutil
import rclpy
from rclpy.node import Node

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.timer = self.create_timer(1.0, self.monitor_callback)
    
    def monitor_callback(self):
        cpu_percent = psutil.cpu_percent()
        memory = psutil.virtual_memory()
        
        self.get_logger().info(f'CPU: {cpu_percent}%, Memory: {memory.percent}%')
        
        if cpu_percent > 80:
            self.get_logger().warn('High CPU usage detected')
```

## Common Use Cases

### Pick and Place Operation
```python
def pick_and_place(self, pick_pose, place_pose):
    # Move to pre-pick position
    pre_pick = pick_pose.copy()
    pre_pick[2] += 0.1  # 10cm above
    self.move_to_pose(pre_pick, [0, 0, 0, 1])
    
    # Move to pick position
    self.move_to_pose(pick_pose, [0, 0, 0, 1])
    
    # Activate gripper (implement based on your gripper)
    # self.activate_gripper()
    
    # Move to pre-place position
    pre_place = place_pose.copy()
    pre_place[2] += 0.1
    self.move_to_pose(pre_place, [0, 0, 0, 1])
    
    # Move to place position
    self.move_to_pose(place_pose, [0, 0, 0, 1])
    
    # Deactivate gripper
    # self.deactivate_gripper()
```

### Continuous Path Following
```python
def follow_path(self, waypoints, duration_per_segment=2.0):
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    
    for i, waypoint in enumerate(waypoints):
        point = JointTrajectoryPoint()
        point.positions = waypoint
        point.time_from_start = rclpy.duration.Duration(
            seconds=(i+1)*duration_per_segment
        ).to_msg()
        trajectory.points.append(point)
    
    self.trajectory_publisher.publish(trajectory)
```

## Safety Guidelines

1. **Always test in simulation first** before connecting to real robot
2. **Keep emergency stop accessible** when working with real robots
3. **Monitor force/torque sensors** for collision detection
4. **Validate all trajectories** before execution
5. **Use appropriate velocity limits** for your application
6. **Implement proper error handling** in all control scripts

## Next Steps

After mastering basic usage:
1. Integrate computer vision for object detection
2. Add force control for contact-rich tasks
3. Implement multi-robot coordination
4. Develop custom end-effectors
5. Create digital twin visualization
