# 🤖 Universal Robots ROS 2 Control Examples & Demonstrations

## 🎯 **Quick Start - Get Robot Moving in 2 Minutes**

### **Step 1: Start the Simulation**
```bash
# Start headless simulation (works immediately)
cd /home/erolc/fyp_v2
docker-compose up ur-simulation
```

### **Step 2: Connect and Control**
```bash
# In a new terminal, connect to the container
docker exec -it ur_ros2_simulation bash

# Source ROS environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Move robot joints immediately!
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
header: {stamp: {sec: 0, nanosec: 0}}
name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
position: [0.5, -1.0, -0.5, -2.0, 0.0, 0.0]"
```

### **Step 3: Monitor Robot State**
```bash
# Watch joint states in real-time
ros2 topic echo /joint_states
```

## 🎮 **Interactive Robot Control Commands**

### **Basic Position Commands**
```bash
# Home position (safe starting point)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]"

# Extended reach position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.5, -1.0, -0.5, -2.0, 0.0, 0.0]"

# Compact folded position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [-0.5, -2.0, 1.0, -1.5, 1.57, 0.0]"

# Overhead reach
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -0.5, -1.0, -2.0, 0.0, 0.0]"
```

### **Advanced Control Examples**

#### **Example 1: Simple Pick and Place**
```bash
# Approach position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.3, -1.2, -0.8, -1.8, 1.57, 0.0]"

# Pick position (lower)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.3, -1.5, -0.5, -2.0, 1.57, 0.0]"

# Lift object
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.3, -1.2, -0.8, -1.8, 1.57, 0.0]"

# Move to place location
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [-0.3, -1.2, -0.8, -1.8, 1.57, 0.0]"

# Place position (lower)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [-0.3, -1.5, -0.5, -2.0, 1.57, 0.0]"
```

#### **Example 2: Scanning Motion**
```bash
# Left scan position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [1.0, -1.0, -0.5, -1.5, 0.0, 0.0]"

# Center scan position  
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -1.0, -0.5, -1.5, 0.0, 0.0]"

# Right scan position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [-1.0, -1.0, -0.5, -1.5, 0.0, 0.0]"
```

## 🚀 **Automated Control Demonstrations**

### **Run Complete Demo Suite**
```bash
# Start the comprehensive control demo
ros2 run ur_control control_demo.py
```

This demo includes:
- ✅ **Basic joint movements** through key positions
- ✅ **Sinusoidal trajectories** for smooth motion
- ✅ **Pick and place simulation** with realistic timing
- ✅ **Continuous state monitoring** with safety checking

### **Custom Demo Script**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
    def move_to(self, positions):
        """Move robot to specified joint positions"""
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        cmd.position = positions
        self.publisher.publish(cmd)
        self.get_logger().info(f'Moving to: {positions}')

def main():
    rclpy.init()
    controller = SimpleController()
    
    # Define a sequence of positions
    sequence = [
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home
        [0.5, -1.0, -0.5, -2.0, 0.0, 0.0],   # Position 1
        [-0.5, -2.0, 1.0, -1.5, 1.57, 0.0],  # Position 2
        [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Back to home
    ]
    
    for i, pos in enumerate(sequence):
        controller.move_to(pos)
        time.sleep(3)  # Wait between moves
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 📊 **Real-Time Monitoring & Debugging**

### **Monitor All Robot Topics**
```bash
# List all available topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor demo status (if running demo)
ros2 topic echo /demo_status

# Check robot description
ros2 topic echo /robot_description
```

### **Debug Joint Information**
```bash
# Get current joint positions
ros2 topic echo /joint_states --once

# Check joint limits and properties
ros2 param list

# Monitor publication rate
ros2 topic hz /joint_states
```

## 🛡️ **Safety Commands & Emergency Procedures**

### **Emergency Stop**
```bash
# Stop any running nodes
Ctrl+C (in terminal running the node)

# Or stop container
docker stop ur_ros2_simulation
```

### **Safe Positions**
```bash
# Return to safe home position immediately
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]"

# Compact safe position (if space is limited)
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -2.0, 1.5, -1.0, 0.0, 0.0]"
```

## 🔄 **Switching to Real Robot**

### **Step 1: Prepare for Real Robot**
```bash
# Stop simulation
docker-compose down ur-simulation

# Set your robot's IP address
export ROBOT_IP=192.168.1.100  # Replace with your robot's IP
```

### **Step 2: Connect to Real Robot**
```bash
# Start real robot connection
docker-compose --profile real-robot up ur-real

# Use the same control commands as simulation!
```

### **Step 3: Safety with Real Robot**
```bash
# Always start with home position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]"

# Move slowly and verify each position
# Keep emergency stop accessible on robot teach pendant
```

## 🎯 **Practical Applications**

### **Application 1: Quality Inspection**
```bash
# Position for camera inspection
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.0, -0.8, -1.0, -1.5, 0.0, 0.0]"
```

### **Application 2: Assembly Task**
```bash
# Assembly approach position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [0.2, -1.3, -0.7, -1.8, 1.57, 0.0]"
```

### **Application 3: Material Handling**
```bash
# Conveyor pickup position
ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState "
position: [1.2, -1.0, -0.5, -1.5, 0.0, 0.0]"
```

## 📚 **Joint Explanation**

Understanding UR5e joint structure:
- **Joint 1 (shoulder_pan)**: Base rotation (±180°)
- **Joint 2 (shoulder_lift)**: Shoulder up/down (±180°)  
- **Joint 3 (elbow)**: Elbow bend (±180°)
- **Joint 4 (wrist_1)**: Wrist rotation 1 (±180°)
- **Joint 5 (wrist_2)**: Wrist rotation 2 (±180°)
- **Joint 6 (wrist_3)**: Tool rotation (±180°)

Angles in radians:
- **0.0** = 0°
- **1.57** ≈ 90°
- **3.14** ≈ 180°
- **-1.57** ≈ -90°

## 🎊 **Success! You Can Now:**

✅ **Control robot joints in real-time**
✅ **Monitor robot state continuously**  
✅ **Execute complex motion sequences**
✅ **Switch between simulation and real robot**
✅ **Implement custom control algorithms**
✅ **Build industrial automation applications**

**Your Universal Robots ROS 2 workspace is fully operational and ready for advanced robotics development!**
