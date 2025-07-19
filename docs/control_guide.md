# Universal Robots ROS 2 Control Guide

## 🎯 **Overview**

This guide demonstrates comprehensive ROS 2 control of Universal Robots with both GUI simulation and headless operation modes. You can control joint positions, execute trajectories, and monitor robot states in real-time.

## 🚀 **Quick Start Guide**

### **Option 1: GUI Simulation (with RViz visualization)**

For visual robot simulation with RViz:

```bash
# Setup X11 forwarding (required for GUI)
xhost +local:docker

# Build and run GUI simulation
docker-compose build ur-gui-simulation
docker-compose up ur-gui-simulation
```

### **Option 2: Headless Simulation (for development)**

For background simulation without GUI:

```bash
# Build and run headless simulation
docker-compose build ur-simulation
docker-compose up ur-simulation
```

### **Option 3: Real Robot Connection**

For connecting to physical UR robot:

```bash
# Set robot IP address
export ROBOT_IP=192.168.1.100

# Build and run real robot connection
docker-compose build ur-real
docker-compose --profile real-robot up ur-real
```

## 🎮 **Interactive Robot Control**

### **Manual Joint Control Commands**

Once simulation is running, open a new terminal and connect to control the robot:

```bash
# Connect to running container
docker exec -it ur_gui_simulation bash

# Source ROS environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Run joint controller
ros2 run ur_control joint_controller.py
```

### **Available Commands:**

1. **Move to specific joint positions:**
   ```bash
   # Format: pos <j1> <j2> <j3> <j4> <j5> <j6> (in radians)
   ros2 topic pub /joint_command sensor_msgs/msg/JointState "
   header:
     stamp: {sec: 0, nanosec: 0}
   name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
   position: [0.5, -1.0, -0.5, -2.0, 0.0, 0.0]
   velocity: []
   effort: []"
   ```

2. **Home position:**
   ```bash
   ros2 topic pub /joint_command sensor_msgs/msg/JointState "
   header:
     stamp: {sec: 0, nanosec: 0}
   name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
   position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
   velocity: []
   effort: []"
   ```

3. **Monitor joint states:**
   ```bash
   ros2 topic echo /joint_states
   ```

## 🤖 **Automated Control Demonstrations**

### **Run Control Demo**

Execute comprehensive control demonstrations:

```bash
# Run all demos in sequence
docker-compose up ur-control-demo

# Or run specific demo manually
docker exec -it ur_simulation bash
ros2 run ur_control control_demo.py
```

### **Demo Sequence Includes:**

1. **Basic Joint Movements**: Moves through predefined positions
2. **Sinusoidal Trajectories**: Smooth sinusoidal joint motions
3. **Pick & Place Simulation**: Simulated pick and place operations
4. **Continuous Monitoring**: Real-time state monitoring and logging

## 📊 **State Monitoring & Logging**

### **Real-time State Monitoring**

```bash
# Run state monitor
docker-compose up monitoring

# View logs
tail -f logs/robot_state.log

# Monitor specific topics
ros2 topic echo /joint_states
ros2 topic echo /demo_status
```

### **Available Topics:**

- `/joint_states` - Current joint positions, velocities, efforts
- `/joint_command` - Joint position commands
- `/demo_status` - Demo execution status
- `/robot_description` - Robot URDF model

## 🔧 **Advanced Control Examples**

### **Custom Trajectory Execution**

Create custom trajectory scripts:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.t = 0.0
    
    def control_loop(self):
        # Create sinusoidal motion
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        # Sinusoidal positions
        cmd.position = [
            0.5 * math.sin(self.t),
            -1.57 + 0.3 * math.sin(self.t * 2),
            0.5 * math.cos(self.t),
            -1.57,
            0.0,
            self.t * 0.1
        ]
        
        self.publisher.publish(cmd)
        self.t += 0.1

def main():
    rclpy.init()
    controller = CustomController()
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### **Velocity Control Example**

```python
# Velocity control
cmd = JointState()
cmd.header.stamp = self.get_clock().now().to_msg()
cmd.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
           'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
cmd.velocity = [0.1, -0.2, 0.15, 0.0, 0.0, 0.3]  # rad/s
```

## 🛡️ **Safety Features**

### **Joint Limits & Safety**

All controllers include safety features:

- **Position limits**: ±6.28 rad for most joints
- **Velocity limits**: Configurable per joint  
- **Emergency stop**: Keyboard interrupt handling
- **Timeout detection**: Communication timeout monitoring

### **Safety Commands**

```bash
# Emergency stop (in any terminal)
Ctrl+C

# Check joint limits
ros2 param get /ur_joint_controller safety_limits

# Monitor safety status
ros2 topic echo /diagnostics
```

## 📈 **Development Workflow**

### **Building Custom Nodes**

1. **Create new control node:**
   ```bash
   # In container
   cd /workspace/src/ur_control/ur_control/
   touch my_controller.py
   chmod +x my_controller.py
   ```

2. **Update CMakeLists.txt:**
   ```cmake
   install(PROGRAMS
     ur_control/my_controller.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```

3. **Build and test:**
   ```bash
   colcon build --packages-select ur_control
   source install/setup.bash
   ros2 run ur_control my_controller.py
   ```

### **Testing & Validation**

```bash
# Build workspace
colcon build --symlink-install

# Run tests
colcon test --packages-select ur_control ur_state_monitor

# Check test results
colcon test-result --verbose
```

## 🔄 **Switching Between Modes**

### **Physical Robot → Simulation**

```bash
# Stop real robot service
docker-compose --profile real-robot down

# Start simulation
docker-compose up ur-gui-simulation
```

### **Headless → GUI**

```bash
# Stop headless
docker-compose down ur-simulation

# Setup X11 and start GUI
xhost +local:docker
docker-compose up ur-gui-simulation
```

## 🎯 **Next Steps**

### **Extend Functionality**

1. **Add MoveIt Integration:**
   - Path planning
   - Collision detection
   - Inverse kinematics

2. **Sensor Integration:**
   - Camera feeds
   - Force/torque sensors
   - Gripper control

3. **Advanced Control:**
   - Impedance control
   - Visual servoing
   - Multi-robot coordination

### **Integration Examples**

```bash
# Example: Integrate with computer vision
docker exec -it ur_gui_simulation bash
pip install opencv-python
ros2 run ur_control vision_controller.py

# Example: Add gripper control
ros2 run ur_control gripper_controller.py
```

This comprehensive setup provides a solid foundation for Universal Robots development with ROS 2, supporting both simulation and real robot control scenarios!
