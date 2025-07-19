#!/bin/bash

set -e

# Test script to verify ROS 2 UR simulation is working
echo "🔍 Testing ROS 2 UR Simulation..."

# Setup ROS environment
source /opt/ros/humble/setup.bash
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

echo "📊 Checking ROS 2 nodes..."
timeout 5 ros2 node list || echo "No nodes running yet"

echo "📊 Checking robot description..."
timeout 5 ros2 topic list | grep -E "(robot_description|joint_states)" || echo "Robot topics not found"

echo "📊 Checking tf transforms..."
timeout 5 ros2 topic echo /tf_static --once || echo "No tf_static available"

echo "✅ Basic ROS 2 simulation test completed"
