#!/bin/bash

# Set up ROS 2 environment
source /opt/ros/humble/setup.bash

# Set up workspace if built
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

# Set display for GUI applications
export DISPLAY=${DISPLAY:-:0}

# Execute the passed command or start bash
exec "$@"
