#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Simple UR simulation launch with Gazebo and RViz."""
    
    # Declare arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ur5e',
        description='UR robot model'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start GUI applications'
    )
    
    # Get configuration
    robot_model = LaunchConfiguration('robot_model')
    use_gui = LaunchConfiguration('use_gui')
    
    # Include UR bringup launch - this will start Gazebo with the robot
    ur_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_bringup'),
                'launch',
                'ur_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': robot_model,
            'use_fake_hardware': 'true',
            'launch_rviz': 'true',
        }.items()
    )
    
    # Custom control node
    ur_control_node = Node(
        package='ur_control',
        executable='robot_controller.py',
        name='robot_controller',
        output='screen',
        parameters=[
            {'robot_model': robot_model},
            {'simulation_mode': True}
        ]
    )
    
    # State monitor node
    ur_state_monitor_node = Node(
        package='ur_state_monitor',
        executable='state_monitor.py',
        name='state_monitor',
        output='screen',
        parameters=[
            {'robot_model': robot_model}
        ]
    )
    
    return LaunchDescription([
        robot_model_arg,
        use_gui_arg,
        ur_bringup_launch,
        ur_control_node,
        ur_state_monitor_node,
    ])
