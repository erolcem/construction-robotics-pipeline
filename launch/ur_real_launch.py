#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch UR real robot connection."""
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.100',
        description='IP address of the UR robot'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ur5e',
        description='UR robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20)'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Start MoveIt motion planning'
    )
    
    kinematics_config_arg = DeclareLaunchArgument(
        'kinematics_config',
        default_value='',
        description='Kinematics config file path'
    )
    
    # Get launch configuration
    robot_ip = LaunchConfiguration('robot_ip')
    robot_model = LaunchConfiguration('robot_model')
    use_moveit = LaunchConfiguration('use_moveit')
    kinematics_config = LaunchConfiguration('kinematics_config')
    
    # UR robot driver launch
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': robot_model,
            'robot_ip': robot_ip,
            'use_sim_time': 'false',
            'launch_rviz': 'true',
            'kinematics_config': kinematics_config
        }.items()
    )
    
    # Robot controller node
    robot_controller_node = Node(
        package='ur_control',
        executable='robot_controller.py',
        name='robot_controller',
        parameters=[
            {'robot_model': robot_model},
            {'simulation_mode': False},
            {'control_frequency': 50.0},
            {'robot_ip': robot_ip}
        ],
        output='screen'
    )
    
    # State monitoring node
    state_monitor_node = Node(
        package='ur_state_monitor',
        executable='state_monitor.py',
        name='state_monitor',
        parameters=[
            {'log_frequency': 10.0},
            {'save_logs': True},
            {'log_file_path': '/workspace/logs/robot_state_real.log'}
        ],
        output='screen'
    )
    
    # MoveIt launch (optional)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': robot_model,
            'use_sim_time': 'false'
        }.items(),
        condition=IfCondition(use_moveit)
    )
    
    return LaunchDescription([
        robot_ip_arg,
        robot_model_arg,
        use_moveit_arg,
        kinematics_config_arg,
        ur_driver_launch,
        robot_controller_node,
        state_monitor_node,
        moveit_launch
    ])
