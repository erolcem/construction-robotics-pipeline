#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindPackageShare, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare as FindPackageShareRos
import os


def generate_launch_description():
    """Launch UR robot simulation environment."""
    
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='ur5e',
        description='UR robot model (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20)'
    )
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start RViz GUI'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Start MoveIt motion planning'
    )
    
    # Get launch configuration
    robot_model = LaunchConfiguration('robot_model')
    use_gui = LaunchConfiguration('use_gui')
    use_moveit = LaunchConfiguration('use_moveit')
    
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', robot_model]),
        '.urdf.xacro',
        ' robot_ip:=xxx.yyy.zzz.www',
        ' joint_limit_params_file:=',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'config', robot_model, 'joint_limits.yaml']),
        ' kinematics_params_file:=',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'config', robot_model, 'default_kinematics.yaml']),
        ' physical_params_file:=',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'config', robot_model, 'physical_parameters.yaml']),
        ' visual_params_file:=',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'config', robot_model, 'visual_parameters.yaml']),
        ' transmission_hw_interface:=hardware_interface/PositionJointInterface',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20'
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )
    
    # Joint state publisher GUI (for manual control in simulation)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'rviz', 'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_gui)
    )
    
    # UR robot control nodes
    robot_controller_node = Node(
        package='ur_control',
        executable='robot_controller.py',
        name='robot_controller',
        parameters=[
            {'robot_model': robot_model},
            {'simulation_mode': True},
            {'control_frequency': 50.0}
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
            {'log_file_path': '/workspace/logs/robot_state_sim.log'}
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
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(use_moveit)
    )
    
    return LaunchDescription([
        robot_model_arg,
        use_gui_arg,
        use_moveit_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        robot_controller_node,
        state_monitor_node,
        moveit_launch
    ])
