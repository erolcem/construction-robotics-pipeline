#!/usr/bin/env python3

"""
UR Simulation Launch with GUI
Launches complete UR simulation with RViz visualization and Gazebo simulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="ur5e",
            description="Type of the UR robot (ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Enable headless mode (disable GUI).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_joint_control",
            default_value="true",
            description="Launch joint controller node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_state_monitor",
            default_value="true",
            description="Launch state monitor node.",
        )
    )

    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    headless = LaunchConfiguration("headless")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_joint_control = LaunchConfiguration("launch_joint_control")
    launch_state_monitor = LaunchConfiguration("launch_state_monitor")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            " ",
            "robot_type:=",
            robot_type,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher for simulation control
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {"source_list": ["joint_states"], "rate": 30}
        ],
    )

    # Joint state publisher GUI (only if not headless)
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition("false"),  # We'll control via our custom controller
    )

    # RViz node (only if not headless and launch_rviz is true)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    # Custom joint controller node
    joint_controller_node = Node(
        package="ur_control",
        executable="joint_controller.py",
        name="ur_joint_controller",
        output="both",
        condition=IfCondition(launch_joint_control),
        parameters=[
            {"robot_type": robot_type},
            {"simulation_mode": use_fake_hardware},
            {"control_frequency": 10.0},
            {"safety_limits": True},
        ],
    )

    # State monitor node
    state_monitor_node = Node(
        package="ur_state_monitor",
        executable="state_monitor.py",
        name="ur_state_monitor",
        output="both",
        condition=IfCondition(launch_state_monitor),
        parameters=[
            {"robot_type": robot_type},
            {"log_frequency": 1.0},
            {"log_file": "/workspace/logs/robot_state.log"},
        ],
    )

    # Try to include ur_bringup launch if available
    try:
        ur_control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("ur_bringup"), "launch", "ur_control.launch.py"])]
            ),
            launch_arguments={
                "robot_type": robot_type,
                "use_fake_hardware": use_fake_hardware,
                "fake_sensor_commands": fake_sensor_commands,
                "activate_joint_controller": "false",  # We'll use our custom controller
            }.items(),
            condition=IfCondition(use_fake_hardware),
        )
        ur_nodes = [ur_control_launch]
    except:
        # Fallback if ur_bringup is not available
        ur_nodes = []

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        joint_controller_node,
        state_monitor_node,
    ] + ur_nodes

    return LaunchDescription(declared_arguments + nodes)
