#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    # Define package paths
    desc_pkg = get_package_share_directory('husky_ur5_description')
    
   
    gazebo_pkg = get_package_share_directory('ros_gz_sim')
    my_gazebo_pkg = get_package_share_directory('husky_ur5_gazebo')

    # File paths
    xacro_file = os.path.join(desc_pkg, 'urdf', 'ur5', 'ur5.urdf.xacro')
    
    world_file = os.path.join(my_gazebo_pkg, 'worlds', 'table_riser.world')

    # Add the workspace install share directory to GZ_SIM_RESOURCE_PATH so gazebo can find meshes
    set_env_var = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(desc_pkg, '..')
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': True
        }]
    )

    # Spawn the robot in Gazebo
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ur5',
            '-x', '-0.2',
            '-y', '-0.5',
            '-z', '1.021',
            # '-z', '1.0',
        ]
    )

    # Bridge for /clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    # Spawn the Joint State Broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Spawn the Arm Controller
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        set_env_var,
        gazebo_launch,
        robot_state_publisher,
        gazebo_spawn_entity,
        clock_bridge,
        joint_state_broadcaster,
        arm_controller
    ])
