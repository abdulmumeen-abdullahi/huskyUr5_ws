#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gazebo_pkg = get_package_share_directory("husky_ur5_gazebo")
    moveit_pkg = get_package_share_directory("ur5_moveit_config")

    # Existing Gazebo bringup, included as-is (Gazebo, rsp, spawn, clock bridge, spawners)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "ur5_on_riser.launch.py")
        )
    )

    
    moveit_config = (
        MoveItConfigsBuilder("ur5-robot", package_name="ur5_moveit_config")
        .robot_description(
            file_path="config/ur5-robot.urdf.xacro",
            mappings={"use_sim": "false"},
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    sim_time = {"use_sim_time": True}

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), sim_time],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_pkg, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_time,
        ],
    )

    # Give Gazebo time to spawn the robot and activate controllers; tune if needed
    return LaunchDescription([gazebo, TimerAction(period=10.0, actions=[move_group, rviz])])