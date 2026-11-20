#!/usr/bin/env python3
"""Autonomous exploration; start the simulator or robot bringup separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    clock = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use /clock; set false for hardware"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Display the map and navigation paths in RViz"),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py",
            ]),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("autonomy_repo"), "rviz", "default.rviz",
                ]),
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        Node(
            package="asl_tb3_lib", executable="state_publisher.py",
            parameters=[clock], output="screen",
        ),
        Node(
            package="autonomy_repo", executable="navigator.py",
            parameters=[clock, {"known_free_only": True}], output="screen",
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package="autonomy_repo", executable="frontier_exploration.py",
                parameters=[clock], output="screen",
            ),
        ]),
    ])
