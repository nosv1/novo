#! /usr/bin/env python3

# Python Imports

# ROS Imports
from launch import LaunchDescription
from launch_ros.actions import Node

# Personal Imports

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='novo',
            executable='controller.py',
            name='controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='novo',
            executable='path_planner.py',
            name='path_planner',
            output='screen',
            parameters=[]
        ),
    ])