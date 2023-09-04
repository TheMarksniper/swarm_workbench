#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
find_robot_namespaces = "ros2 topic list | grep --only-matching 'linorobot2_[^/]*' | sort --unique"
#os.system(find_robot_namespaces)
result = subprocess.getoutput(find_robot_namespaces)
print(len(result))
namespaces = result.split("\n")

def generate_launch_description():
    use_sim_time = True
    DeclareLaunchArgument(
            'namespace', default_value='',
             description='Top-level namespace')
    navigation_launch_path = PathJoinSubstitution(
        [FindPackageShare('swarm_gazebo'), 'launch', 'nav_launch.py']
    )
    spawn_nav_cmds = []
    for namespace in namespaces:
        spawn_nav_cmds.append(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'namespace':namespace,
                'use_sime_time':use_sim_time
            }.items()
                
        )
    print()
    # Create the launch description and populate
    ld = LaunchDescription()
    #
    for spawn_nav_cmd in spawn_nav_cmds:
        ld.add_action(spawn_nav_cmd)      

    return ld