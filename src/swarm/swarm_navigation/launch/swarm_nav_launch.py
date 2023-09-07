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
from launch.substitutions import PathJoinSubstitution, TextSubstitution

find_robot_namespaces = "ros2 topic list | grep --only-matching 'linorobot2_[^/]*' | sort --unique"
#os.system(find_robot_namespaces)
result = subprocess.getoutput(find_robot_namespaces)
namespaces = result.split("\n")
print(namespaces)

def generate_launch_description():
    if (len(namespaces) <= 0 ): return 0
    else:
        use_sim_time = True
        DeclareLaunchArgument(
                'namespace', default_value='',
                 description='Top-level namespace')
        navigation_launch_path = PathJoinSubstitution(
            [FindPackageShare('swarm_navigation'), 'launch', 'nav_launch.py']
        )
        spawn_nav_cmds = []
        for namespace in namespaces:
            spawn_nav_cmds.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(navigation_launch_path),
                    launch_arguments={
                        'namespace': namespace,
                        #'use_sime_time':use_sim_time
                    }.items()
                )
            )
        print()
        # Create the launch description and populate
        ld = LaunchDescription()
        #
        for spawn_nav_cmd in spawn_nav_cmds:
            ld.add_action(spawn_nav_cmd)      

        return ld