#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription,GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
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
        map_yaml_file_dir = Path(get_package_share_directory("linorobot2_navigation"))
        map_yaml_file = map_yaml_file_dir.joinpath("map").joinpath("slim_blockage.yaml")
        DeclareLaunchArgument(
                'map_yaml_file', default_value='',
                 description='map file')
        navigation_launch_path = PathJoinSubstitution(
            [FindPackageShare('swarm_navigation'), 'launch', 'nav_launch.py']
        )
        nav_instances_cmds = []
        for robot in namespaces:
            params_file = eval(f"{robot}_params_file.yaml")

            group = GroupAction([
                # Instances use the robot's name for namespace
                PushRosNamespace(robot),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(navigation_launch_path, 'launch', 'bringup_launch.py')),
                    launch_arguments={
                                      'namespace': robot,
                                      'use_namespace':'true',
                                      'map': map_yaml_file,
                                      'use_sim_time': use_sim_time,
                                      'params_file': params_file,
                                      'use_composition':'true',
                                      'autostart': 'True',
                                      'use_remappings': 'True'}.items())
        ])
        nav_instances_cmds.append(group)
        #spawn_nav_cmds = []
        #for namespace in namespaces:
        #    spawn_nav_cmds.append(
        #        IncludeLaunchDescription(
        #            PythonLaunchDescriptionSource(navigation_launch_path),
        #            launch_arguments={
        #                'namespace': namespace,
        #                #'use_sime_time':use_sim_time
        #            }.items()
        #        )
        #    )
        #print()
        # Create the launch description and populate
        ld = LaunchDescription()
        for nav_instance in nav_instances_cmds:
            ld.add_action(nav_instance)
        #
        #for spawn_nav_cmd in spawn_nav_cmds:
        #    ld.add_action(spawn_nav_cmd)      

        return ld
    
    