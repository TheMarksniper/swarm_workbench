#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from pathlib import Path

import xacro 

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution



def gen_robot_list(column: int, row: int):
    robots = []

    for x in range(column):
        for y in range(row):
            robot_name = f"linorobot2_x{x:03}_y{y:03}"
            x_pos = float(x)
            y_pos = float(y)
            robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.01})


    return robots 

def generate_launch_description():
    robot_path = Path(get_package_share_directory("linorobot2_description"))
    robot_urdf_robot_path = robot_path.joinpath("urdf").joinpath("robots")
    # robot_urdf_robot_path = robot_path.joinpath("linorobot2_description").joinpath("urdf").joinpath("robots")
    xacro_file_path = robot_urdf_robot_path.joinpath("2wd.urdf.xacro")

    urdf = xacro.process_file(xacro_file_path.__str__(), mappings={'simulate_obstacles' : 'false'})

    #urdf = os.path.join(get_package_share_directory('swarm_description'), 'robot/', 'box_bot_v2.urdf')
    pkg_swarm_description_path = Path(get_package_share_directory('swarm_description'))
    pkg_swarm_description_robot_path = pkg_swarm_description_path.joinpath("robot")
    urdf_file_path = pkg_swarm_description_robot_path.joinpath("linorobot2.urdf")

    with open(urdf_file_path, "w") as f:
        urdf.writexml(f)


    # Names and poses of the robots
    robots = gen_robot_list(2, 2)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        robot_name = robot["name"]

        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_swarm_description_path.__str__(), 'launch',
                                                           'spawn_swarm_launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf_file_path.__str__(),
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot_name,
                                  'robot_namespace': robot_name
                                  }.items()),
                
                #PythonLaunchDescriptionSource(os.path.join("/opt/ros/humble/share/nav2_bringup/launch", 'launch',
                #                                           'navigation_launch.py')),
                #launch_arguments={
                #                  'namespace': robot_name
                #                  }.items())
        )

        #spawn_robots_cmds.append(
        #    IncludeLaunchDescription(
        #        PythonLaunchDescriptionSource(os.path.join("/opt/ros/humble/share/nav2_bringup/launch", 'bringup_launch.py')),
        #        launch_arguments={
        #            'namespace': robot_name,
        #            'use_namespace': robot_name,
        #            #'slam': slam,
        #            #'map': map_yaml_file,
        #            'use_sim_time': "True",
        #            'params_file': params_file,
        #            'autostart': autostart,
        #            'use_composition': use_composition,
        #            'use_respawn': use_respawn,
        #        }.items()
        #    )
        #)
            

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld

if __name__ == "__main__":
    generate_launch_description()
