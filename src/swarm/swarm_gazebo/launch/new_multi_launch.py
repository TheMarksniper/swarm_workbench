# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawning multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks.
"""

import os
import sys
import yaml
from math import pi
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def gen_robot_list(column: int, row: int):
    robots = []
    i = 0
    for x in range(column):
        for y in range(row):
            i = i+1
            robot_name = f"Srobot{i}"
            #robot_name = ""
            x_pos = float(x)
            y_pos = float(y)
            robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.01,
                                                'roll': 0.0, 'pitch': 0.0, 'yaw': pi },)


    return robots 

#old approach
#def gen_robot_list(column: int):
#    robots = []
#    for x in range(column): 
#        robot_name = f"robot{x+1}"
#        #robot_name = ""
#        x_pos = float(x)
#        y_pos = float(1.0)
#        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.01, 
#                                            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},)
#    return robots

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    swarm_dir = get_package_share_directory('swarm_navigation')
    swarm_gazebo_dir = get_package_share_directory('swarm_gazebo')
    linorobot_dir = get_package_share_directory('linorobot2_navigation')


    for arg in sys.argv:
        if arg.startswith("swarm_size:="):
            swarm_size_arg = int(arg.split(":=")[1])
        else:
            swarm_size_arg = 5
    # Names and poses of the robots
     ##THIS WHOLE THING SHOULD BE A FUNCTION##
    if(swarm_size_arg > 100):
        print('too many, maximum is 100, spawning 10 for now')
        robots = gen_robot_list(2, 5)
    elif(swarm_size_arg % 10 == 0):
        robots = gen_robot_list(10,int(swarm_size_arg/10))
    elif(swarm_size_arg % 9 == 0):
        robots = gen_robot_list(9,int(swarm_size_arg/9))
    elif(swarm_size_arg % 8 == 0):
        robots = gen_robot_list(8,int(swarm_size_arg/8))   
    elif(swarm_size_arg % 7 == 0):
        robots = gen_robot_list(7,int(swarm_size_arg/7))   
    elif(swarm_size_arg % 6 == 0):
        robots = gen_robot_list(6,int(swarm_size_arg/6)) 
    elif(swarm_size_arg % 5 == 0):
        robots = gen_robot_list(5,int(swarm_size_arg/5))
    elif(swarm_size_arg % 4 == 0):
        robots = gen_robot_list(4,int(swarm_size_arg/4))   
    elif(swarm_size_arg % 3 == 0):
        robots = gen_robot_list(3,int(swarm_size_arg/3))   
    elif(swarm_size_arg % 2 == 0):
        robots = gen_robot_list(2,int(swarm_size_arg/2)) 
    else: 
        print('try number that is divisible for matrix spawn (divisible bu 5,4,3,2), spawning 10 for now')
        robots = gen_robot_list(2, 5)    
    ##ALL THIS##

    print(robots)
    with open(os.path.join(swarm_dir, 'params','nav2_multirobot_params.yaml'), 'r') as file:
                params = yaml.safe_load(file)   
    for x in range(swarm_size_arg):
        params['local_costmap']['local_costmap']['ros__parameters']['voxel_layer']['scan']['topic'] = f'/robot{x+1}/scan'
        params['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['scan']['topic'] = f'/robot{x+1}/scan'
        with open(os.path.join(swarm_dir, 'params',f'nav2_multirobot_params_{x+1}.yaml'), 'w') as yaml_file:
            yaml.dump(params,yaml_file)
            yaml_file.close()
    

    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(swarm_gazebo_dir, 'worlds', 'slim_blockage.world'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(linorobot_dir, 'maps', 'slim_blockage.yaml'),
        description='Full path to map file to load')
   
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Start Gazebo with plugin providing the robot spawning service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for index, robot in enumerate(robots):
        params_file = os.path.join(swarm_dir, 'params', f'nav2_multirobot_params_{index+1}.yaml')

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(swarm_gazebo_dir,
                                                           'launch',
                                                           'new_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),
                                  

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
