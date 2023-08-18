from launch import LaunchDescription

import launch.actions
import launch_ros.actions
import os
from launch import LaunchDescription
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = True
    #slam_launch_path = PathJoinSubstitution(
    #    [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']
    #)
#
    #slam_config_path = PathJoinSubstitution(
    #    [FindPackageShare('linorobot2_navigation'), 'config', 'slam.yaml']
    #)
#
    #rviz_config_path = PathJoinSubstitution(
    #    [FindPackageShare('linorobot2_navigation'), 'rviz', 'linorobot2_slam.rviz']
    #)
    #
    #description_launch_path = PathJoinSubstitution(
    #    [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    #)
    swarm_description_dir = FindPackageShare(package='swarm_description').find('swarm_description')
    lc = LaunchContext()
    ros_distro = EnvironmentVariable('ROS_DISTRO')
    slam_param_name = 'slam_params_file'
    if ros_distro.perform(lc) == 'humble': 
        slam_param_name = 'params_file'
    robot_namespace = launch.substitutions.LaunchConfiguration('robot_namespace'),
    remappings = [('/tf','tf'),
                  ('/tf_static','tf_static')]
    return LaunchDescription([
        launch_ros.actions.Node(
            package='swarm_description',
            executable='spawn_robot.py',
            output='screen',
            arguments=[
                '--robot_urdf', launch.substitutions.LaunchConfiguration('robot_urdf'),
                '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                '-x', launch.substitutions.LaunchConfiguration('x'),
                '-y', launch.substitutions.LaunchConfiguration('y'),
                '-z', launch.substitutions.LaunchConfiguration('z')]),
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        remappings=remappings,
        parameters=[{'robot_description': launch.substitutions.Command(['xacro ',os.path.join(swarm_description_dir,'urdf/robots/2wd.urdf.xacro')])}]

        ) 
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(description_launch_path),
        #    launch_arguments={
        #        'use_sim_time': str(use_sim_time),
        #        'publish_joints': 'false',
        #        'tf_prefix': launch.substitutions.LaunchConfiguration('robot_namespace'),
#
        #        
        #    }.items()
        #)
        #DeclareLaunchArgument(
        #    name='rviz', 
        #    default_value='True',
        #    description='Run rviz'
        #),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(description_launch_path),
        #    launch_arguments={
        #        'use_sim_time': str(use_sim_time),
        #        'publish_joints': 'True',
        #    }.items()
        #),
#
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(slam_launch_path),
        #    launch_arguments={
        #        'use_sim_time': str(use_sim_time),
        #        slam_param_name: slam_config_path
        #    }.items()
        #),
#
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen',
        #    arguments=['-d', rviz_config_path],
        #    condition=IfCondition(LaunchConfiguration("rviz")),
        #    parameters=[{'use_sim_time': str(use_sim_time)}]
        #)
    ])
