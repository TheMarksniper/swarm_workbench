import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = True

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "u_shape.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'namespace.description.launch.py']
    )

    pkg_gazebo_ros = '/opt/ros/humble/share/gazebo_ros/'
    
    # Gazebo launch
    paused_arg = DeclareLaunchArgument(
        'pause',
        default_value='false',
        description='Start Gazebo paused'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    ) 

    return LaunchDescription([
        #ExecuteProcess(
        #    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', world_path],
        #    output='screen'
        #),
        DeclareLaunchArgument(
          'world',
          default_value=world_path,
          description='SDF world file'),
        paused_arg,
        gazebo,

        Node(
            namespace="linobot",
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "linorobot2"]
        ),
#
        #Node(
        #    package='linorobot2_gazebo',
        #    executable='command_timeout.py',
        #    name='command_timeout'
        #),
#
        #Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    name='ekf_filter_node',
        #    output='screen',
        #    parameters=[
        #        {'use_sim_time': use_sim_time}, 
        #        ekf_config_path
        #    ],
        #    remappings=[("odometry/filtered", "odom")]
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
                
            }.items()
        #),
#
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(joy_launch_path),
        )
        
    ])