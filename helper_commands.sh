colcon build --merge-install --packages-select swarm_gazebo --cmake-args -Wno-dev
#for colcon build error around cmake with Gazebo files
ros2 launch swarm_gazebo multi_swarm_launch.py 
#launch file for multi robot spawning
#ERROR] [launch]: Caught exception in launch (see debug for traceback):
#cannot import name 'GazeboRosPaths' from 'scripts' 
#(/home/swarmmaster/swarm_workbench/install/local/lib/python3.11/dist-packages/scripts/__init__.py)
cd /install/local/lib/python3.11/dist-packages/scripts/
rm __init__.py
#removes the launch file error

#teleoperation file
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/linorobot2_x000_y000/cmd_vel
#change the remap to fit the robot that you want

#first launch should be paused becouse of gazebo
ros2 launch swarm_gazebo multi_swarm_launch.py pause:=true
