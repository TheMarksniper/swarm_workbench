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

---------usage of TF2_TOOLS-------------
ros2 run tf2_tools view_frames --ros-args -r __ns:=/linorobot2_x001_y000 -r /tf:=tf -r /tf_static:=tf_static

#navigation solo starter
ros2 launch swarm_gazebo nav_launch.py namespace:=/linorobot2_x000_y001

#start all navigations at once 
ros2 launch swarm_gazebo swarm_nav_launch.py

