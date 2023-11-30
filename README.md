# swarm_workbench
This project is focused on Decentralised Swarm Operation, from simulation, navigation to production.  If you want to contribute to this project, comment and we can collaborate.


What will you need?
Ubuntu 22.04
ROS2 Humble atleast November 1st 2023 update ->https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
Gazebo, can be installed along with ROS2
NAV2, has to be installed along with ROS2
this multi robot work is using robot from Linorobot 2 project -> https://github.com/linorobot/linorobot2  (It is included in this project)
Or Turtlebot3 downloaded through ROS2 packages 

If you have all these already you can download this repository:
git clone https://github.com/TheMarksniper/swarm_workbench.git


build your environment in the local file:
colcon build --merge-install
#for colcon build error around cmake with Gazebo files
colcon build --merge-install --packages-select swarm_gazebo --cmake-args -Wno-dev


Then just source your newly built files:
source install/setup.bash


For simulation of multiple robots there is an easy launcher ready:
#new dynamic number of robots for spawning using swarm_size arg
#max size is set to 100, can be expanded in multi_swawn_robot_laucnh.py in swarm descrition


ros2 launch swarm_gazebo multi_swarm_launch.py swarm_size:=100

#new multi launch works pretty much same, but should be more general approach
#with changeble robot types (linorobot/turtlebot,etc.)
#also this one autogenerates parameter files and autostarts nav2 with RVIZ on every robot (much more demanding)
ros2 launch swarm_gazebo new_multi_launch.py swarm_size:=20 

Try some of the robot commander scripts in swarm_navigation(you have to move to src/swarm/swarm_navigation):
python3 select_route.py --robot Srobot1


good to know tricks:
#rostopic show all running robots
ros2 topic list | grep --only-matching 'Srobot[^/]*' | sort --unique
#used in multi robot navigation starter


ros2 launch swarm_gazebo multi_swarm_launch.py 
#launch file for multi robot spawning

#teleoperation file
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/Srobot1/cmd_vel
#change the remap to fit the robot that you want

#first launch should be paused becouse of gazebo
ros2 launch swarm_gazebo multi_swarm_launch.py pause:=true

usage of TF2_TOOLS
ros2 run tf2_tools view_frames --ros-args -r __ns:=/Srobot1 -r /tf:=tf -r /tf_static:=tf_static

#navigation solo starter
ros2 launch swarm_gazebo nav_launch.py namespace:=/Srobot1

#start all navigations at once 
ros2 launch swarm_gazebo swarm_nav_launch.py

