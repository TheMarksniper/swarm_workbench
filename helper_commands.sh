colcon build --merge-install --packages-select swarm_gazebo --cmake-args -Wno-dev
#for colcon build error around cmake with Gazebo files
ros2 launch swarm_gazebo multi_swarm_launch.py 
#launch file for multi robot spawning

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

#rostopic show all running robots
ros2 topic list | grep --only-matching 'linorobot2_[^/]*' | sort --unique
#used in multi robot navigation starter

#new dynamic number of robots for spawning using swarm_size arg
#max size is set to 100, can be expanded in multi_swawn_robot_laucnh.py in swarm descrition
ros2 launch swarm_gazebo multi_swarm_launch.py swarm_size:=100

