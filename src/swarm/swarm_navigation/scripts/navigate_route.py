#! /usr/bin/env python3


from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import subprocess
import sys
from math import pi
import random




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

def swarm_size_positions(swarm_size_arg):

    
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
    return robots
    
def starting_navigators(name,x_pos,y_pos,route):

    navigator = BasicNavigator(namespace=name)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x_pos
    initial_pose.pose.position.y = y_pos
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    inspection_pose.pose.orientation.z = 1.0
    inspection_pose.pose.orientation.w = 0.0
    for pt in route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(inspection_pose))
    navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Inspection of shelving failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass



def main():
    rclpy.init()
    inspection_route = [
        [3.461, -0.450],
        [5.531, -0.450],
        [3.461, -2.200],
        [5.531, -2.200],
        [3.661, -4.121],
        [5.431, -4.121],
        [3.661, -5.850],
        [5.431, -5.800]]
    loader_positions = [
        [-7.745750,-3.577660],
        [-7.745750,-1.385260],
        [-7.745750, 0.972970],
        [-7.745750, 3.203130],
        [-7.745750, 5.784420],
        [-7.745750, 8.363370],
    ]
    find_robot_namespaces = "ros2 topic list | grep --only-matching 'Srobot[^/]*' | sort --unique"
    result = subprocess.getoutput(find_robot_namespaces)
    namespaces = []
    namespaces = result.split("\n")
    if namespaces ==[]:
        print('no robots to command')
        exit(-2)
    else:
        swarm_size_positions(len(namespaces))
    for namespace in namespaces:
        print("starting robot " +namespace)
        starting_navigators(namespace,1.0,0.0,loader_positions)
    exit(0)


if __name__ == '__main__':
    main()