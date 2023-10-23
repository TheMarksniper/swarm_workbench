#! /usr/bin/env python3


from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import subprocess
import sys
from math import pi
import random
import re

def main():
    rclpy.init()
    find_robot_namespaces = "ros2 topic list | grep --only-matching 'Srobot[^/]*' | sort --unique"
    #os.system(find_robot_namespaces)
    result = subprocess.getoutput(find_robot_namespaces)
    namespaces = result.split("\n")
    for i in range(1, len(sys.argv)):
        print(sys.argv[i])
        if sys.argv[i] in namespaces:
            print("good namespace")
            name = sys.argv[i]
            j = re.findall(r'\d+', name)
            print(float(j[0]))
            x_pos = -1.0 + float(j[0])
        else:
            print("bad namespace")
            exit(-3)
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
   
    navigator = BasicNavigator(namespace=name)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = x_pos
    initial_pose.pose.position.y = 1.0
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
    random.shuffle(loader_positions)
    for pt in loader_positions:
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

    exit(0)


if __name__ == '__main__':
    main()