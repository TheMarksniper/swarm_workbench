#! /usr/bin/env python3


from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import argparse
import subprocess
from rclpy.duration import Duration
from math import pi
import random


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot", type=str, help="Namespace of the robot you wish to control", required=True
    )
    parser.add_argument(
        "--position", type=float,help="x and y position for robots Initial pose", nargs=2, default=[1.0, 1.0], required=False
    )
    args = parser.parse_args()
    rclpy.init()
    find_robot_namespaces = "ros2 topic list | grep --only-matching 'Srobot[^/]*' | sort --unique"
    #os.system(find_robot_namespaces)
    result = subprocess.getoutput(find_robot_namespaces)
    namespaces = result.split("\n")
    print(namespaces)
    if args.robot in namespaces:
            print("good namespace")
    else:
            print("bad namespace")
            exit(-3)
    circling = [
    [-5.990370, 8.674410],
    [-2.706150, 8.674410],
    [-2.706150,-3.947350],
    [-5.990370,-3.947350],
    ]
    unload = [5.0, 5.0]

    loader_positions = [
        [-7.745750,-3.577660],
        [-7.745750,-1.385260],
        [-7.745750, 0.972970],
        [-7.745750, 3.203130],
        [-7.745750, 5.784420],
        [-7.745750, 8.363370],
    ]
   
    navigator = BasicNavigator(namespace=args.robot)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = args.position[0]
    initial_pose.pose.position.y = args.position[1]
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
            print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')
            
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    navigator.cancelTask()
                    

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Loading complete! going to unload...')
        navigator.goToPose(unload)
    elif result == TaskResult.CANCELED:
        print('Loading was canceled. Circling to try again...')
        navigator.goThroughPoses(circling)
    elif result == TaskResult.FAILED:
        print('Loading failed! Returning to start...')

    # go back to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()