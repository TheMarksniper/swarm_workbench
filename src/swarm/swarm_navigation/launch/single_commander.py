#!/bin/sh
#!/usr/bin/env python3
from copy import deepcopy
import argparse
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import os
import rclpy
from rclpy.duration import Duration
def valid_yaml(param):
    base, ext = os.path.splitext(param)
    if ext.lower() not in '.yaml':
        raise argparse.ArgumentTypeError('File must have a .yaml extension')
    return param

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot", type=str, help="Namespace of the robot you wish to control", required=True
    )
    parser.add_argument(
        "--position", type=float,help="x and y position for robots Initial pose", nargs=2, default=[1, 1], required=False
    )
    parser.add_argument(
        "--route", type=valid_yaml, help="File with the loading points,unloading point and circling points", required=True
    )
    args = parser.parse_args()
    rclpy.init()

    navigator = BasicNavigator(namespace = args.robot)

    # Security route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    route = args.route["loading"]
    unloading = args.route["unloading"]
    circling = args.route["circling"]

    # Set our demo's initial pose
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

    # Do security route until dead
    while rclpy.ok():
        # Send our route
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))
        navigator.goThroughPoses(route_poses)

        # Do something during our route (e.x. AI detection on camera images for anomalies)
        # Simply print ETA for the demonstation
        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                # Some failure mode, must stop since the robot is clearly stuck
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    navigator.cancelTask()
                    navigator.goThroughPoses(circling)

        # If at end of route, reverse the route to restart
        navigator.goToPose(unloading)
        route.reverse()

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Route complete! Restarting...')
        elif result == TaskResult.CANCELED:
            print('Security route was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Security route failed! Restarting from other side...')

    exit(0)


if __name__ == '__main__':
    main()