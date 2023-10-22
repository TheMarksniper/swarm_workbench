#!/bin/sh
#!/usr/bin/env python3
import time
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

#Positions of loaders
loader_positions = {
    "loader_1":[-7.745750,-3.577660],
    "loader_2":[-7.745750,-1.385260],
    "loader_3":[-7.745750, 0.972970],
    "loader_4":[-7.745750, 3.203130],
    "loader_5":[-7.745750, 5.784420],
    "loader_6":[-7.745750, 8.363370],
}
#where the robot is suppoused to after finishing task
shipping_destination = {
    "main_shipping":[7.015590,6.999010]
}

# around what points the robot can circle around
circling_positions = {
    "position_A":[-5.990370, 8.674410],
    "position_B":[-2.706150, 8.674410],
    "position_C":[-2.706150,-3.947350],
    "position_D":[-5.990370,-3.947350],
}

def main():
    rclpy.init()
    print("got here")
    navigator = BasicNavigator(namespace="/Srobot1")
    print("got here")
    #get initial position
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.45
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)
#
    ## Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    print("got here")
    # Do security route until dead
    #while rclpy.ok():
    #    # Send our route
    #    route_poses = []
    #    pose = PoseStamped()
    #    pose.header.frame_id = 'map'
    #    pose.header.stamp = navigator.get_clock().now().to_msg()
    #    pose.pose.orientation.w = 1.0
    #    for pt in loader_positions:
    #        pose.pose.position.x = pt[0]
    #        pose.pose.position.y = pt[1]
    #        route_poses.append(deepcopy(pose))
    #    navigator.goThroughPoses(route_poses)
#
    #    # Do something during our route (e.x. AI detection on camera images for anomalies)
    #    # Simply print ETA for the demonstation
    #    i = 0
    #    while not navigator.isTaskComplete():
    #        i += 1
    #        feedback = navigator.getFeedback()
    #        if feedback and i % 5 == 0:
    #            print('Estimated time to complete current route: ' + '{0:.0f}'.format(
    #                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #                  + ' seconds.')
#
    #            # Some failure mode, must stop since the robot is clearly stuck
    #            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
    #                print('Navigation has exceeded timeout of 180s, canceling request.')
    #                navigator.cancelTask()
#
    #    # If at end of route, reverse the route to restart
    #    loader_positions.reverse()
#
    #    result = navigator.getResult()
    #    if result == TaskResult.SUCCEEDED:
    #        print('Route complete! Restarting...')
    #    elif result == TaskResult.CANCELED:
    #        print('Security route was canceled, exiting.')
    #        exit(1)
    #    elif result == TaskResult.FAILED:
    #        print('Security route failed! Restarting from other side...')

    exit(0)

if __name__ == '__main__':
    main()
    