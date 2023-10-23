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
    request_item_location = 'loader_3'
    request_destination = 'main_shipping'

    rclpy.init()
    navigator = BasicNavigator(namespace="Srobot1")
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 1.0
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    loader_item_pose = PoseStamped()
    loader_item_pose.header.frame_id = 'map'
    loader_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    loader_item_pose.pose.position.x = loader_positions[request_item_location][0]
    loader_item_pose.pose.position.y = loader_positions[request_item_location][1]
    loader_item_pose.pose.orientation.z = 1.0
    loader_item_pose.pose.orientation.w = 0.0
    print('recieved request for item picking at ' + request_item_location + '.')
    navigator.goToPose(loader_item_pose)
    i = 0
    while not navigator.isTaskComplete():
        i=i+1
        feedback = navigator.getFeedback()
        if feedback and i % 5 ==0:
            print("estimated time " + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) +' seconds')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("good job")
        shopping_destination_pose = PoseStamped()
        shopping_destination_pose.header.frame_id = 'map'
        shopping_destination_pose.header.stamp = navigator.get_clock().now().to_msg()
        shopping_destination_pose.pose.position.x = shipping_destination[request_destination][0]
        shopping_destination_pose.pose.position.y = shipping_destination[request_destination][1]
        shopping_destination_pose.pose.orientation.z = -1.5
        shopping_destination_pose.pose.orientation.w = 0.0
        print('recieved request for item drop off at ' + request_destination + '.')
        navigator.goToPose(shopping_destination_pose)

    elif result == TaskResult.CANCELED:
        print('task was cancelled')
        navigator.goToPose(initial_pose)
    
    elif result== TaskResult.FAILED:
        print('Task failed')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()