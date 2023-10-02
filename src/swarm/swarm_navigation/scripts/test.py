#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations


def castToPose(
    nav: BasicNavigator, x_pos: float, y_pos: float, yaw: float
) -> PoseStamped:
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = nav.get_clock().now().to_msg()

    goal_pose.pose.position.x = x_pos
    goal_pose.pose.position.y = y_pos
    goal_pose.pose.position.z = 0.01

    x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

    goal_pose.pose.orientation.x = x
    goal_pose.pose.orientation.y = y
    goal_pose.pose.orientation.z = z
    goal_pose.pose.orientation.w = w

    return goal_pose


def main():
    rclpy.init()

    nav = BasicNavigator(namespace="robot1")
    # print("sending initial goal:")
    nav.setInitialPose(castToPose(nav=nav, x_pos=0.0, y_pos=0.5, yaw=0.00))
    # print("initial goal sent!")
    nav.waitUntilNav2Active()
    print("done waitng nav2 active!")
    # ---------------------- single goal pose -------------------------

    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = nav.get_clock().now().to_msg()

    # goal_pose.pose.position.x = 2.0
    # goal_pose.pose.position.y = 1.0
    # goal_pose.pose.position.z = 0.0

    # x,y,z,w = tf_transformations.quaternion_from_euler(0.0,0.0,1.57)

    # goal_pose.pose.orientation.x = x
    # goal_pose.pose.orientation.y = y
    # goal_pose.pose.orientation.z = z
    # goal_pose.pose.orientation.w = w

    # nav.goToPose(goal_pose)

    # -----------------------------------------------------------------

    gp1 = castToPose(nav=nav, x_pos=2.0, y_pos=0.0, yaw=0.0)
    gp2 = castToPose(nav=nav, x_pos=0.5, y_pos=0.5, yaw=0.0)
    gp3 = castToPose(nav=nav, x_pos=0.0, y_pos=2.0, yaw=0.0)
    waypoints = [gp1, gp2, gp3]

    print("Node active sending follow wp req")
    # nav.followWaypoints(waypoints)
    nav.goToPose(gp1)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)

    rclpy.shutdown()


if __name__ == "__main__":
    main()