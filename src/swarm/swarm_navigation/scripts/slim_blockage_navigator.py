import time
from copy import deepcopy
from geographic_msgs.msg import GeoPoseStamped
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

    navigator = BasicNavigator()
    