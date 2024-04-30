# Copyright © 2023 University of Turin, Daniele Bortoluzzi & Giordano Scarso. All Rights Reserved.

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped

import time, threading, string

from . import store
from . import config
from . import coords
from .reader import RobotInformationReader
from .position import PositionDTO
from .battery import BatteryDTO
from .goal import GoalDTO
from .feedback_writer import FeedbackWriter

class OdomRobotInformationReader(RobotInformationReader):
    """!
    Class extending RobotInformationReader to receive position data from the /odom topic
    """

    def position_subscribe(self, namespace):
        # subscribe odometry
        self.get_logger().info('Subscribing to: "%s/%s"' % (namespace, config.ODOM_TOPIC))
        return self.create_subscription(
                Odometry,
                config.ODOM_TOPIC,
                self.listener_position_callback,
                config.DEFAULT_QOS_TOPIC
                )

def main(args=sys.argv):
    rclpy.init(args=args)
    if(len(args) > 4):
        robot_information_reader = OdomRobotInformationReader(namespace=args[1],
                robot_name=args[2], origin_x=args[3], origin_y=args[4],
                rotation=args[5])
    else:
        robot_information_reader = OdomRobotInformationReader(namespace=args[1],
                robot_name=args[2])
    rclpy.spin(robot_information_reader)
    robot_information_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
