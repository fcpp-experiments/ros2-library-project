# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified by: Giordano Scarso
# Source code use for a simulated Turtlebot3 Robot with NAV2

import sys, math, rclpy, time
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from math import cos, sin, pi
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from threading import Lock, Thread

from .navigator import Navigator

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import Dock
from battery_services.srv import SetCharge
from nav_system_interfaces.msg import Goal, NavigatorFeedback
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PocNavigator(Navigator):
    """!
    Class used for issuing navigation goal using NAV2.
    """

    x: float
    y: float
    yaw: float
    nav: BasicNavigator
    navigating: bool

    def __init__(self, namespace, nav):
        """!
        PocNavigator constructor requires a namespace for topics and a
        nav2_simple_commander.robot_navigator.BasicNavigator node.
        Declares the node parameter decrease_battery to enable battery level
        decrease whenever a navigation goal is reached.
        """
        super().__init__(namespace=namespace, nav=nav)
        self.declare_parameter('decrease_battery', False)
        self.set_charge_battery_srv = self.create_client(SetCharge, 'gazebo_ros_battery/set_charge')

    def on_goal_start(self):
        self.nav.clearAllCostmaps()

    def on_hallway_start(self):
        pass

    def on_dock_reached(self):
        self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_REACHED)

    def on_goal_reached(self):
        if self.get_parameter('decrease_battery').get_parameter_value().bool_value:
            self.decrease_battery()

    def on_goal_step(self):
        pass

    def decrease_battery(self):
        """!
        Decrease the battery level of -0.1 units.
        Executed if decrease_battery is True.
        """
        while not self.set_charge_battery_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetCharge.Request()
        self.req.charge = Float32()
        self.req.charge.data = -0.1
        self.future = self.set_charge_battery_srv.call_async(self.req)
        self.get_logger().info('Decreasing battery after reaching goal.')

def main(args=sys.argv):
    rclpy.init(args=args)
    navigator = BasicNavigator(namespace=args[1])
    poc_navigator = PocNavigator(namespace=args[1], nav=navigator)
    rclpy.spin(poc_navigator)
    poc_navigator.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
