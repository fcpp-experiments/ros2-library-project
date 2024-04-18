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
# Source code use for a physical Create3 Robot with NAV2

import sys, math, rclpy, time
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from math import cos, sin, pi
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from threading import Lock, Thread

from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_system_interfaces.msg import Goal, NavigatorFeedback
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class Navigator(Node):
    """!
    Class used for issuing navigation goal using NAV2.
    """

    # Constants
    GOAL_REACHED    : int = 0
    GOAL_ABORTED    : int = 1
    GOAL_FAILED     : int = 2
    GOAL_RUNNING    : int = 3
    GOAL_UNKNOWN    : int = 4
    GOAL_ILLEGAL    : int = 5
    GOAL_NONE       : int = -1

    DOCK_REACHED    : int = 0
    DOCK_ABORTED    : int = 1
    DOCK_FAILED     : int = 2
    DOCK_RUNNING    : int = 3
    DOCK_UNKNOWN    : int = 4
    DOCK_NONE       : int = -1

    DOCK_DISABLED           : int = 0
    DOCK_ENABLED_NO_LINK    : int = 1
    DOCK_ENABLED_WITH_LINK  : int = 2

    MSG_TYPE_NAV   = "nav"
    MSG_TYPE_DOCK  = "dock"

    # Fields
    x: float
    y: float
    yaw: float
    nav: BasicNavigator
    navigating: bool

    def __init__(self, namespace, nav):
        """!
        PocNavigator constructor requires a namespace for topics and a
        nav2_simple_commander.robot_navigator.BasicNavigator node.
        """
        super().__init__('navigator', namespace=namespace)
        self.goal_subscription = self.create_subscription(
            Goal, 'ap_goal', self.new_goal, 10)
        self.abort_subscription = self.create_subscription(
            Goal, 'ap_abort', self.abort_goal, 10)
        self.timer = self.create_timer(1.0, self.check_navigation_status)
        self.publisher_ = self.create_publisher(NavigatorFeedback, 'navigator_state', 10)
        self.x = self.declare_parameter('x', 0.0)
        self.y = self.declare_parameter('y', 0.0)
        self.yaw = self.declare_parameter('yaw', 0.0)
        self.sleep_time = self.declare_parameter('sleep_time', 0)
        self.declare_parameter('dock_enabled', 0,
                ParameterDescriptor(name='dock_enable',
                    type=ParameterType.PARAMETER_INTEGER, read_only=False
                    ))
        self.current_goal_id = ""
        self.current_step = 0
        self.nav = nav
        self.poses = []
        self.navigating = False
        self.going_home = False
        self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_NONE)
        if self.dock_enabled():
            self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_REACHED)
        else:
            self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_NONE)

    def on_goal_start(self):
        pass

    def on_hallway_start(self):
        pass

    def on_dock_reached(self):
        self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_REACHED)

    def on_goal_step(self):
        pass

    def on_goal_reached(self):
        pass

    def get_initial_pose(self):
        return {'x': self.x.get_parameter_value().double_value,
                'y': self.y.get_parameter_value().double_value,
                'yaw': self.yaw.get_parameter_value().double_value
                }

    def get_sleep_time(self):
        return self.get_parameter('sleep_time').get_parameter_value().integer_value

    def dock_enabled(self):
        val = self.get_parameter('dock_enabled').get_parameter_value().integer_value
        return val > 0

    def yaw_to_quaternion(self, yaw):
        return {'qz': sin((yaw % (2*pi))/2),
                'qw': cos((yaw % (2*pi))/2)}

    def create_pose(self, timestamp, x, y, qz, qw, frame_id='map'):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = timestamp
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        return goal_pose

    def publish_feedback(self, _id, _type, state, step=0):
        msg = NavigatorFeedback()
        msg.id = _id
        msg.type = _type
        msg.state = state
        msg.current_step = step
        self.publisher_.publish(msg)

    def is_nan(self, target):
        return target != target

    def new_goal(self, msg):
        """!
        Given a nav_system_interfaces.msg.Goal message sends the robot to
        the requested coordinates or executes a dock/undock action.
        """
        self.current_step = 0
        if msg.type == 'GOAL':
            self.on_goal_start()
            # Wait for self.navigation to fully activate, since autostarting nav2
            self.nav.waitUntilNav2Active()
            self.current_goal_id = msg.goal_id

            # Set goal pose
            goal_pose = self.create_pose(self.nav.get_clock().now().to_msg(),
                    msg.pos_end_x, msg.pos_end_y, msg.pos_end_qz, msg.pos_end_qw)
            self.get_logger().info('Publishing: "%s"' % goal_pose)
            path = self.nav.getPath(PoseStamped(), goal_pose)
            if path:
                self.nav.goToPose(goal_pose)
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_RUNNING)
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_NONE)
                self.navigating = True
                self.poses = [goal_pose]
            else:
                self.get_logger().warn('No valid path found')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_RUNNING)
                time.sleep(1)
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_FAILED)
        if msg.type == 'DOCK':
            self.go_to_initial_pose()

    def go_to_initial_pose(self):
            #self.get_logger().info('Requesting dock action')
            pos = self.get_initial_pose()
            rot = self.yaw_to_quaternion(pos['yaw'])
            goal_pose = self.create_pose(self.nav.get_clock().now().to_msg(),
                    pos['x'], pos['y'], rot['qz'], rot['qw'])
            self.get_logger().info('Publishing: "%s"' % goal_pose)
            if self.dock_enabled():
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_RUNNING)
            self.nav.goToPose(goal_pose)

    def check_navigation_status(self):
        """!
        Verify current navigation state, publishing the navigator_state messages
        accordingly.
        Called once per second.
        """
        if self.dock_enabled() and self.going_home and self.nav.isTaskComplete():
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Dock was reached!')
                self.on_dock_reached()
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Dock was canceled!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_ABORTED)
            elif result == TaskResult.FAILED:
                self.get_logger().info('Dock failed!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_FAILED)
            else:
                self.get_logger().info('Dock has an invalid return status!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_DOCK, self.DOCK_UNKNOWN)
            self.going_home = False

        if self.navigating and self.nav.isTaskComplete():
            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.poses = self.poses[1:]
                self.current_step += 1
                self.on_goal_step()
                time.sleep(self.get_sleep_time())
                if len(self.poses) == 0:
                    self.get_logger().info('Goal succeeded!')
                    self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_REACHED,
                            self.current_step)
                    self.on_goal_reached()
                    if self.dock_enabled():
                        self.get_logger().info('Going back to dock')
                        self.go_to_initial_pose()
                        self.going_home = True
                else:
                    self.poses[0].header.stamp = self.nav.get_clock().now().to_msg()
                    self.nav.goToPose(self.poses[0])
                    self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_RUNNING,
                            self.current_step)
            elif result == TaskResult.CANCELED:
                self.poses = []
                self.get_logger().info('Goal was canceled!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_ABORTED,
                        self.current_step)
            elif result == TaskResult.FAILED:
                self.poses = []
                self.get_logger().info('Goal failed!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_FAILED,
                        self.current_step)
            else:
                self.poses = []
                self.get_logger().info('Goal has an invalid return status!')
                self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_UNKNOWN,
                        self.current_step)
        if len(self.poses) == 0:
            self.navigating = False

    def abort_goal(self, msg):
        """!
        Cancel the current goal.
        Subscribed to ap_abort.
        """
        if (self.current_goal_id == msg.goal_id):
            self.get_logger().info('Aborting goal: "%s"' % msg.goal_id)
            self.nav.cancelTask()
        else:
            self.get_logger().info('Ignore abort goal: "%s"' % msg.goal_id)

def main(args=sys.argv):
    rclpy.init(args=args)
    navigator = BasicNavigator(namespace=args[1])
    poc_navigator = Navigator(namespace=args[1], nav=navigator)
    rclpy.spin(poc_navigator)
    poc_navigator.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
