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

from std_msgs.msg import Float32, Header, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import Dock
from irobot_create_msgs.msg import LightringLeds, LedColor, AudioNoteVector, AudioNote
from irobot_create_msgs.srv import EStop
from builtin_interfaces.msg import Time
from nav_system_interfaces.msg import Goal, NavigatorFeedback
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class DemoNavigator(Navigator):
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
        self.broken = False
        self.led_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.audio_publisher = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.set_estop_srv = self.create_client(EStop, 'e_stop')
        self.subscription = self.create_subscription(
            String,
            'emulate_failure',
            self.emulate_failure,
            2)

    def emulate_failure(self, msg):
        """!
        Activate Estop and set color led to orange.
        """
        if self.broken:
            self.broken = False
            self.led_publisher.publish(LightringLeds())
            self.get_logger().info('Estop deactivated.')
            self.play_ok_sound()
            return

        while not self.set_estop_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.navigating = False
        self.broken = True
        
        self.change_led_color()
        self.play_ko_sound()
        self.nav.cancelTask()

        # Publish goal failed
        self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_FAILED, self.current_step)
        self.get_logger().info('Estop activated.')

    def play_ok_sound(self, silent_mode=True):
        if silent_mode:
          return
        sound_msg = AudioNoteVector()
        notes = []
        note_1 = AudioNote()
        note_1.frequency = 300  # Set an high frequency (300 Hz)
        note_1.max_runtime.sec = 1  # Set a long duration (1 sec)
        note_1.max_runtime.nanosec = 0
        note_2 = AudioNote()
        note_2.frequency = 200  # Set an high frequency (200 Hz)
        note_2.max_runtime.sec = 0  # Set a short duration (0.25 sec)
        note_2.max_runtime.nanosec = 250000000
        note_3 = AudioNote()
        note_3.frequency = 300  # Set an high frequency  (300 Hz)
        note_3.max_runtime.sec = 0  # Set a short duration (0.5 sec)
        note_3.max_runtime.nanosec = 500000000
        notes.append(note_1)
        notes.append(note_2)
        notes.append(note_3)
        sound_msg.notes = notes
        self.audio_publisher.publish(sound_msg)

    def play_ko_sound(self, silent_mode=True):
        if silent_mode:
          return
        error_sound_msg = AudioNoteVector()
        error_notes = []
        error_note_1 = AudioNote()
        error_note_1.frequency = 100  # Set a low frequency (100 Hz)
        error_note_1.max_runtime.sec = 1  # Set a short duration (1sec)
        error_note_1.max_runtime.nanosec = 0
        error_note_2 = AudioNote()
        error_note_2.frequency = 100  # Set a low frequency (100 Hz)
        error_note_2.max_runtime.sec = 0  # Set a short duration (0.0sec)
        error_note_2.max_runtime.nanosec = 500000000
        error_notes.append(error_note_1)
        error_notes.append(error_note_2)
        error_sound_msg.notes = error_notes
        self.audio_publisher.publish(error_sound_msg)


    def change_led_color(self):
        msg = LightringLeds()
        msg.header=Header(stamp=Time(sec=0, nanosec=0), frame_id='')
        #ORANGE
        msg.leds=[
                LedColor(red=255, green=0, blue=0),
                LedColor(red=255, green=57, blue=0),
                LedColor(red=255, green=0, blue=0),
                LedColor(red=255, green=57, blue=0),
                LedColor(red=255, green=0, blue=0),
                LedColor(red=255, green=57, blue=0)
                ]
        msg.override_system=True
        self.led_publisher.publish(msg)

    def new_goal(self, msg):
        super().new_goal(msg)
        if self.broken:
            self.navigating = False
            self.nav.cancelTask()
            self.publish_feedback(self.current_goal_id, self.MSG_TYPE_NAV, self.GOAL_FAILED, self.current_step)
            self.get_logger().info('Robot is still broken.')

def main(args=sys.argv):
    rclpy.init(args=args)
    navigator = BasicNavigator(namespace=args[1])
    poc_navigator = DemoNavigator(namespace=args[1], nav=navigator)
    rclpy.spin(poc_navigator)
    poc_navigator.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
