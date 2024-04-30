import os
import sys
import csv
from datetime import datetime, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_system_interfaces.msg import Goal
from pathlib import Path
from . import coords

STORAGE_BASE_PATH           = "../../../../Storage/"
ROBOT_PLACEHOLDER           = "#ROBOT"
ROBOT_INPUT_PATH_FROM_AP    = "from_ap/to_robot/actions/#ROBOT/"
DELIMITER                   = ";"
TIMER_PERIOD                = 0.5


class GoalPublisher(Node):
    """!
    Class used for publishing navigation goal with coordinates compatible
    with the selected robot.
    """

    def __init__(self, namespace, robot_name, origin_x, origin_y, rotation):
        """!
        Class constructor requires a namespace, x and y values (in meters)
        compared to the global origin and the rotation in radians.
        With a rotation of 0.0 the robot is initially faced along the x axis,
        a rotation of 1.57 the robot is initially faced along the y axis.
        Declare a ROS 2 parameter backup_storage, when True store old action
        file into a backup folder.
        """
        super().__init__('goal_publisher', namespace=namespace)
        self.goal_publisher_ = self.create_publisher(Goal, 'ap_goal', 10)
        self.abort_publisher_ = self.create_publisher(Goal, 'ap_abort', 10)
        self.declare_parameter('backup_storage', False)
        self.timer = self.create_timer(TIMER_PERIOD, self.publish_goal)
        self.robot_name = robot_name
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.rotation = float(rotation)

    def publish_goal(self):
        """!
        Prepare and publish messages on ap_goal and ap_abort.
        """
        actions = self.read_file(self.robot_name)
        for act in actions:
            msg = Goal()
            msg.type = act['action']
            msg.goal_id = act['goal']
            msg.pos_end_x = act['pos_end_x']
            msg.pos_end_y = act['pos_end_y']
            msg.pos_end_qz = act['orientation_end_z']
            msg.pos_end_qw = act['orientation_end_w']
            msg.pos_start_x = act['pos_start_x']
            msg.pos_start_y = act['pos_start_y']
            msg.pos_start_qz = act['orientation_start_z']
            msg.pos_start_qw = act['orientation_start_w']
            msg.step = act['step']
            if act['action'] in ['GOAL', 'DOCK', 'UNDOCK', 'HALLWAY']:
                self.goal_publisher_.publish(msg)
                self.get_logger().info('Publishing GOAL: "%s"' % act['goal'])
            if act['action'] in ['SOS']:
                self.goal_publisher_.publish(msg)
                self.get_logger().info('Publishing SOS: "%s"' % act['goal'])    
            if act['action'] == 'ABORT':
                self.abort_publisher_.publish(msg)
                self.get_logger().info('Publishing ABORT: "%s"' % act['goal'])

    def read_file(self, robot):
        """!
        Search for action goal files in the configured path,
        returning them into an array called actions.
        """
        target = ROBOT_INPUT_PATH_FROM_AP.replace(ROBOT_PLACEHOLDER, robot)
        path_tmp = os.path.join(STORAGE_BASE_PATH, target)
        actions = []

        # create folder if not exists
        path_actions = Path(__file__).parent / path_tmp
        path_actions.parent.mkdir(exist_ok=True, parents=True)
        # create backup folder if not exists
        path_actions_backup = path_actions / "backup"
        path_actions_backup.mkdir(exist_ok=True, parents=True)

        path = path_actions.absolute()

        files = [f for f in os.listdir(path) if f.endswith('.txt')]
        # sort files by (creation_time, file_name)
        files_sorted = sorted(files, key=lambda f: (os.path.getctime(os.path.join(path, f)), f))

        for file_name in files_sorted:
            with open(os.path.join(path, file_name), 'r') as action_file:
                reader = csv.reader(action_file, delimiter=DELIMITER)
                for row in reader:
                    sx, sy, sqz, sqw = coords.abs2rel(float(row[3]), float(row[4]), float(row[5]),
                            self.origin_x, self.origin_y, self.rotation)
                    ex, ey, eqz, eqw = coords.abs2rel(float(row[6]), float(row[7]), float(row[8]),
                            self.origin_x, self.origin_y, self.rotation)
                    step = int(row[9])
                    self.get_logger().debug('"computed coordinates %s"' % str([ex, ey, eqz, eqw]))
                    actions.append({ 'action': row[0],
                            'goal': row[1], 'robot': row[2],
                            'pos_end_x': ex, 'pos_end_y': ey,
                            'orientation_end_z': eqz, 'orientation_end_w': eqw,
                            'pos_start_x': sx, 'pos_start_y': sy,
                            'orientation_start_z': sqz, 'orientation_start_w': sqw,
                            'step': step
                            })
                if (os.path.isdir(os.path.join(path, 'backup')) and
                    self.get_parameter('backup_storage').get_parameter_value().bool_value):
                        os.replace(os.path.join(path, file_name), os.path.join(path, 'backup',
                        file_name))
                else:
                    try:
                        os.remove(os.path.join(path, file_name))
                    except FileNotFoundError:
                        self.get_logger().warn('Trying to delete nonexisting goal file')

        return(actions)

def main(args=sys.argv):
    rclpy.init(args=args)
    if(len(args) > 5):
        goal_publisher = GoalPublisher(namespace=args[1], robot_name=args[2],
                origin_x=args[3], origin_y=args[4], rotation=args[5])
    else:
        goal_publisher = GoalPublisher(namespace=args[1], robot_name=args[2], origin_x=0.0,
                origin_y=0.0, rotation=0.0)
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
