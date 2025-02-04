#!/bin/bash
source Battery/install/local_setup.bash
robot_code=$1
ros2 service call /$robot_code/gazebo_ros_battery/set_charge battery_services/srv/SetCharge "{charge: {data: -4.0}}"
