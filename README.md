# ROS2 Sample Project
Components for ROS2 Sample Project

## Documentation

Documentation is available under doc folder:
- *ros2*: documentation of components written for ROS2 (humble version)
- *usecase*: some sequence diagrams to depict the relation of components
- *doxygen*: components documented using doxygen (https://www.doxygen.nl/). 

The doxygen documentation can be compiled in /doc/doxygen folder, using
```bash
doxygen doc/Doxyfile 
```
It can be viewed using [/doc/doxygen/html/index.html](/doc/doxygen/html/index.html)

## TL;DR

### Dependencies

#### ROS 2

[ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  
[ROS 2 DDS tuning](https://docs.ros.org/en/foxy/How-To-Guides/DDS-tuning.html)

```bash
# in ubuntu, before you should enable multiverse repository

# Build deps
apt install python3-colcon-common-extensions python3-vcstool python3-rosdep ros-humble-irobot-create-msgs

# Cyclone DDS
apt install ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp
```
___

# TODO

### Simulation

To launch a simulation execute from the root of the repository:

```bash
./PoC/turtlebot3_run.sh
```

The script will list the components that need to be compiled to run,
enter the folders and follow compilation instructions.

#### Simulated SLAM

To perform a SLAM of a simulated environment use the following command:

```bash
WORLD_PATH=/home/ws/NODES/use_case_resources/hospital/world.sdf ros2 launch turtlebot3_gazebo turtlebot3_slam.launch.py y_pose:=3.0 x_pose:=1.0
```

This will load the world specified in WORLD\_PATH spawning a Turtlebot3 in position (x\_pose, y\_pose).

On another terminal launch the mapping node:

```bash
TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

On another terminal launch the keyboard control node:

```bash
TURTLEBOT3_MODEL=burger ros2 run turtlebot3_teleop teleop_keyboard
```

When the map is complete save it using the following command:

```bash
ros2 run nav2_map_server map_saver_cli -f map-filename
```

### AP Engine

To compile and execute the AP engine , you should
: 
- follow instructions on [/PoC/AP_Engine/README.MD](/PoC/AP_Engine/README.MD)
- or run on a new terminal window (but with logs deleted after each round):
```bash
cd PoC/AP_Engine
./ap_run.sh
```