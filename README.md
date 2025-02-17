# ROS2 Library Project
Components for ROS2 Library Project

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
To run this application and bash scripts included, you should use a UNIX operating system: we suggest Ubuntu 22.04 Jammy for best integration with ROS2 Humble. 

#### ROS 2

[ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  
[ROS 2 DDS tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)

```bash
# in ubuntu, before you should enable multiverse repository

# Build deps
apt install python3-colcon-common-extensions python3-vcstool python3-rosdep ros-humble-irobot-create-msgs

# Cyclone DDS
apt install ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp
```

Add to your `.bashrc` file the cyclone config file (you should fix the path according to your project path):
```bash
# set $LIBRARY_PROJECT_PATH according with your installation
export LIBRARY_PROJECT_PATH=XXXXXXX
export CYCLONEDDS_URI=$LIBRARY_PROJECT_PATH/config/cyclone-dds-interface-select-laptop.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$LIBRARY_PROJECT_PATH
```

Source file ~/.bashrc:
```bash
source ~/.bashrc
```

You should now run install script to build all ROS2 components:
```bash
cd PoC
./install.sh
```

The script will list the components that need to be compiled to run,
enter the folders and follow compilation instructions.

#### FCPP
Install prerequisites on:

- [PoC/AP_Engine/fcpp/README.md](PoC/AP_Engine/fcpp/README.md)
- and install following dependencies:
```bash
sudo apt-get install clang libstdc++-12-dev
```

# Run
To launch a simulation execute from the root of the repository:

## Simulation
You can use enable or disable automatic dock after reaching goal.    
`$DOCK` can be 1 (disable), 2 (enable).

```bash
./PoC/rumbo_run.sh dock_enabled:=$DOCK
```

## AP Engine

To compile and execute the AP engine with library use case, you should: 
- follow instructions on [/PoC/AP_Engine/README.MD](/PoC/AP_Engine/README.MD)
- or run on a new terminal window (but with logs deleted after each round):
```bash
cd PoC/AP_Engine
./ap_run.sh
```

You can also pass as first argument (`true` or `false`) to clean build directory, as below:
```bash
cd PoC/AP_Engine
./ap_run.sh true
```

### Network partition
If you want to simulate a network partition, you can:
- Run gazebo (see specific README)
- Run AP with custom configuration to reduce communication range instead of "default" script described before:

```bash
./ap_run_network_partition.sh
```
- Create a goal for shelf n°7:
```bash
cd Storage
./create_goal.sh "0.2; 5.0; -1.57"
```

## Use case

### Library
To create a new goal with *X=`$POS_X`, Y=`$POS_Y`, YAW=`$ORIENT`* (all coordinates are float values), you can run:
```bash
cd Storage
./create_goal.sh "$POS_X;$POS_Y;$ORIENT"
```

example:
```
./create_goal.sh "1.0;2.0;3.14"
```

### Abort a goal
To abort a current with *ID=`$GOAL_CODE`*, you can run:
```bash
cd Storage
./create_abort.sh $GOAL_CODE
```

example:
```
./create_abort.sh GOAL-123456789
```

### Out of order of a robot
If you want to simulate a robot failure, you can run this script:
```bash
### Out of order robot
# $ROBOT_NAME can be tb3_1, tb3_2 etc...
./out_of_order.sh $ROBOT_NAME
```

## Library world
The world is a simulated library. The shelfs can be located at:

| Bookshelf | Position         |
|-----------|-----------------|
| ID 8      | "0.2; 5.5; 1.57" |
| ID 7      | "0.2; 5.0; -1.57" |
| ID 6      | "0.2; 4.1; 1.57" |
| ID 5      | "0.2; 3.5; -1.57" |
| ID 4      | "0.2; 2.5; 1.57" |
| ID 3      | "0.2; 2.0; -1.57" |
| ID 2      | "0.2; 1.0; 1.57" |
| ID 1      | "0.2; 0.3; -1.57" |
