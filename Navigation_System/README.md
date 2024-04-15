# Install 

```bash
rosdep install --from-paths src --ignore-src -r -y -i

# Import distribution dependencies
colcon build --event-handlers console_direct+ --symlink-install
```
 
# Run SLAM
## Analyze 
Source `Navigation_System/install/local_setup.bash`

Run the sensors launch script in a board shell, which includes the LIDAR driver and transform from the laser scan to the robot:
```bash
ros2 launch create3_lidar_slam sensors_launch.py
```
In a separate terminal (of board or minipc) run slam toolbox:
```bash
ros2 launch create3_lidar_slam slam_toolbox_launch.py
```
In a third terminal (of board or minipc), drive the robot around:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

In a terminal of your laptop (or your minipc, if it's provided with X server):
```bash
# substitute ID with ROS_DOMAIN_ID of your create3
ROS_DOMAIN_ID=$ID ros2 launch create3_lidar_slam rviz_launch.py
```

## Save map
- Run in the system where Rviz is running:
    ```bash
    # substitute ID with ROS_DOMAIN_ID of your create3
    # the map will be saved in home location, creating two files: map.pgm and map.yaml
    ROS_DOMAIN_ID=$ID 
    ros2 run nav2_map_server map_saver_cli -f ~/map
    ```
  
- Rename the file names of exported files in what you want (for example `my_office.pgm` and `my_office.yaml`)
- Edit the file `my_office.yaml`, changing the property `image` according to pgm file name (for example to `my_office.pgm`)
- Copy the files to `Navigation_System/src/nav_system/params`
- Rebuild the project Navigation_System

# Run navigation
- Tuning the navigation parameters of file `Navigation_System/src/nav_system/params/localization.yaml`. (optionally)
- Edit the file `Navigation_System/src/nav_system/params/localization.yaml` with the initial position of Create3.
- Rebuild the project Navigation_System
- Source `Navigation_System/install/local_setup.bash`
- Run in a terminal of your minipc connected to create3
    ```bash
    # substitute the map path 
    ros2 launch nav_system nav2_lidar.launch.py map:=/opt/projects/robobase/NODES/Navigation_System/src/nav_system/params/my_office.yaml
    ```
- Run in a terminal of your laptop (or your minipc, if it's provided with X server):
    ```bash
    # substitute ID with ROS_DOMAIN_ID of your create3 
    ROS_DOMAIN_ID=1 ros2 launch nav2_bringup rviz_launch.py 
    ```
- Use Rviz to set initial position of robot (if needed, using *2D Pose Estimate button*) and set nav2 goal (using *2D Goal Pose* or *Nav2 Goal*)
