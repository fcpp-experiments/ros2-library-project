# Install 

```bash
rosdep install --from-paths src --ignore-src -r -y -i

# Import distribution dependencies
colcon build --event-handlers console_direct+ --symlink-install
```

To use cyclonedds instead of fastrpts, install:
```bash
sudo apt-get install ros-humble-rmw-cyclonedds-cpp
```
and change some environment variables:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# change path according with your filesystem
export CYCLONEDDS_URI=NODES/config/cyclone-dds-interface-select-laptop.xml
```
