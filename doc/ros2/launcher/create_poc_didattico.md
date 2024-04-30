## create3\_board.launch.py

Launch ROS 2 Node to run onto a board inside a physical Create3.

- Node
  + robot\_reader.robot\_reader
  + robot\_writer.robot\_writer
  + create3\_poc\_didattico.main\_node
  + inbound\_proxy.inbound\_proxy
  + outbound\_proxy.outbound\_proxy

## create3\_gazebo.launch.py

Launch a simulation of a Create3 inside a Gazebo environment.

- Launch
  + pkg\_create3\_gazebo\_bringup.launch.gazebo.launch.py
  + pkg\_create3\_poc.launch.create3\_navigation\_spawn.launch.py
## create3\_gazebo\_library.launch.py

Launch a simulation of a Create3 inside a Gazebo environment,
with using the library world model.

- Launch
  + pkg\_create3\_poc.launch.create3\_gazebo.launch.py
  + bringup\_dir.worlds.biblioteca.sdf

## create3\_navigation\_spawn.launch.py

Launch ROS 2 Node to run navigation onto a simulated Create3.

- Launch
  + pkg\_create3\_common\_bringup.launch.create3\_nodes.launch.py
  + pkg\_create3\_common\_bringup.launch.dock\_description.launch.py
  + pkg\_create3\_common\_bringup.launch.robot\_description.launch.py
  + pkg\_create3\_common\_bringup.launch.rviz2.launch.py
- Node
  + gazebo\_ros.spawn\_entity.py
  + robot\_reader.robot\_reader
  + robot\_writer.robot\_writer
  + create3\_poc\_didattico.main\_node
  + gazebo\_ros.spawn\_entity.py
  + inbound\_proxy.inbound\_proxy
  + outbound\_proxy.outbound\_proxy

## create3\_physical\_robot.launch.py

Launch a physical Create3.

- Launch
  + pkg\_create3\_gazebo\_bringup.launch.gazebo.launch.p
  + pkg\_create3\_poc.launch.create3\_physical\_spawn.launch.py

## create3\_physical\_spawn.launch.py

Launch ROS 2 Node to run navigation onto a physical Create3.

- Launch
  + pkg\_create3\_common\_bringup.launch.rviz2.launch.py
  + pkg\_create3\_common\_bringup.launch.robot\_description.launch.py
  + pkg\_create3\_common\_bringup.launch.dock\_description.launch.py
- Node
  + robot\_reader.robot\_reader
  + robot\_writer.robot\_writer
  + create3\_poc\_didattico.main\_node
  + inbound\_proxy.inbound\_proxy
  + outbound\_proxy.outbound\_proxy

## navigation\_node\_launch.py

Launch ROS 2 Node to run navigation onto a Create3.
<!--TODO potrebbe essere obsoleto-->

- Node
  - create3\_poc\_didattico.main\_node
