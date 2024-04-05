## bringup\_launch.py

Launch NAV 2 localization and navigation components.

* Node
  - name=nav2\_container, rclcpp\_components.component\_container\_isolated
* Launch
  - turtlebot\_gazebo\_dir.launch.localization\_launch.py
  - launch\_dir.navigation\_launch.py
* Arguments
  - 'namespace':
    + Top-level namespace
    + (default: '')
  - 'use\_namespace':
    + Whether to apply a namespace to the navigation stack
    + (default: 'false')
  - 'slam':
    + Whether run a SLAM
    + (default: 'False')
  - 'map':
    + Full path to map yaml file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/maps/library-new.yaml')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'params\_file':
    + Full path to the ROS2 parameters file to use for all launched nodes
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/params/nav2\_params.yaml')
  - 'autostart':
    + Automatically startup the nav2 stack
    + (default: 'true')
  - 'use\_composition':
    + Whether to use composed bringup
    + (default: 'True')
  - 'use\_respawn':
    + Whether to respawn if a node crashes. Applied when composition is disabled.
    + (default: 'False')
  - 'log\_level':
    + log level
    + (default: 'info')
  - 'container\_name':
    + the name of conatiner that nodes will load in if use composition
    + (default: 'nav2\_container')

## empty\_world.launch.py

Launch a turtlebot with navigation into a empty world.

* Launch
  - pkg\_gazebo\_ros.launch.gzserver.launch.py
  - pkg\_gazebo\_ros.launch.gzclient.launch.py
  - launch\_file\_dir.robot\_state\_publisher.launch.py
  - launch\_file\_dir.spawn\_turtlebot3.launch.py
* Arguments
  - 'world':
    + Specify world file name
    + (default: '')
  - 'version':
    + Set "true" to output version information.
    + (default: 'false')
  - 'verbose':
    + Set "true" to increase messages written to terminal.
    + (default: 'false')
  - 'lockstep':
    + Set "true" to respect update rates
    + (default: 'false')
  - 'help':
    + Set "true" to produce gzserver help message.
    + (default: 'false')
  - 'pause':
    + Set "true" to start the server in a paused state.
    + (default: 'false')
  - 'physics':
    + Specify a physics engine (ode|bullet|dart|simbody).
    + (default: '')
  - 'play':
    + Play the specified log file.
    + (default: '')
  - 'record':
    + Set "true" to record state data.
    + (default: 'false')
  - 'record\_encoding':
    + Specify compression encoding format for log data (zlib|bz2|txt).
    + (default: '')
  - 'record\_path':
    + Absolute path in which to store state data.
    + (default: '')
  - 'record\_period':
    + Specify recording period (seconds).
    + (default: '')
  - 'record\_filter':
    + Specify recording filter (supports wildcard and regular expression).
    + (default: '')
  - 'seed':
    + Start with a given a random number seed.
    + (default: '')
  - 'iters':
    + Specify number of iterations to simulate.
    + (default: '')
  - 'minimal\_comms':
    + Set "true" to reduce TCP/IP traffic output.
    + (default: 'false')
  - 'profile':
    + Specify physics preset profile name from the options in the world file.
    + (default: '')
  - 'extra\_gazebo\_args':
    + Extra arguments to be passed to Gazebo
    + (default: '')
  - 'params\_file':
    + Path to ROS 2 yaml parameter file
    + (default: '')
  - 'gdb':
    + Set "true" to run gzserver with gdb
    + (default: 'false')
  - 'valgrind':
    + Set "true" to run gzserver with valgrind
    + (default: 'false')
  - 'init':
    + Set "false" not to load "libgazebo\_ros\_init.so"
    + (default: 'true')
  - 'factory':
    + Set "false" not to load "libgazebo\_ros\_factory.so"
    + (default: 'true')
  - 'force\_system':
    + Set "false" not to load "libgazebo\_ros\_force\_system.so"
    + (default: 'true')
  - 'server\_required':
    + Set "true" to shut down launch script when server is terminated
    + (default: 'false')
  - 'gui\_required':
    + Set "true" to shut down launch script when GUI is terminated
    + (default: 'false')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'x\_pose':
    + Specify the x position of the robot
    + (default: '0.0')
  - 'y\_pose':
    + Specify the y position of the robot
    + (default: '0.0')
  - 'namespace':
    + Specify the namespace of the robot
    + (default: '')

## localization\_launch.py

Launch NAV2 components for robot localization.

* Node
  - nav2\_map\_server.map\_server
  - nav2\_amcl.amcl
  - nav2\_lifecycle\_manager.lifecycle\_manager
* ComposableNode
  - nav2\_map\_server.nav2\_map\_server::MapServer
  - nav2\_amcl.nav2\_amcl::AmclNode
  - nav2\_lifecycle\_manager.nav2\_lifecycle\_manager::LifecycleManager
* Arguments
  - 'namespace':
    + Top-level namespace
    + (default: '')
  - 'map':
    + Full path to map yaml file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/maps/library-new.yaml')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'params\_file':
    + Full path to the ROS2 parameters file to use for all launched nodes
  - 'autostart':
    + Automatically startup the nav2 stack
    + (default: 'True')
  - 'use\_composition':
    + Use composed bringup if True
    + (default: 'False')
  - 'container\_name':
    + the name of conatiner that nodes will load in if use composition
    + (default: 'nav2\_container')
  - 'use\_respawn':
    + Whether to respawn if a node crashes. Applied when composition is disabled.
    + (default: 'False')
  - 'log\_level':
    + log level
    + (default: 'info')

## multi\_tb3\_simulation\_launch.py

Launch multiple turtlebot inside the same simulation,
based on values in robots.yaml.

* Node
  - nav\_system.navigator
  - robot\_writer.robot\_writer
  - robot\_reader.robot\_reader
* Launch
  - launch\_dir.rviz\_launch.py
  - tb3\_simulation\_launch.py
* Arguments
  - 'simulator':
    + The simulator to use (gazebo or gzserver)
    + (default: 'gazebo')
  - 'world':
    + Full path to world file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/worlds/world\_only.model')
  - 'robots':
    + Full path to configuration for spawning multiple robots
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/robots.yaml')
  - 'map':
    + Full path to map file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/maps/library-new.yaml')
  - 'tb3\_template\_params\_file':
    + Full path to the ROS2 parameters file to use for robot1 launched nodes
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/nav2\_multirobot\_params\_template.yaml')
  - 'use\_rviz':
    + Whether to start RVIZ
    + (default: 'True')
  - 'use\_respawn':
    + Whether to restart nav2 crashed processes
    + (default: 'True')
  - 'autostart':
    + Automatically startup the stacks
    + (default: 'false')
  - 'rviz\_config':
    + Full path to the RVIZ config file to use.
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_namespaced\_view.rviz')
  - 'use\_robot\_state\_pub':
    + Whether to start the robot state publisher
    + (default: 'True')
  - 'decrease\_battery':
    + Whether to decrease battery charge level on successful goal
    + (default: 'True')
  - 'namespace':
    + Top-level namespace. The value will be used to replace the <robot\_namespace> keyword on the rviz config file.
    + (default: 'navigation')
  - 'use\_namespace':
    + Whether to apply a namespace to the navigation stack
    + (default: 'false')
  - 'slam':
    + Whether run a SLAM
    + (default: 'False')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'true')
  - 'params\_file':
    + Full path to the ROS2 parameters file to use for all launched nodes
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/nav2\_params.yaml')
  - 'use\_composition':
    + Whether to use composed bringup
    + (default: 'True')
  - 'rviz\_config\_file':
    + Full path to the RVIZ config file to use
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_default\_view.rviz')
  - 'use\_simulator':
    + Whether to start the simulator
    + (default: 'True')
  - 'robot\_name':
    + name of the robot
    + (default: 'turtlebot3\_burger')
  - 'robot\_sdf':
    + Full path to robot sdf file to spawn the robot in gazebo
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/models/turtlebot3\_burger/model.sdf')
  - 'log\_level':
    + log level
    + (default: 'info')
  - 'container\_name':
    + the name of conatiner that nodes will load in if use composition
    + (default: 'nav2\_container')

## robot\_state\_publisher.launch.py

* Node
  - robot\_state\_publisher.robot\_state\_publisher
* Arguments
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')

## spawn\_turtlebot3.launch.py

Launch a turtlebot in the given coordinates.

* Node
  - gazebo\_ros.spawn\_entity.py
* Arguments
  - 'x\_pose':
    + Specify the x position of the robot
    + (default: '0.0')
  - 'y\_pose':
    + Specify the y position of the robot
    + (default: '0.0')
  - 'namespace':
    + Specify the namespace of the robot
    + (default: '')

## tb3\_simulation\_launch.py

Launch a turtlebot inside a Gazebo environment.

* Node
  - robot\_state\_publisher.robot\_state\_publisher
* Launch
  - gazebo\_ros.spawn\_entity.py
  - launch\_dir.rviz\_launch.py
  - turtlebot\_gazebo\_dir.launch.bringup\_launch.py
* Arguments
  - 'namespace':
    + Top-level namespace
    + (default: '')
  - 'use\_namespace':
    + Whether to apply a namespace to the navigation stack
    + (default: 'false')
  - 'slam':
    + Whether run a SLAM
    + (default: 'False')
  - 'map':
    + Full path to map file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/maps/library-new.yaml')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'true')
  - 'params\_file':
    + Full path to the ROS2 parameters file to use for all launched nodes
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/nav2\_params.yaml')
  - 'autostart':
    + Automatically startup the nav2 stack
    + (default: 'true')
  - 'use\_composition':
    + Whether to use composed bringup
    + (default: 'True')
  - 'rviz\_config\_file':
    + Full path to the RVIZ config file to use
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_default\_view.rviz')
  - 'use\_simulator':
    + Whether to start the simulator
    + (default: 'True')
  - 'use\_robot\_state\_pub':
    + Whether to start the robot state publisher
    + (default: 'True')
  - 'use\_rviz':
    + Whether to start RVIZ
    + (default: 'True')
  - 'world':
    + Full path to world model file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/worlds/biblioteca.sdf')
  - 'robot\_name':
    + name of the robot
    + (default: 'turtlebot3\_burger')
  - 'robot\_sdf':
    + Full path to robot sdf file to spawn the robot in gazebo
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/models/turtlebot3\_burger/model.sdf')
  - 'use\_respawn':
    + Whether to respawn if a node crashes. Applied when composition is disabled.
    + (default: 'False')
  - 'rviz\_config':
    + Full path to the RVIZ config file to use
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_default\_view.rviz')
  - 'log\_level':
    + log level
    + (default: 'info')
  - 'container\_name':
    + the name of conatiner that nodes will load in if use composition
    + (default: 'nav2\_container')

## turtlebot3\_library.launch.py

Launch multiple turtlebot inside a library simulation,
in a Gazebo environment.

* Launch
  - pkg\_gazebo\_ros.launch.gzserver.launch.py
  - pkg\_gazebo\_ros.launch.gzclient.launch.py
  - launch\_file\_dir.robot\_state\_publisher.launch.py
  - launch\_file\_dir.multi\_tb3\_simulation\_launch.py
* Arguments
  - 'robots':
    + Full path to configuration for spawning multiple robots
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/robots.yaml')
  - 'world':
    + Full path to world file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/worlds/biblioteca.sdf')
  - 'map':
    + Full path to map file to load
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/maps/library-new.yaml')
  - 'decrease\_battery':
    + Whether to decrease battery on goal success
    + (default: 'True')
  - 'version':
    + Set "true" to output version information.
    + (default: 'false')
  - 'verbose':
    + Set "true" to increase messages written to terminal.
    + (default: 'false')
  - 'lockstep':
    + Set "true" to respect update rates
    + (default: 'false')
  - 'help':
    + Set "true" to produce gzserver help message.
    + (default: 'false')
  - 'pause':
    + Set "true" to start the server in a paused state.
    + (default: 'false')
  - 'physics':
    + Specify a physics engine (ode|bullet|dart|simbody).
    + (default: '')
  - 'play':
    + Play the specified log file.
    + (default: '')
  - 'record':
    + Set "true" to record state data.
    + (default: 'false')
  - 'record\_encoding':
    + Specify compression encoding format for log data (zlib|bz2|txt).
    + (default: '')
  - 'record\_path':
    + Absolute path in which to store state data.
    + (default: '')
  - 'record\_period':
    + Specify recording period (seconds).
    + (default: '')
  - 'record\_filter':
    + Specify recording filter (supports wildcard and regular expression).
    + (default: '')
  - 'seed':
    + Start with a given a random number seed.
    + (default: '')
  - 'iters':
    + Specify number of iterations to simulate.
    + (default: '')
  - 'minimal\_comms':
    + Set "true" to reduce TCP/IP traffic output.
    + (default: 'false')
  - 'profile':
    + Specify physics preset profile name from the options in the world file.
    + (default: '')
  - 'extra\_gazebo\_args':
    + Extra arguments to be passed to Gazebo
    + (default: '')
  - 'params\_file':
    + Path to ROS 2 yaml parameter file
    + (default: '')
  - 'gdb':
    + Set "true" to run gzserver with gdb
    + (default: 'false')
  - 'valgrind':
    + Set "true" to run gzserver with valgrind
    + (default: 'false')
  - 'init':
    + Set "false" not to load "libgazebo\_ros\_init.so"
    + (default: 'true')
  - 'factory':
    + Set "false" not to load "libgazebo\_ros\_factory.so"
    + (default: 'true')
  - 'force\_system':
    + Set "false" not to load "libgazebo\_ros\_force\_system.so"
    + (default: 'true')
  - 'server\_required':
    + Set "true" to shut down launch script when server is terminated
    + (default: 'false')
  - 'gui\_required':
    + Set "true" to shut down launch script when GUI is terminated
    + (default: 'false')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'simulator':
    + The simulator to use (gazebo or gzserver)
    + (default: 'gazebo')
  - 'tb3\_template\_params\_file':
    + Full path to the ROS2 parameters file to use for robot1 launched nodes
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/params/nav2\_multirobot\_params\_template.yaml')
  - 'use\_rviz':
    + Whether to start RVIZ
    + (default: 'True')
  - 'use\_respawn':
    + Whether to restart nav2 crashed processes
    + (default: 'True')
  - 'autostart':
    + Automatically startup the stacks
    + (default: 'false')
  - 'rviz\_config':
    + Full path to the RVIZ config file to use.
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_namespaced\_view.rviz')
  - 'use\_robot\_state\_pub':
    + Whether to start the robot state publisher
    + (default: 'True')
  - 'namespace':
    + Top-level namespace. The value will be used to replace the <robot\_namespace> keyword on the rviz config file.
    + (default: 'navigation')
  - 'use\_namespace':
    + Whether to apply a namespace to the navigation stack
    + (default: 'false')
  - 'slam':
    + Whether run a SLAM
    + (default: 'False')
  - 'use\_composition':
    + Whether to use composed bringup
    + (default: 'True')
  - 'rviz\_config\_file':
    + Full path to the RVIZ config file to use
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/nav2\_bringup/share/nav2\_bringup/rviz/nav2\_default\_view.rviz')
  - 'use\_simulator':
    + Whether to start the simulator
    + (default: 'True')
  - 'robot\_name':
    + name of the robot
    + (default: 'turtlebot3\_burger')
  - 'robot\_sdf':
    + Full path to robot sdf file to spawn the robot in gazebo
    + (default: '/home/ws/NODES/GAZEBO\_Turtlebot3/install/turtlebot3\_gazebo/share/turtlebot3\_gazebo/models/turtlebot3\_burger/model.sdf')
  - 'log\_level':
    + log level
    + (default: 'info')
  - 'container\_name':
    + the name of conatiner that nodes will load in if use composition
    + (default: 'nav2\_container')

## turtlebot3\_slam\_library.launch.py

Launch a turtlebot to perform a SLAM mapping of the library world.

* Launch
  - pkg\_gazebo\_ros.launch.gzserver.launch.py
  - pkg\_gazebo\_ros.launch.gzclient.launch.py
  - launch\_file\_dir.robot\_state\_publisher.launch.py
  - launch\_file\_dir.spawn\_turtlebot3.launch.py
* Arguments
  - 'world':
    + Specify world file name
    + (default: '')
  - 'version':
    + Set "true" to output version information.
    + (default: 'false')
  - 'verbose':
    + Set "true" to increase messages written to terminal.
    + (default: 'false')
  - 'lockstep':
    + Set "true" to respect update rates
    + (default: 'false')
  - 'help':
    + Set "true" to produce gzserver help message.
    + (default: 'false')
  - 'pause':
    + Set "true" to start the server in a paused state.
    + (default: 'false')
  - 'physics':
    + Specify a physics engine (ode|bullet|dart|simbody).
    + (default: '')
  - 'play':
    + Play the specified log file.
    + (default: '')
  - 'record':
    + Set "true" to record state data.
    + (default: 'false')
  - 'record\_encoding':
    + Specify compression encoding format for log data (zlib|bz2|txt).
    + (default: '')
  - 'record\_path':
    + Absolute path in which to store state data.
    + (default: '')
  - 'record\_period':
    + Specify recording period (seconds).
    + (default: '')
  - 'record\_filter':
    + Specify recording filter (supports wildcard and regular expression).
    + (default: '')
  - 'seed':
    + Start with a given a random number seed.
    + (default: '')
  - 'iters':
    + Specify number of iterations to simulate.
    + (default: '')
  - 'minimal\_comms':
    + Set "true" to reduce TCP/IP traffic output.
    + (default: 'false')
  - 'profile':
    + Specify physics preset profile name from the options in the world file.
    + (default: '')
  - 'extra\_gazebo\_args':
    + Extra arguments to be passed to Gazebo
    + (default: '')
  - 'params\_file':
    + Path to ROS 2 yaml parameter file
    + (default: '')
  - 'gdb':
    + Set "true" to run gzserver with gdb
    + (default: 'false')
  - 'valgrind':
    + Set "true" to run gzserver with valgrind
    + (default: 'false')
  - 'init':
    + Set "false" not to load "libgazebo\_ros\_init.so"
    + (default: 'true')
  - 'factory':
    + Set "false" not to load "libgazebo\_ros\_factory.so"
    + (default: 'true')
  - 'force\_system':
    + Set "false" not to load "libgazebo\_ros\_force\_system.so"
    + (default: 'true')
  - 'server\_required':
    + Set "true" to shut down launch script when server is terminated
    + (default: 'false')
  - 'gui\_required':
    + Set "true" to shut down launch script when GUI is terminated
    + (default: 'false')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'x\_pose':
    + Specify the x position of the robot
    + (default: '0.0')
  - 'y\_pose':
    + Specify the y position of the robot
    + (default: '0.0')
  - 'namespace':
    + Specify the namespace of the robot
    + (default: '')

## turtlebot3\_world.launch.py

* Arguments
  - 'world':
    + Specify world file name
    + (default: '')
  - 'version':
    + Set "true" to output version information.
    + (default: 'false')
  - 'verbose':
    + Set "true" to increase messages written to terminal.
    + (default: 'false')
  - 'lockstep':
    + Set "true" to respect update rates
    + (default: 'false')
  - 'help':
    + Set "true" to produce gzserver help message.
    + (default: 'false')
  - 'pause':
    + Set "true" to start the server in a paused state.
    + (default: 'false')
  - 'physics':
    + Specify a physics engine (ode|bullet|dart|simbody).
    + (default: '')
  - 'play':
    + Play the specified log file.
    + (default: '')
  - 'record':
    + Set "true" to record state data.
    + (default: 'false')
  - 'record\_encoding':
    + Specify compression encoding format for log data (zlib|bz2|txt).
    + (default: '')
  - 'record\_path':
    + Absolute path in which to store state data.
    + (default: '')
  - 'record\_period':
    + Specify recording period (seconds).
    + (default: '')
  - 'record\_filter':
    + Specify recording filter (supports wildcard and regular expression).
    + (default: '')
  - 'seed':
    + Start with a given a random number seed.
    + (default: '')
  - 'iters':
    + Specify number of iterations to simulate.
    + (default: '')
  - 'minimal\_comms':
    + Set "true" to reduce TCP/IP traffic output.
    + (default: 'false')
  - 'profile':
    + Specify physics preset profile name from the options in the world file.
    + (default: '')
  - 'extra\_gazebo\_args':
    + Extra arguments to be passed to Gazebo
    + (default: '')
  - 'params\_file':
    + Path to ROS 2 yaml parameter file
    + (default: '')
  - 'gdb':
    + Set "true" to run gzserver with gdb
    + (default: 'false')
  - 'valgrind':
    + Set "true" to run gzserver with valgrind
    + (default: 'false')
  - 'init':
    + Set "false" not to load "libgazebo\_ros\_init.so"
    + (default: 'true')
  - 'factory':
    + Set "false" not to load "libgazebo\_ros\_factory.so"
    + (default: 'true')
  - 'force\_system':
    + Set "false" not to load "libgazebo\_ros\_force\_system.so"
    + (default: 'true')
  - 'server\_required':
    + Set "true" to shut down launch script when server is terminated
    + (default: 'false')
  - 'gui\_required':
    + Set "true" to shut down launch script when GUI is terminated
    + (default: 'false')
  - 'use\_sim\_time':
    + Use simulation (Gazebo) clock if true
    + (default: 'false')
  - 'x\_pose':
    + Specify the x position of the robot
    + (default: '0.0')
  - 'y\_pose':
    + Specify the y position of the robot
    + (default: '0.0')
  - 'namespace':
    + Specify the namespace of the robot
    + (default: '')
