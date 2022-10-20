## How a custom plugin works

- a gazebo plugin is basically a `.so` file (complied from a c++ script) that is included in the robot model's sdf, say `model.sdf`
- This c++ file, lets name it `gazebo_plugin.cpp`, should contain the code as to what the plugin does. For example, if the plugin is meant to 
provide a way for the developed ROS nodes to communicate with gazebo for controlling the joints of a robot, or adding a force the to robot, then all this
fuctionality is supposed to be coded in the `gazebo_plugin.cpp` file. 
- gazebo provides a way to do this using an API. This API is is basically a bunch of header files and libraries that come installed with Gazebo. They can be found
in `/urs/include/gazebo-9/gazebo` directory. Notably, the `gazebo_plugin.so` file generally needs to subscribe to topics so it can recieve messages from nodes
and can aslo be used to publish topics regarding the robot state, say like it's position, velocity etc...
- The API files provides a bunch of classes and methods than can be used to affect changes in gazebo, like control joints and add forces. These is documentation available
for the same below.
- [gazebo API documentation](http://osrf-distributions.s3.amazonaws.com/gazebo/api/2.2.1/index.html)
- so the general idea is this compiled file (`gazebo_plugin.so`) is loaded into gazebo as gazebo is launched. It makes gazebo subscribe to topics so it can recieve
messages from the nodes that publish in that topic. It makes gazebo publish topoics on various aspects of the robot state or the simulation state like sensor inputs,
camera view, etc.. things of that sort.
