# ros_mavros_wp_mission

similar to: https://github.com/erlerobot/ros_erle_takeoff_land

Demonstrates how mavros can be used to set waypoint and how to execute them. 
IMHO there is no full demo code that covers a mission, that's why this repo was created.
This was tested with JMAVSim + PX4 + QGroundControl + ROS kinetic.
After installation of JMAVSim the there should be a folder `~/src/Firmware`. Go to that direction and type:

```
make posix_sitl_default jmavsim
```

In an other terminal:
```
roscore
```

In an other terminal:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

In an other start qgroundcontrol:
```
./qgroundcontrol-start.sh 
```

Finally launch this node (you need to build that package first and source/pollute the ROS environment with the new definition):
```
roslaunch ros_mavros_wp_mission default.launch
```
