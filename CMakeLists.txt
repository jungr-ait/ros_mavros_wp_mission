cmake_minimum_required(VERSION 2.8.3)
set(target ros_mavros_wp_mission)
project(${target})

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs mavros)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${target} src/main.cpp)
target_link_libraries(${target} ${catkin_LIBRARIES})

install(
  TARGETS ${target}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
