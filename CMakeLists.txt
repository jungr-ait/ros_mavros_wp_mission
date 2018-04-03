cmake_minimum_required(VERSION 2.8.3)
project(ros_mavros_wp_mission)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs mavros)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

set(target global_pos_mission)
add_executable(${target} src/global_pos_mission.cpp)
target_link_libraries(${target} ${catkin_LIBRARIES})

install(
  TARGETS ${target}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


set(target local_pos_mission)
add_executable(${target} src/local_pos_mission.cpp)
target_link_libraries(${target} ${catkin_LIBRARIES})

install(
  TARGETS ${target}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
