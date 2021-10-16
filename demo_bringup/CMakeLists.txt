cmake_minimum_required(VERSION 2.8.3)
project(demo_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  actionlib
  control_msgs
  ros_ign_bridge
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_pub src/test_pub.cpp)
add_dependencies(test_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pub
  ${catkin_LIBRARIES}
)

add_executable(test_action_client src/test_action_client.cpp)
add_dependencies(test_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_action_client
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS test_pub test_action_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY 
    launch controller rviz worlds doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)