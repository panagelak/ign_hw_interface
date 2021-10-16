cmake_minimum_required(VERSION 2.8.3)
project(ignition_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  rosparam_shortcuts
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ign_hardware_interface
  CATKIN_DEPENDS 
    roscpp
    hardware_interface
    controller_manager
    rosparam_shortcuts
    tf2_ros
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS 
    Boost
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

# Control loop
add_library(ign_hardware_control_loop src/hw_control_loop.cpp)
target_link_libraries(ign_hardware_control_loop
  ${catkin_LIBRARIES}
)

# Ignition Hardware Interface
add_library(ign_hardware_interface
  src/ign_hw_interface.cpp
)
add_dependencies(ign_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ign_hardware_interface  
  ${catkin_LIBRARIES}
)

# Ignition Hardware Interface node
add_executable(ign_hardware_interface_node src/ign_hw_interface_node.cpp)
add_dependencies(ign_hardware_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ign_hardware_interface_node
  ign_hardware_interface
  ign_hardware_control_loop
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ign_hardware_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ign_hardware_interface ign_hardware_control_loop 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
