cmake_minimum_required(VERSION 2.8.3)
project(blinky_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  ft_scservo_driver
  hardware_interface
  joint_limits_interface
  roscpp
  roslaunch
  transmission_interface
  vesc_driver
)

find_package(Boost REQUIRED COMPONENTS thread chrono)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES blinky_hardware
  CATKIN_DEPENDS controller_manager ft_scservo_driver hardware_interface joint_limits_interface roscpp transmission_interface vesc_driver
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(blinky_hardware
  src/blinky_hardware.cpp
)

add_executable(blinky_hardware_node src/blinky_hardware_node.cpp)

target_link_libraries(blinky_hardware
  ${catkin_LIBRARIES}
)

target_link_libraries(blinky_hardware_node
  blinky_hardware
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#############
## Install ##
#############

install(TARGETS blinky_hardware blinky_hardware_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
roslaunch_add_file_check(launch/base.launch)

