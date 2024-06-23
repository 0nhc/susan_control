cmake_minimum_required(VERSION 3.0.2)
project(susan_control)

find_package(catkin 
  REQUIRED
  COMPONENTS
  roscpp
  can_msgs
  std_msgs
  controller_manager
  socketcan_bridge
  joint_trajectory_controller
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lk_ros
  CATKIN_DEPENDS 
  roscpp
  can_msgs
  std_msgs
  controller_manager
  socketcan_bridge
  joint_trajectory_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ME8010E17B50 src/motor_protocols/ME8010E17B50.cpp)
add_library(MG4005I10 src/motor_protocols/MG4005I10.cpp)
add_library(MG4010I36 src/motor_protocols/MG4010I36.cpp)
add_library(MG6012I36 src/motor_protocols/MG6012I36.cpp)
add_library(DM4340 src/motor_protocols/DM4340.cpp)
add_library(utils src/utils/utils.cpp)
add_library(pid src/utils/pid.cpp)
add_library(susan_hardware_interface src/ros_interface/susan_hardware_interface.cpp)
add_library(calibration src/ros_interface/calibration.cpp)

target_link_libraries(susan_hardware_interface MG6012I36 DM4340 utils ${catkin_LIBRARIES})
add_executable(susan_hardware_interface_node src/ros_nodes/susan_hardware_interface_node.cpp)
target_link_libraries(susan_hardware_interface_node MG6012I36 DM4340 utils susan_hardware_interface ${catkin_LIBRARIES})

target_link_libraries(calibration MG6012I36 DM4340 utils ${catkin_LIBRARIES})
add_executable(calibration_node src/ros_nodes/calibration_node.cpp)
target_link_libraries(calibration_node MG6012I36 DM4340 utils calibration ${catkin_LIBRARIES})

install(TARGETS 
  ME8010E17B50
  MG4005I10
  MG4010I36
  susan_hardware_interface
  calibration
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
