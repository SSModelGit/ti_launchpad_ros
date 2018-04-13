cmake_minimum_required(VERSION 2.8.3)
project(txi_launchpad)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  LIBRARIES txi_launchpad
  CATKIN_DEPENDS joy roscpp sensor_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cc3200_joy src/cc3200_joy.cpp)
target_link_libraries(cc3200_joy ${catkin_LIBRARIES})
# add_dependencies(cc3200_joy txi_launchpad_generate_messages_cpp)

