cmake_minimum_required(VERSION 2.8.3)
project(puzzlebot_control)

add_compile_options(-std=c++11)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs geometry_msgs sensor_msgs message_generation tf)

find_package(OpenCV REQUIRED)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS  std_msgs nav_msgs geometry_msgs sensor_msgs roscpp message_runtime)

include_directories(include 
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS})
