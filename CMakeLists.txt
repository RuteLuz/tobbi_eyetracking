cmake_minimum_required(VERSION 2.8.3)
project(tobii_eyetracking)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
)

## Build
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(eyetracking ros/src/eyetracking.cpp)
target_link_libraries(eyetracking ${catkin_LIBRARIES})
target_link_libraries(eyetracking /usr/lib/tobii/libtobii_stream_engine.so)
target_link_libraries(eyetracking pthread)
