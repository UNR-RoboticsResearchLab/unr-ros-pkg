cmake_minimum_required(VERSION 2.8.3)
project(stage_adapt)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS tf roscpp)

catkin_package(
  CATKIN_DEPENDS tf roscpp
)
include_directories(
 ${catkin_INCLUDE_DIRS} 
)

add_executable(stage-adapt src/stage-adapt.cc)
target_link_libraries(stage-adapt
  ${catkin_LIBRARIES}
)
