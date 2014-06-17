cmake_minimum_required(VERSION 2.8.3)
project(gmm_classify)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  CATKIN_DEPENDS opencv2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gmm-classify src/gmm_classify.cc)
target_link_libraries(gmm-classify ${catkin_LIBRARIES})


