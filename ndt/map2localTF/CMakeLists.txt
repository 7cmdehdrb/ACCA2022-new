cmake_minimum_required(VERSION 2.8.3)
project(map2localTF)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  eigen_conversions
  std_msgs
  tf
  tf_conversions
  tf2_ros
  tf2
  autoware_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  sensor_msgs
  eigen_conversions
  std_msgs
  tf
  tf_conversions
  tf2_ros
  tf2
  autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(map2localTF src/map2localTF.cpp)
target_link_libraries(map2localTF ${catkin_LIBRARIES})
add_dependencies(map2localTF ${catkin_EXPORTED_TARGETS})



install(
  TARGETS
    map2localTF
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)