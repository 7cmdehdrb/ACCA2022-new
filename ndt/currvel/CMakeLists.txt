cmake_minimum_required(VERSION 2.8.3)
project(currvel)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  erp42_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  erp42_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(currvel src/currvel.cpp)
target_link_libraries(currvel ${catkin_LIBRARIES})
add_dependencies(currvel ${catkin_EXPORTED_TARGETS})



install(
  TARGETS
    currvel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)