cmake_minimum_required(VERSION 2.4.6)
project(map_matching_localization)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   sensor_msgs
   message_generation
   eigen_conversions
   tf
   tf_conversions
   tf2_ros
   tf2
)

add_message_files(
  FILES
  Error.msg
  #  HdlTF.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msgs
   CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)

add_executable(map_odom_tf_publisher src/map_odom_tf_publisher.cpp)
target_link_libraries(map_odom_tf_publisher ${catkin_LIBRARIES})
add_dependencies(map_odom_tf_publisher ${catkin_EXPORTED_TARGETS})



install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
