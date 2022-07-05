cmake_minimum_required(VERSION 2.4.6)
project(map_matching_localization)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   message_generation
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


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
