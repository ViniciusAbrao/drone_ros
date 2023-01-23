cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  mavros_msgs
  message_generation
)

add_message_files(
  FILES
  Corner.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
