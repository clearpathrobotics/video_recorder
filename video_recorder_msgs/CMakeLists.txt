cmake_minimum_required(VERSION 3.0.2)
project(video_recorder_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  std_msgs
)

add_action_files(
  FILES
  SaveImage.action
  StartRecording.action
  StopRecording.action
)

add_message_files(FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs std_msgs genmsg)
