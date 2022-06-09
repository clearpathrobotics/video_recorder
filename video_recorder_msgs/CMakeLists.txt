cmake_minimum_required(VERSION 3.0.2)
project(video_recorder_msgs)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
)

add_action_files(
  FILES
  SaveImage.action
  StartRecording.action
  StopRecording.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs genmsg)
