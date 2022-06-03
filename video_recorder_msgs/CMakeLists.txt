cmake_minimum_required(VERSION 3.0.2)
project(video_recorder_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_service_files(
  FILES
  SaveImage.srv
  StartRecording.srv
  StopRecording.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)
