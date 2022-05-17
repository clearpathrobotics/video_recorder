cmake_minimum_required(VERSION 3.0.2)
project(video_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  std_msgs
  cv_bridge
  roscpp
  image_transport
)
find_package(OpenCV 3 REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_service_files(
  FILES
  SaveImage.srv
  StartRecording.srv
  StopRecording.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package()

add_executable(video_recorder_node src/video_recorder_node.cpp)
target_link_libraries(video_recorder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(video_recorder_node sensor_msgs std_msgs)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
