cmake_minimum_required(VERSION 3.0.2)
project(video_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  roscpp
  image_transport
  video_recorder_msgs
)
find_package(OpenCV 4 REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(video_recorder_node src/video_recorder_node.cpp)
target_link_libraries(video_recorder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video_recorder_node video_recorder_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})