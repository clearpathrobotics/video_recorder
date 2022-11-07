cmake_minimum_required(VERSION 3.0.2)
project(video_recorder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  tf
  tf2
  actionlib
  actionlib_msgs
  geometry_msgs
  video_recorder_msgs
)
find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS actionlib actionlib_msgs video_recorder_msgs)

add_executable(video_recorder_node src/video_recorder_node.cpp)
target_link_libraries(video_recorder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(video_recorder_node
    video_recorder_msgs_generate_messages_cpp
    geometry_msgs_generate_messages_cpp
    sensor_msgs_generate_messages_cpp
    std_msgs_generate_messages_cpp
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
