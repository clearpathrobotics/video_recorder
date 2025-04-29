cmake_minimum_required(VERSION 3.8)
project(video_recorder)

find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(video_recorder_msgs REQUIRED)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(
  video_recorder_node
    src/video_recorder_node.cpp
)

ament_target_dependencies(
  video_recorder_node
    OpenCV
    "cv_bridge"
    "geometry_msgs"
    "image_transport"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "sensor_msgs"
    "std_msgs"
    "tf2"
    "tf2_ros"
    "video_recorder_msgs"
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  video_recorder_msgs
)

ament_package()