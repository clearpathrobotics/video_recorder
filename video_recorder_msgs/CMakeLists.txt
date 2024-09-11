cmake_minimum_required(VERSION 3.8)
project(video_recorder_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 17)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/Status.msg
        action/StartRecording.action
        action/SaveImage.action
        action/StopRecording.action
    DEPENDENCIES
      builtin_interfaces
      std_msgs
)

install(FILES
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(
  builtin_interfaces
  std_msgs
)
ament_package()
