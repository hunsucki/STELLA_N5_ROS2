cmake_minimum_required(VERSION 3.8)
project(inspection_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CaptureZone.msg"
  "srv/AbortCaptureRun.srv"
  "srv/CapturePair.srv"
  "srv/FinishCaptureRun.srv"
  "srv/StartCaptureRun.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

install(FILES
  CAMERA_SERVICE_INTEGRATION.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
