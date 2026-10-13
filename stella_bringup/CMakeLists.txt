################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(stella_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  add_library(stella_dds_udp_guard SHARED src/dds_udp_guard.cpp src/dds_log_throttle.cpp)
  target_link_libraries(stella_dds_udp_guard dl)
  install(TARGETS stella_dds_udp_guard LIBRARY DESTINATION lib)
  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_library(dds_transport_stub SHARED test/dds_transport_stub.cpp)
    set_target_properties(dds_transport_stub PROPERTIES OUTPUT_NAME "ddsc.so.guard_test")
    add_library(sendmsg_stub SHARED test/sendmsg_stub.cpp)
    ament_add_gtest(test_dds_udp_guard test/test_dds_udp_guard.cpp
      APPEND_ENV "LD_PRELOAD=$<TARGET_FILE:stella_dds_udp_guard>")
    target_link_libraries(test_dds_udp_guard dds_transport_stub sendmsg_stub)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    ament_add_test(test_python_dds_initialization
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_dds_initialization.py"
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      ENV
        "LD_PRELOAD=$<TARGET_FILE:stella_dds_udp_guard>"
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        "ROS_DOMAIN_ID=124"
        "CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>"
      TIMEOUT 15)
  endif()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/measure_stability.py DESTINATION lib/${PROJECT_NAME})

################################################################################
# Macro for ament package
################################################################################
ament_package()
