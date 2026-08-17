cmake_minimum_required(VERSION 3.8)
project(udl_aa_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Telemetry - components first, then the snapshot that composes them.
  "msg/FcStatus.msg"
  "msg/GlobalPosition.msg"
  "msg/LocalPosition.msg"
  "msg/BodyVelocity.msg"
  "msg/EnuVelocity.msg"
  "msg/ImuData.msg"
  "msg/Battery.msg"
  "msg/GpsStatus.msg"
  "msg/VfrHud.msg"
  "msg/RcInput.msg"
  "msg/StatusText.msg"
  "msg/VehicleTelemetry.msg"

  # Command chain, fs_adapter <-> flight commander.
  "msg/VehicleCommand.msg"
  "msg/VehicleStatus.msg"

  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
