cmake_minimum_required(VERSION 3.5)

project(vision_to_mavros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(tf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)

# set(msg_files
#     msg/BatteryEstimate.msg
#     msg/DronePosition.msg
#     msg/DroneState.msg
#     msg/ErrorState.msg
#     msg/Mission.msg

# )
# set(srv_files
#     srv/ArmDrone.srv
#     srv/ContinueMission.srv
#     srv/FlyToPosition.srv
#     srv/FlyToWaypoints.srv
#     srv/GetGeoFence.srv
#     srv/PauseMission.srv
#     srv/PrepareDrone.srv
#     srv/ReturnToBase.srv
#     srv/SetBasePosition.srv
#     srv/SetGeoFence.srv
#     srv/SetROI.srv
#     srv/SetServo.srv
#     srv/SetWaypoints.srv
#     srv/StopAndDrop.srv
#     srv/StopAndHover.srv
#     srv/StopAndLand.srv
#     srv/SwitchHeadlights.srv
#     srv/Takeoff.srv
# )
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
#   ADD_LINTER_TESTS
# )

add_executable(vision_to_mavros src/vision_to_mavros.cpp)
ament_target_dependencies(vision_to_mavros 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  # tf
  mavros_msgs
)

install(TARGETS
  vision_to_mavros
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()