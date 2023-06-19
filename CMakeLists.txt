cmake_minimum_required(VERSION 3.5)
project(camera_waypoints_trigger)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  sensor_msgs
  cv_bridge
  px4_msgs
)

add_executable(camera_screenshots_handler src/camera_screenshots_handler.cpp)
ament_target_dependencies(camera_screenshots_handler ${dependencies})


install(TARGETS
  camera_screenshots_handler

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
