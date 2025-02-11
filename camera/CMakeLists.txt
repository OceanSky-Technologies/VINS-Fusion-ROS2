cmake_minimum_required(VERSION 3.8)
project(camera_publisher)

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# GStreamer
# We'll use CMake's pkg-config to find GStreamer libraries.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PUBLIC
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
)
target_link_libraries(camera_node
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()