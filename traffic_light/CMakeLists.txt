cmake_minimum_required(VERSION 3.8)
project(traffic_light)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(traffic_light_msgs REQUIRED)

#
# Component library
#

add_library(traffic_light_component
  SHARED
    src/traffic_light/traffic_light_component.cpp
)

target_compile_features(traffic_light_component PUBLIC cxx_std_17)

target_include_directories(traffic_light_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(traffic_light_component
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  traffic_light_msgs
)

target_link_libraries(traffic_light_component
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_nodes(traffic_light_component
  "traffic_light::TrafficLightComponent"
)

#
# Standalone executable
#

add_executable(traffic_light_node
  src/traffic_light_node.cpp
)

target_compile_features(traffic_light_node PUBLIC cxx_std_17)

target_link_libraries(traffic_light_node
  traffic_light_component
)

ament_target_dependencies(traffic_light_node
  rclcpp
)

#
# Installation
#

install(
  TARGETS traffic_light_component
  EXPORT export_traffic_light
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS traffic_light_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  FILES
    README.md
    LICENSE
  DESTINATION
    share/${PROJECT_NAME}
)

ament_export_targets(
  export_traffic_light
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  OpenCV
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  traffic_light_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
