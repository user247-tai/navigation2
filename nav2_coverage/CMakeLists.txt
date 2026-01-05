cmake_minimum_required(VERSION 3.5)
project(nav2_coverage)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

nav2_package()

set(executable_name coverage_server)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/coverage_server.cpp
  src/poses_creator.cpp
)

target_include_directories(${library_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component

  nav2_util::nav2_util_core
  nav2_costmap_2d::nav2_costmap_2d_core

  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(${executable_name}
  src/main.cpp
)

target_include_directories(${executable_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

rclcpp_components_register_nodes(${library_name} "nav2_coverage::CoverageServer")

install(TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav2_util
  nav2_msgs
  geometry_msgs
  nav_msgs
  nav2_costmap_2d
)
ament_export_targets(${library_name})

ament_package()
