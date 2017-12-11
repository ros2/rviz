cmake_minimum_required(VERSION 3.5)

project(rviz_default_plugins)

# TODO(wjwood): remove when windows build is fixed
if(DEFINED ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE})
  set(RVIZ_BUILD_ON_WINDOWS_OVERRIDE "$ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE}")
endif()
if(WIN32 AND NOT DEFINED RVIZ_BUILD_ON_WINDOWS_OVERRIDE)
  message(STATUS "${PROJECT_NAME} is not supported on Windows, skipping...")
  return()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_default_plugins_headers_to_moc
  src/rviz_default_plugins/displays/grid/grid_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp
)

set(rviz_default_plugins_source_files
  src/rviz_default_plugins/displays/grid/grid_display.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/axis_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/flat_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/intensity_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgb8_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgbf32_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/xyz_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_display.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.cpp
  src/rviz_default_plugins/tools/move/move_tool.cpp
  src/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.cpp
)

add_library(rviz_default_plugins SHARED
  ${rviz_default_plugins_headers_to_moc}
  ${rviz_default_plugins_source_files}
)
target_include_directories(rviz_default_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include>
)
target_link_libraries(rviz_default_plugins
  rviz_rendering::rviz_rendering
  rviz_common::rviz_common
)
ament_target_dependencies(rviz_default_plugins
  # geometry_msgs
  rclcpp
  # tf2
  # tf2_geometry_msgs
  # tf2_ros
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_interfaces(rviz_default_plugins)
ament_export_dependencies(Qt5)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(
  # geometry_msgs
  rclcpp
  # tf2
  # tf2_geometry_msgs
  # tf2_ros
)

install(
  TARGETS rviz_default_plugins
  EXPORT rviz_default_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# TODO(wjwwood): reenable tests for Windows after resolving building problems
if(BUILD_TESTING AND NOT WIN32)
  # TODO(wjwwood): replace this with ament_lint_auto() and/or add the copyright linter back
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(point_cloud2_display_test
      test/rviz_default_plugins/displays/pointcloud/message_creators.hpp
      test/rviz_default_plugins/displays/pointcloud/message_creators.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud2_display_test.cpp)
  if(TARGET point_cloud2_display_test)
    target_link_libraries(point_cloud2_display_test rviz_default_plugins)
  endif()

  ament_add_gtest(point_cloud_transformers_test
      test/rviz_default_plugins/displays/pointcloud/message_creators.hpp
      test/rviz_default_plugins/displays/pointcloud/message_creators.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/axis_color_pc_transformer_test.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/flat_color_pc_transformer_test.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/intensity_pc_transformer_test.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgb8_pc_transformer_test.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgbf32_pc_transformer_test.cpp
      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/xyz_pc_transformer_test.cpp)
  if(TARGET point_cloud_transformers_test)
    target_link_libraries(point_cloud_transformers_test rviz_default_plugins)
  endif()
endif()

ament_package()
