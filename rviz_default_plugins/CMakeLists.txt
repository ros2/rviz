cmake_minimum_required(VERSION 3.5)

project(rviz_default_plugins)

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

# find_package(Qt5 REQUIRED COMPONENTS Widgets)

# find_package(geometry_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_default_plugins_headers_to_moc
)

set(rviz_default_plugins_source_files
  src/rviz_default_plugins/tools/move_tool.cpp
  src/rviz_default_plugins/view_controllers/orbit_view_controller.cpp
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
# ament_target_dependencies(rviz_default_plugins geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_interfaces(rviz_default_plugins)
ament_export_dependencies(Qt5)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)

install(
  TARGETS rviz_default_plugins
  EXPORT rviz_default_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# if(BUILD_TESTING)
#   # TODO(wjwwood): replace this with ament_lint_auto() and/or add the copyright linter back
#   find_package(ament_cmake_cppcheck REQUIRED)
#   find_package(ament_cmake_cpplint REQUIRED)
#   find_package(ament_cmake_lint_cmake REQUIRED)
#   find_package(ament_cmake_uncrustify REQUIRED)
#   ament_cppcheck()
#   ament_cpplint()
#   ament_lint_cmake()
#   ament_uncrustify()

#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(rviz_common_config_test
#     test/config_test.cpp)
#   if(TARGET rviz_common_config_test)
#     target_link_libraries(rviz_common_config_test rviz_common)
#   endif()

#   ament_add_gtest(rviz_common_uniform_string_stream_test
#     test/uniform_string_stream_test.cpp)
#   if(TARGET rviz_common_uniform_string_stream_test)
#     target_link_libraries(rviz_common_uniform_string_stream_test rviz_common)
#   endif()

#   ament_add_gtest(rviz_common_display_test
#     test/display_test.cpp
#     test/mock_display.cpp
#     test/mock_display_group.cpp)
#   if(TARGET rviz_common_display_test)
#     target_include_directories(rviz_common_display_test PUBLIC src)
#     target_link_libraries(rviz_common_display_test rviz_common)
#   endif()

#   ament_add_gtest(rviz_common_property_test
#     test/property_test.cpp
#     test/mock_property_change_receiver.cpp)
#   if(TARGET rviz_common_property_test)
#     target_link_libraries(rviz_common_property_test rviz_common)
#   endif()
# endif()

ament_package()
