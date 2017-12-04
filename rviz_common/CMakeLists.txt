cmake_minimum_required(VERSION 3.5)

project(rviz_common)

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

# TODO(wjwwood): figure out why autogen fails on Windows, and why it isn't necessary...
if(NOT WIN32)
  # Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  set(CMAKE_AUTOMOC ON)
endif()

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

find_package(TinyXML REQUIRED)  # provided by tinyxml_vendor

# Copy env_config.hpp so that env_config.cpp can find it
# TODO(jsquare): Get rid of copy hpp file
set(ENV_CONFIG_HPP ${CMAKE_CURRENT_BINARY_DIR}/src/rviz_common/env_config.hpp)
configure_file(src/rviz_common/env_config.hpp ${ENV_CONFIG_HPP} @ONLY)

# TODO(jsquare): Adopt parameters given here
set(ENV_CONFIG_CPP ${CMAKE_CURRENT_BINARY_DIR}/src/rviz_common/env_config.cpp)
set(RVIZ_VERSION "Alpha")
set(ROS_DISTRO "ROS 2.0")
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")
configure_file(src/rviz_common/env_config.cpp.in ${ENV_CONFIG_CPP} @ONLY)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_common_headers_to_moc
  ${ENV_CONFIG_HPP}
  src/rviz_common/add_display_dialog.hpp
  # src/rviz_common/help_panel.hpp
  include/rviz_common/properties/enum_property.hpp
  include/rviz_common/properties/property_tree_widget.hpp
  include/rviz_common/properties/splitter_handle.hpp
  # src/rviz_common/selection_panel.hpp
  include/rviz_common/frame_position_tracking_view_controller.hpp
  include/rviz_common/properties/quaternion_property.hpp
  include/rviz_common/visualizer_app.hpp
  include/rviz_common/display.hpp
  include/rviz_common/display_context.hpp
  src/rviz_common/displays_panel.hpp
  src/rviz_common/failed_panel.hpp
  include/rviz_common/frame_manager.hpp
  src/rviz_common/loading_dialog.hpp
  src/rviz_common/new_object_dialog.hpp
  src/rviz_common/panel.hpp
  src/rviz_common/panel_dock_widget.hpp
  include/rviz_common/properties/bool_property.hpp
  include/rviz_common/properties/color_editor.hpp
  include/rviz_common/properties/color_property.hpp
  include/rviz_common/properties/combo_box.hpp
  include/rviz_common/properties/editable_combo_box.hpp
  include/rviz_common/properties/editable_enum_property.hpp
  include/rviz_common/properties/float_edit.hpp
  include/rviz_common/properties/float_property.hpp
  include/rviz_common/properties/int_property.hpp
  include/rviz_common/properties/line_edit_with_button.hpp
  include/rviz_common/properties/property.hpp
  include/rviz_common/properties/property_tree_delegate.hpp
  include/rviz_common/properties/property_tree_model.hpp
  include/rviz_common/properties/property_tree_with_help.hpp
  include/rviz_common/properties/status_list.hpp
  include/rviz_common/properties/status_property.hpp
  include/rviz_common/properties/string_property.hpp
  include/rviz_common/properties/tf_frame_property.hpp
  include/rviz_common/properties/vector_property.hpp
  include/rviz_common/render_panel.hpp
  src/rviz_common/scaled_image_widget.hpp
  src/rviz_common/screenshot_dialog.hpp
  include/rviz_common/selection/selection_manager.hpp
  src/rviz_common/splash_screen.hpp
  # src/rviz_common/time_panel.hpp
  include/rviz_common/tool.hpp
  src/rviz_common/tool_manager.hpp
  # src/rviz_common/tool_properties_panel.hpp
  include/rviz_common/view_controller.hpp
  include/rviz_common/view_manager.hpp
  src/rviz_common/views_panel.hpp
  src/rviz_common/visualization_frame.hpp
  src/rviz_common/visualization_manager.hpp
  src/rviz_common/widget_geometry_change_detector.hpp
  src/rviz_common/window_manager_interface.hpp
  include/rviz_common/validate_floats.hpp
  include/rviz_common/message_filter_display.hpp
)

# TODO(wjwwood): remove this later
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/default_plugins/displays/grid_display.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/default_plugins/displays/tf_display.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/default_plugins/displays/robot_model_display.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot_link.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot_joint.hpp)

# TODO(greimela): Remove when moving the default plugins to separate package
list(APPEND rviz_common_headers_to_moc
  src/rviz_default_plugins/point_cloud_common.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_default_plugins/point_cloud_display.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_default_plugins/point_cloud_transformer.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_default_plugins/point_cloud_helpers.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_default_plugins/point_cloud2_display.hpp)

set(rviz_common_source_files
  ${ENV_CONFIG_CPP}
  src/rviz_common/add_display_dialog.cpp
  src/rviz_common/bit_allocator.cpp
  src/rviz_common/config.cpp
  src/rviz_common/display_factory.cpp
  src/rviz_common/display_group.cpp
  src/rviz_common/display.cpp
  src/rviz_common/displays_panel.cpp
  src/rviz_common/failed_display.cpp
  src/rviz_common/failed_panel.cpp
  src/rviz_common/failed_tool.cpp
  src/rviz_common/failed_view_controller.cpp
  src/rviz_common/frame_manager.cpp
  src/rviz_common/frame_position_tracking_view_controller.cpp
  # src/rviz_common/help_panel.cpp
  src/rviz_common/load_resource.cpp
  src/rviz_common/loading_dialog.cpp
  src/rviz_common/logging.cpp
  src/rviz_common/new_object_dialog.cpp
  src/rviz_common/panel_dock_widget.cpp
  src/rviz_common/panel_factory.cpp
  src/rviz_common/panel.cpp
  src/rviz_common/properties/bool_property.cpp
  src/rviz_common/properties/color_editor.cpp
  src/rviz_common/properties/color_property.cpp
  src/rviz_common/properties/combo_box.cpp
  src/rviz_common/properties/editable_combo_box.cpp
  src/rviz_common/properties/editable_enum_property.cpp
  src/rviz_common/properties/enum_property.cpp
  src/rviz_common/properties/float_edit.cpp
  src/rviz_common/properties/float_property.cpp
  src/rviz_common/properties/int_property.cpp
  src/rviz_common/properties/line_edit_with_button.cpp
  src/rviz_common/properties/parse_color.cpp
  src/rviz_common/properties/property_tree_delegate.cpp
  src/rviz_common/properties/property_tree_model.cpp
  src/rviz_common/properties/property_tree_widget.cpp
  src/rviz_common/properties/property_tree_with_help.cpp
  src/rviz_common/properties/property.cpp
  src/rviz_common/properties/quaternion_property.cpp
  src/rviz_common/properties/splitter_handle.cpp
  src/rviz_common/properties/status_list.cpp
  src/rviz_common/properties/status_property.cpp
  src/rviz_common/properties/string_property.cpp
  src/rviz_common/properties/tf_frame_property.cpp
  src/rviz_common/properties/vector_property.cpp
  src/rviz_common/render_panel.cpp
  src/rviz_common/ros_integration/get_topic_names_and_types.cpp
  src/rviz_common/ros_integration/init.cpp
  src/rviz_common/ros_integration/ok.cpp
  src/rviz_common/ros_integration/shutdown.cpp
  src/rviz_common/ros_integration/rclcpp_node_storage.cpp
  src/rviz_common/scaled_image_widget.cpp
  src/rviz_common/screenshot_dialog.cpp
  # src/rviz_common/selection_panel.cpp
  src/rviz_common/selection/selection_handler.cpp
  src/rviz_common/selection/selection_manager.cpp
  src/rviz_common/splash_screen.cpp
  # src/rviz_common/time_panel.cpp
  src/rviz_common/tool_manager.cpp
  # src/rviz_common/tool_properties_panel.cpp
  src/rviz_common/tool.cpp
  src/rviz_common/uniform_string_stream.cpp
  src/rviz_common/view_controller.cpp
  src/rviz_common/view_manager.cpp
  src/rviz_common/views_panel.cpp
  src/rviz_common/viewport_mouse_event.cpp
  src/rviz_common/visualization_frame.cpp
  src/rviz_common/visualization_manager.cpp
  src/rviz_common/visualizer_app.cpp
  src/rviz_common/widget_geometry_change_detector.cpp
  src/rviz_common/yaml_config_reader.cpp
  src/rviz_common/yaml_config_writer.cpp
)

# TODO(wjwwood): remove this later
list(APPEND rviz_common_source_files
  src/rviz_common/temp/default_plugins/displays/grid_display.cpp)
list(APPEND rviz_common_source_files
  src/rviz_common/temp/default_plugins/displays/tf_display.cpp)
list(APPEND rviz_common_source_files
  src/rviz_common/temp/default_plugins/displays/robot_model_display.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot_link.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/robot_joint.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/robot/tf_link_updater.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/mesh_loader.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/stl_loader.cpp)

# TODO(greimela): Remove when moving the default plugins to separate package
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_common.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_common.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_display.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/axis_color_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/flat_color_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/intensity_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/rgb8_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/rgbf32_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_transformers/xyz_pc_transformer.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud2_display.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_selection_handler.cpp)
list(APPEND rviz_common_source_files
  src/rviz_default_plugins/point_cloud_to_point_cloud2.cpp)

add_library(rviz_common
  ${rviz_common_headers_to_moc}
  ${rviz_common_source_files}
  ${rviz_default_plugins_source_files}
)
target_include_directories(rviz_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${pluginlib_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)
target_link_libraries(rviz_common
  ${pluginlib_LIBRARIES}
  resource_retriever::resource_retriever
  rviz_rendering::rviz_rendering
  Qt5::Widgets
  ${TinyXML_LIBRARIES}
)
ament_target_dependencies(rviz_common
  geometry_msgs
  rclcpp
  rviz_assimp_vendor
  rviz_yaml_cpp_vendor
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
)
# prevent pluginlib from using boost
target_compile_definitions(rviz_common PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_export_interfaces(rviz_common)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(
  geometry_msgs
  rclcpp
  resource_retriever
  rviz_yaml_cpp_vendor
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_export_libraries(tinyxml2)

install(
  TARGETS rviz_common
  EXPORT rviz_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES default.rviz
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake_gtest REQUIRED)

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

  ament_add_gtest(rviz_common_config_test
    test/config_test.cpp)
  if(TARGET rviz_common_config_test)
    target_link_libraries(rviz_common_config_test rviz_common)
  endif()

  ament_add_gtest(rviz_common_uniform_string_stream_test
    test/uniform_string_stream_test.cpp)
  if(TARGET rviz_common_uniform_string_stream_test)
    target_link_libraries(rviz_common_uniform_string_stream_test rviz_common)
  endif()

  ament_add_gtest(rviz_common_display_test
    test/display_test.cpp
    test/mock_display.cpp
    test/mock_display_group.cpp)
  if(TARGET rviz_common_display_test)
    target_link_libraries(rviz_common_display_test rviz_common)
  endif()

  ament_add_gtest(rviz_common_property_test
    test/property_test.cpp
    test/mock_property_change_receiver.cpp)
  if(TARGET rviz_common_property_test)
    target_link_libraries(rviz_common_property_test rviz_common)
  endif()

  ament_add_gtest(point_cloud2_display_test
    test/rviz_default_plugins/message_creators.hpp
    test/rviz_default_plugins/message_creators.cpp
    test/rviz_default_plugins/point_cloud2_display_test.cpp)
  if(TARGET point_cloud2_display_test)
    target_link_libraries(point_cloud2_display_test rviz_common)
  endif()

  ament_add_gtest(point_cloud_transformers_test
    test/rviz_default_plugins/message_creators.hpp
    test/rviz_default_plugins/message_creators.cpp
    test/rviz_default_plugins/point_cloud_transformers/axis_color_pc_transformer_test.cpp
    test/rviz_default_plugins/point_cloud_transformers/flat_color_pc_transformer_test.cpp
    test/rviz_default_plugins/point_cloud_transformers/intensity_pc_transformer_test.cpp
    test/rviz_default_plugins/point_cloud_transformers/rgb8_pc_transformer_test.cpp
    test/rviz_default_plugins/point_cloud_transformers/rgbf32_pc_transformer_test.cpp
    test/rviz_default_plugins/point_cloud_transformers/xyz_pc_transformer_test.cpp)
  if(TARGET point_cloud_transformers_test)
    target_link_libraries(point_cloud_transformers_test rviz_common)
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_common-extras.cmake"
)
