cmake_minimum_required(VERSION 3.5)

project(rviz_common)

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

find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT WIN32)
  find_package(PkgConfig)
  pkg_check_modules(YAMLCPP yaml-cpp>=0.5 REQUIRED)
  # if(YAMLCPP_FOUND)
  #   add_definitions(-DRVIZ_HAVE_YAMLCPP_05)
  # endif(YAMLCPP_FOUND)
else()
  find_path(YAMLCPP_INCLUDE_DIRS NAMES yaml-cpp/yaml.h)
  find_library(YAMLCPP_LIBRARIES NAMES libyaml-cppmdd libyaml-cppmd)
endif()

find_library(TINYXML_LIBRARIES NAMES tinyxmld tinyxml)

# TODO(greimela) Remove as soon as boost is not required anymore (in temp files)
find_package(Boost REQUIRED)

find_package(ASSIMP REQUIRED)

# TODO(greimela): Temporary fix for Windows, remove this later
if(WIN32)
  set(ASSIMP_LIBRARIES "${ASSIMP_LIBRARY_DIRS}/${ASSIMP_LIBRARIES}.lib")
endif()

include(FindCURL)
if(NOT CURL_FOUND)
  message("CURL not found!  Aborting...")
  fail()
endif()

find_package(urdf REQUIRED)

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
  # src/rviz_common/add_display_dialog.hpp
  # src/rviz_common/help_panel.hpp
  include/rviz_common/properties/enum_property.hpp
  # src/rviz_common/properties/property_tree_widget.hpp
  # src/rviz_common/properties/splitter_handle.hpp
  # src/rviz_common/selection_panel.hpp
  include/rviz_common/frame_position_tracking_view_controller.hpp
  include/rviz_common/properties/quaternion_property.hpp
  include/rviz_common/visualizer_app.hpp
  include/rviz_common/display.hpp
  include/rviz_common/display_context.hpp
  # src/rviz_common/displays_panel.hpp
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
  # src/rviz_common/properties/property_tree_delegate.hpp
  include/rviz_common/properties/property_tree_model.hpp
  # src/rviz_common/properties/property_tree_with_help.hpp
  include/rviz_common/properties/status_list.hpp
  include/rviz_common/properties/status_property.hpp
  include/rviz_common/properties/string_property.hpp
  include/rviz_common/properties/tf_frame_property.hpp
  include/rviz_common/properties/vector_property.hpp
  src/rviz_common/render_panel.hpp
  src/rviz_common/scaled_image_widget.hpp
  src/rviz_common/screenshot_dialog.hpp
  src/rviz_common/selection/selection_manager.hpp
  src/rviz_common/splash_screen.hpp
  # src/rviz_common/time_panel.hpp
  src/rviz_common/tool.hpp
  src/rviz_common/tool_manager.hpp
  # src/rviz_common/tool_properties_panel.hpp
  include/rviz_common/view_controller.hpp
  src/rviz_common/view_manager.hpp
  # src/rviz_common/views_panel.hpp
  src/rviz_common/visualization_frame.hpp
  src/rviz_common/visualization_manager.hpp
  src/rviz_common/widget_geometry_change_detector.hpp
  src/rviz_common/window_manager_interface.hpp
  include/rviz_common/validate_floats.hpp
)

# TODO(wjwwood): remove this later
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/default_plugins/view_controllers/orbit_view_controller.hpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/default_plugins/tools/move_tool.hpp)
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

set(rviz_common_source_files
  ${ENV_CONFIG_CPP}
  # src/rviz_common/add_display_dialog.cpp
  src/rviz_common/bit_allocator.cpp
  src/rviz_common/config.cpp
  # src/rviz_common/display_factory.cpp
  src/rviz_common/display_group.cpp
  src/rviz_common/display.cpp
  # src/rviz_common/displays_panel.cpp
  src/rviz_common/failed_display.cpp
  src/rviz_common/failed_panel.cpp
  # src/rviz_common/failed_tool.cpp
  # src/rviz_common/failed_view_controller.cpp
  src/rviz_common/frame_manager.cpp
  src/rviz_common/frame_position_tracking_view_controller.cpp
  # src/rviz_common/help_panel.cpp
  src/rviz_common/load_resource.cpp
  src/rviz_common/loading_dialog.cpp
  src/rviz_common/logging.cpp
  src/rviz_common/new_object_dialog.cpp
  src/rviz_common/panel_dock_widget.cpp
  # src/rviz_common/panel_factory.cpp
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
  # src/rviz_common/properties/property_tree_delegate.cpp
  src/rviz_common/properties/property_tree_model.cpp
  # src/rviz_common/properties/property_tree_widget.cpp
  # src/rviz_common/properties/property_tree_with_help.cpp
  src/rviz_common/properties/property.cpp
  src/rviz_common/properties/quaternion_property.cpp
  # src/rviz_common/properties/splitter_handle.cpp
  src/rviz_common/properties/status_list.cpp
  src/rviz_common/properties/status_property.cpp
  src/rviz_common/properties/string_property.cpp
  src/rviz_common/properties/tf_frame_property.cpp
  src/rviz_common/properties/vector_property.cpp
  src/rviz_common/render_panel.cpp
  src/rviz_common/ros_integration/init.cpp
  src/rviz_common/ros_integration/ok.cpp
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
  # src/rviz_common/views_panel.cpp
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
  src/rviz_common/temp/default_plugins/view_controllers/orbit_view_controller.cpp)
list(APPEND rviz_common_source_files
  src/rviz_common/temp/default_plugins/tools/move_tool.cpp)
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
  src/rviz_common/temp/resource_retriever/retriever.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/mesh_loader.cpp)
list(APPEND rviz_common_headers_to_moc
  src/rviz_common/temp/stl_loader.cpp)

add_library(rviz_common
  ${rviz_common_headers_to_moc}
  ${rviz_common_source_files}
)
target_include_directories(rviz_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${YAMLCPP_INCLUDE_DIRS}
    ${CURL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    # TODO(greimela) Remove as soon as boost is not required anymore (in temp files)
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Widgets
  ${YAMLCPP_LIBRARIES}
  ${CURL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${TINYXML_LIBRARIES}
  ${urdf_LIBRARIES}
)
ament_target_dependencies(
  rviz_common
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  std_msgs
)

ament_export_interfaces(rviz_common)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(geometry_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros sensor_msgs std_msgs)

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

find_package(ament_cmake_gtest REQUIRED)

if(BUILD_TESTING)
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
      target_include_directories(rviz_common_display_test PUBLIC src)
      target_link_libraries(rviz_common_display_test rviz_common)
    endif()

    ament_add_gtest(rviz_common_property_test
      test/property_test.cpp
      test/mock_property_change_receiver.cpp)
    if(TARGET rviz_common_property_test)
      target_link_libraries(rviz_common_property_test rviz_common)
    endif()
endif()

ament_package()
