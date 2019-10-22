cmake_minimum_required(VERSION 3.5)

project(rviz_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # TODO(wjwwood): try to remove this -- currently needed to pass on CI
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-Wno-gnu-zero-variadic-macro-arguments HAS_W_FLAG)
  if(HAS_W_FLAG)
    add_compile_options(-Wno-gnu-zero-variadic-macro-arguments)
  endif()
endif()

find_package(ament_cmake REQUIRED)
# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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
  src/rviz_common/help_panel.hpp
  include/rviz_common/properties/enum_property.hpp
  include/rviz_common/properties/property_tree_widget.hpp
  include/rviz_common/properties/splitter_handle.hpp
  include/rviz_common/display_group.hpp
  include/rviz_common/frame_position_tracking_view_controller.hpp
  include/rviz_common/properties/quaternion_property.hpp
  include/rviz_common/visualizer_app.hpp
  include/rviz_common/display.hpp
  include/rviz_common/display_context.hpp
  src/rviz_common/displays_panel.hpp
  src/rviz_common/failed_panel.hpp
  src/rviz_common/frame_manager.hpp
  include/rviz_common/transformation/frame_transformer.hpp
  include/rviz_common/frame_manager_iface.hpp
  src/rviz_common/loading_dialog.hpp
  src/rviz_common/new_object_dialog.hpp
  include/rviz_common/panel.hpp
  include/rviz_common/panel_dock_widget.hpp
  include/rviz_common/properties/bool_property.hpp
  include/rviz_common/properties/color_editor.hpp
  include/rviz_common/properties/color_property.hpp
  include/rviz_common/properties/covariance_property.hpp
  include/rviz_common/properties/combo_box.hpp
  include/rviz_common/properties/display_group_visibility_property.hpp
  include/rviz_common/properties/display_visibility_property.hpp
  include/rviz_common/properties/editable_combo_box.hpp
  include/rviz_common/properties/editable_enum_property.hpp
  include/rviz_common/properties/float_edit.hpp
  include/rviz_common/properties/float_property.hpp
  include/rviz_common/properties/grouped_checkbox_property.hpp
  include/rviz_common/properties/grouped_checkbox_property_group.hpp
  include/rviz_common/properties/int_property.hpp
  include/rviz_common/properties/line_edit_with_button.hpp
  include/rviz_common/properties/property.hpp
  include/rviz_common/properties/property_tree_delegate.hpp
  include/rviz_common/properties/property_tree_model.hpp
  include/rviz_common/properties/property_tree_with_help.hpp
  include/rviz_common/properties/qos_profile_property.hpp
  include/rviz_common/properties/ros_topic_property.hpp
  include/rviz_common/properties/status_list.hpp
  include/rviz_common/properties/status_property.hpp
  include/rviz_common/properties/string_property.hpp
  include/rviz_common/properties/tf_frame_property.hpp
  include/rviz_common/properties/vector_property.hpp
  include/rviz_common/render_panel.hpp
  src/rviz_common/scaled_image_widget.hpp
  src/rviz_common/screenshot_dialog.hpp
  include/rviz_common/interaction/selection_manager.hpp
  include/rviz_common/interaction/selection_manager_iface.hpp
  include/rviz_common/interaction/selection_renderer.hpp
  src/rviz_common/splash_screen.hpp
  # src/rviz_common/time_panel.hpp
  src/rviz_common/transformation/identity_frame_transformer.hpp
  include/rviz_common/tool.hpp
  src/rviz_common/tool_manager.hpp
  src/rviz_common/tool_properties_panel.hpp
  include/rviz_common/transformation/transformation_manager.hpp
  src/rviz_common/transformation_panel.hpp
  include/rviz_common/view_controller.hpp
  include/rviz_common/view_manager.hpp
  src/rviz_common/views_panel.hpp
  src/rviz_common/visualization_frame.hpp
  src/rviz_common/visualization_manager.hpp
  src/rviz_common/widget_geometry_change_detector.hpp
  include/rviz_common/window_manager_interface.hpp
  include/rviz_common/validate_floats.hpp
  include/rviz_common/ros_topic_display.hpp
  include/rviz_common/message_filter_display.hpp
)

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
  src/rviz_common/help_panel.cpp
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
  src/rviz_common/properties/covariance_property.cpp
  src/rviz_common/properties/display_group_visibility_property.cpp
  src/rviz_common/properties/display_visibility_property.cpp
  src/rviz_common/properties/editable_combo_box.cpp
  src/rviz_common/properties/editable_enum_property.cpp
  src/rviz_common/properties/enum_property.cpp
  src/rviz_common/properties/file_picker.cpp
  src/rviz_common/properties/file_picker_property.cpp
  src/rviz_common/properties/float_edit.cpp
  src/rviz_common/properties/float_property.cpp
  src/rviz_common/properties/grouped_checkbox_property.cpp
  src/rviz_common/properties/grouped_checkbox_property_group.cpp
  src/rviz_common/properties/int_property.cpp
  src/rviz_common/properties/line_edit_with_button.cpp
  src/rviz_common/properties/parse_color.cpp
  src/rviz_common/properties/property_tree_delegate.cpp
  src/rviz_common/properties/property_tree_model.cpp
  src/rviz_common/properties/property_tree_widget.cpp
  src/rviz_common/properties/property_tree_with_help.cpp
  src/rviz_common/properties/property.cpp
  src/rviz_common/properties/ros_topic_property.cpp
  src/rviz_common/properties/quaternion_property.cpp
  src/rviz_common/properties/qos_profile_property.cpp
  src/rviz_common/properties/splitter_handle.cpp
  src/rviz_common/properties/status_list.cpp
  src/rviz_common/properties/status_property.cpp
  src/rviz_common/properties/string_property.cpp
  src/rviz_common/properties/tf_frame_property.cpp
  src/rviz_common/properties/vector_property.cpp
  src/rviz_common/render_panel.cpp
  src/rviz_common/ros_integration/ros_client_abstraction.cpp
  src/rviz_common/ros_integration/ros_node_abstraction.cpp
  src/rviz_common/scaled_image_widget.cpp
  src/rviz_common/screenshot_dialog.cpp
  src/rviz_common/selection_panel.cpp
  src/rviz_common/interaction/handler_manager.cpp
  src/rviz_common/interaction/selection_handler.cpp
  src/rviz_common/interaction/selection_manager.cpp
  src/rviz_common/interaction/selection_renderer.cpp
  src/rviz_common/interaction/view_picker.cpp
  src/rviz_common/splash_screen.cpp
  # src/rviz_common/time_panel.cpp
  src/rviz_common/transformation/identity_frame_transformer.cpp
  src/rviz_common/tool_manager.cpp
  src/rviz_common/tool_properties_panel.cpp
  src/rviz_common/tool.cpp
  src/rviz_common/transformation_panel.cpp
  src/rviz_common/transformation/transformation_manager.cpp
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

add_library(rviz_common SHARED
  ${rviz_common_headers_to_moc}
  ${rviz_common_source_files}
)

target_include_directories(rviz_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${pluginlib_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

target_link_libraries(rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ${pluginlib_LIBRARIES}
  rviz_rendering::rviz_rendering
  Qt5::Widgets
  ${TinyXML_LIBRARIES}
)

ament_target_dependencies(rviz_common
  geometry_msgs
  rclcpp
  resource_retriever
  rviz_assimp_vendor
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  message_filters
  urdf
  yaml_cpp_vendor
)
# prevent pluginlib from using boost
target_compile_definitions(rviz_common PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_common PRIVATE "RVIZ_COMMON_BUILDING_LIBRARY")

ament_export_interfaces(rviz_common)
ament_export_dependencies(
  rviz_rendering
  geometry_msgs
  pluginlib
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  yaml_cpp_vendor
)
ament_export_include_directories(include)
ament_export_libraries(rviz_common)

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

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/images"
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/help"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  # TODO(wjwwood): replace this with ament_lint_auto() and/or add the copyright linter back
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  ament_add_gmock(frame_manager_test
    test/frame_manager_test.cpp
    src/rviz_common/frame_manager.cpp
  )
  if(TARGET frame_manager_test)
    target_link_libraries(frame_manager_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gtest(rviz_common_config_test
    test/config_test.cpp
  )
  if(TARGET rviz_common_config_test)
    target_link_libraries(rviz_common_config_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gtest(rviz_common_uniform_string_stream_test
    test/uniform_string_stream_test.cpp
  )
  if(TARGET rviz_common_uniform_string_stream_test)
    target_link_libraries(rviz_common_uniform_string_stream_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gtest(rviz_common_property_test
    test/property_test.cpp
    test/mock_property_change_receiver.cpp
  )
  if(TARGET rviz_common_property_test)
    target_link_libraries(rviz_common_property_test rviz_common)
  endif()

  ament_add_gmock(qos_profile_property_test test/properties/qos_profile_property_test.cpp)
  if(TARGET qos_profile_property_test)
    target_link_libraries(qos_profile_property_test rviz_common)
  endif()

  ament_add_gmock(rviz_common_visualizer_app_test test/visualizer_app_test.cpp)
  if(TARGET rviz_common_visualizer_app_test)
    target_link_libraries(rviz_common_visualizer_app_test rviz_common)
  endif()

  ament_add_gmock(rviz_common_ros_node_abstraction_test
    test/ros_node_abstraction_test.cpp
  )
  if(TARGET rviz_common_ros_node_abstraction_test)
    target_link_libraries(rviz_common_ros_node_abstraction_test rviz_common)
  endif()

  ament_add_gmock(identity_transformer_test
    test/transformation/identity_frame_transformer_test.cpp
  )
  if(TARGET identity_transformer_test)
    target_link_libraries(identity_transformer_test rviz_common)
  endif()

  ament_add_gmock(selection_manager_test
    test/interaction/selection_manager_test.cpp
    test/interaction/mock_selection_renderer.hpp
    test/interaction/selection_test_fixture.hpp
    test/display_context_fixture.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET selection_manager_test)
    target_link_libraries(selection_manager_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gmock(selection_handler_test
    test/interaction/selection_handler_test.cpp
    test/interaction/selection_test_fixture.hpp
    test/display_context_fixture.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET selection_handler_test)
    target_link_libraries(selection_handler_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gtest(rviz_common_display_test
    test/display_test.cpp
    test/mock_display.cpp
    test/mock_display_group.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET rviz_common_display_test)
    target_link_libraries(rviz_common_display_test rviz_common Qt5::Widgets)
    ament_target_dependencies(rviz_common_display_test yaml_cpp_vendor)
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_common-extras.cmake"
)
