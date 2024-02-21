cmake_minimum_required(VERSION 3.5)

project(rviz_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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
endif()

find_package(ament_cmake REQUIRED)
# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(TinyXML2 REQUIRED)  # provided by tinyxml2_vendor

# Copy env_config.hpp so that env_config.cpp can find it
# TODO(jsquare): Get rid of copy hpp file
set(ENV_CONFIG_HPP ${CMAKE_CURRENT_BINARY_DIR}/src/rviz_common/env_config.hpp)
configure_file(src/rviz_common/env_config.hpp ${ENV_CONFIG_HPP} @ONLY)

# TODO(jsquare): Adopt parameters given here
set(ENV_CONFIG_CPP ${CMAKE_CURRENT_BINARY_DIR}/src/rviz_common/env_config.cpp)
ament_package_xml()
set(RVIZ_VERSION "${rviz_common_VERSION}")
set(ROS_DISTRO "ROS 2")
set(OGRE_PLUGIN_PATH "rviz_ogre_vendor")
configure_file(src/rviz_common/env_config.cpp.in ${ENV_CONFIG_CPP} @ONLY)

# These need to be added in the add_library() call
set(rviz_common_headers_to_moc
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
  include/rviz_common/visualization_manager.hpp
  src/rviz_common/scaled_image_widget.hpp
  src/rviz_common/screenshot_dialog.hpp
  include/rviz_common/interaction/selection_manager.hpp
  include/rviz_common/interaction/selection_manager_iface.hpp
  src/rviz_common/splash_screen.hpp
  src/rviz_common/time_panel.hpp
  include/rviz_common/tool.hpp
  include/rviz_common/tool_manager.hpp
  src/rviz_common/tool_properties_panel.hpp
  include/rviz_common/transformation/transformation_manager.hpp
  src/rviz_common/transformation_panel.hpp
  include/rviz_common/view_controller.hpp
  include/rviz_common/view_manager.hpp
  src/rviz_common/views_panel.hpp
  include/rviz_common/visualization_frame.hpp
  src/rviz_common/widget_geometry_change_detector.hpp
  include/rviz_common/ros_topic_display.hpp
)

foreach(header "${rviz_common_headers_to_moc}")
  qt5_wrap_cpp(rviz_common_moc_files "${header}")
endforeach()

set(rviz_common_source_files
  ${ENV_CONFIG_CPP}
  src/rviz_common/add_display_dialog.cpp
  src/rviz_common/bit_allocator.cpp
  src/rviz_common/config.cpp
  src/rviz_common/depth_cloud_mld.cpp
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
  src/rviz_common/time_panel.cpp
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
  ${rviz_common_moc_files}
  ${rviz_common_source_files}
)

target_include_directories(rviz_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(rviz_common PUBLIC
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  pluginlib::pluginlib
  Qt5::Widgets
  rclcpp::rclcpp
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_rendering::rviz_rendering
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  yaml-cpp
)

target_link_libraries(rviz_common PRIVATE
  resource_retriever::resource_retriever
  tinyxml2::tinyxml2
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_common PRIVATE "RVIZ_COMMON_BUILDING_LIBRARY")

ament_export_dependencies(
  geometry_msgs
  message_filters
  pluginlib
  Qt5
  rclcpp
  rviz_ogre_vendor
  rviz_rendering
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  yaml_cpp_vendor
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(rviz_common)

# Export modern CMake targets
ament_export_targets(rviz_common)

install(
  TARGETS rviz_common
  EXPORT rviz_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  qt5_wrap_cpp(rviz_common_test_moc_files test/mock_display.hpp)
  qt5_wrap_cpp(rviz_common_test_moc_files test/mock_property_change_receiver.hpp)

  qt5_wrap_cpp(rviz_common_test_frame_manager_moc src/rviz_common/frame_manager.hpp)

  ament_add_gmock(display_factory_test
    test/display_factory_test.cpp
    src/rviz_common/display_factory.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET display_factory_test)
    target_compile_definitions(display_factory_test PUBLIC
      -D_TEST_PLUGIN_DESCRIPTIONS="${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(display_factory_test rviz_common Qt5::Widgets)
  endif()

  ament_add_gmock(frame_manager_test
    test/frame_manager_test.cpp
    src/rviz_common/frame_manager.cpp
    ${rviz_common_test_frame_manager_moc}
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
    ${rviz_common_test_moc_files}
    test/property_test.cpp
    test/mock_display.cpp
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
    test/ogre_testing_environment.cpp
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
    test/ogre_testing_environment.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET selection_handler_test)
    target_link_libraries(selection_handler_test
      rviz_common
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
    )
  endif()

  ament_add_gtest(rviz_common_display_test
    ${rviz_common_test_moc_files}
    test/display_test.cpp
    test/mock_display.cpp
    test/mock_display_group.cpp
    test/mock_property_change_receiver.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET rviz_common_display_test)
    target_link_libraries(rviz_common_display_test rviz_common Qt5::Widgets yaml-cpp)
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_common-extras.cmake"
)
