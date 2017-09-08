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

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_common_headers_to_moc
  include/rviz_common/visualizer_app.hpp
  # include/rviz_common/logging.hpp
  # include/rviz_common/ros_integration/init.hpp
  # include/rviz_common/ros_integration/ok.hpp
  # src/rviz_common/add_display_dialog.hpp
  # src/rviz_common/display_context.hpp
  # src/rviz_common/display.hpp
  # src/rviz_common/displays_panel.hpp
  # src/rviz_common/failed_panel.hpp
  # src/rviz_common/frame_manager.hpp
  # src/rviz_common/help_panel.hpp
  # src/rviz_common/loading_dialog.hpp
  # src/rviz_common/new_object_dialog.hpp
  # src/rviz_common/panel_dock_widget.hpp
  # src/rviz_common/panel.hpp
  # src/rviz_common/render_panel.hpp
  # src/rviz_common/scaled_image_widget.hpp
  # src/rviz_common/screenshot_dialog.hpp
  # src/rviz_common/selection_panel.hpp
  # src/rviz_common/splash_screen.hpp
  # src/rviz_common/time_panel.hpp
  # src/rviz_common/tool.hpp
  # src/rviz_common/tool_manager.hpp
  # src/rviz_common/tool_properties_panel.hpp
  # src/rviz_common/view_controller.hpp
  # src/rviz_common/view_manager.hpp
  # src/rviz_common/views_panel.hpp
  # src/rviz_common/visualization_frame.hpp
  # src/rviz_common/visualization_manager.hpp
  # src/rviz_common/widget_geometry_change_detector.hpp
  src/rviz_common/properties/bool_property.hpp
  src/rviz_common/properties/color_editor.hpp
  src/rviz_common/properties/color_property.hpp
  src/rviz_common/properties/combo_box.hpp
  src/rviz_common/properties/editable_combo_box.hpp
  src/rviz_common/properties/editable_enum_property.hpp
  src/rviz_common/properties/enum_property.hpp
  src/rviz_common/properties/float_edit.hpp
  src/rviz_common/properties/float_property.hpp
  src/rviz_common/properties/int_property.hpp
  src/rviz_common/properties/line_edit_with_button.hpp
  src/rviz_common/properties/property.hpp
  src/rviz_common/properties/property_tree_delegate.hpp
  src/rviz_common/properties/property_tree_model.hpp
  src/rviz_common/properties/property_tree_widget.hpp
  src/rviz_common/properties/property_tree_with_help.hpp
  src/rviz_common/properties/splitter_handle.hpp
  src/rviz_common/properties/status_list.hpp
  src/rviz_common/properties/status_property.hpp
  src/rviz_common/properties/string_property.hpp
  # src/rviz_common/properties/tf_frame_property.hpp
  # src/rviz_common/selection/selection_manager.hpp
)
set(rviz_common_source_files
  # src/rviz_common/add_display_dialog.cpp
  # src/rviz_common/bit_allocator.cpp
  src/rviz_common/config.cpp
  # src/rviz_common/display_factory.cpp
  # src/rviz_common/display_group.cpp
  # src/rviz_common/display.cpp
  # src/rviz_common/displays_panel.cpp
  # src/rviz_common/failed_display.cpp
  # src/rviz_common/failed_panel.cpp
  # src/rviz_common/failed_tool.cpp
  # src/rviz_common/failed_view_controller.cpp
  # src/rviz_common/frame_manager.cpp
  # src/rviz_common/help_panel.cpp
  src/rviz_common/load_resource.cpp
  # src/rviz_common/loading_dialog.cpp
  # src/rviz_common/new_object_dialog.cpp
  # src/rviz_common/panel_dock_widget.cpp
  # src/rviz_common/panel_factory.cpp
  # src/rviz_common/panel.cpp
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
  src/rviz_common/properties/splitter_handle.cpp
  src/rviz_common/properties/status_list.cpp
  src/rviz_common/properties/status_property.cpp
  src/rviz_common/properties/string_property.cpp
  # src/rviz_common/properties/tf_frame_property.cpp
  # src/rviz_common/render_panel.cpp
  # src/rviz_common/scaled_image_widget.cpp
  # src/rviz_common/screenshot_dialog.cpp
  # src/rviz_common/selection_panel.cpp
  # src/rviz_common/selection/selection_handler.cpp
  # src/rviz_common/selection/selection_manager.cpp
  # src/rviz_common/splash_screen.cpp
  # src/rviz_common/time_panel.cpp
  # src/rviz_common/tool_manager.cpp
  # src/rviz_common/tool_properties_panel.cpp
  # src/rviz_common/tool.cpp
  # src/rviz_common/view_controller.cpp
  # src/rviz_common/view_manager.cpp
  # src/rviz_common/views_panel.cpp
  # src/rviz_common/visualization_frame.cpp
  # src/rviz_common/visualization_manager.cpp
  src/rviz_common/visualizer_app.cpp
  # src/rviz_common/widget_geometry_change_detector.cpp
  # src/rviz_common/yaml_config_reader.cpp
  # src/rviz_common/yaml_config_writer.cpp
)

add_library(rviz_common
  ${rviz_common_headers_to_moc}
  ${rviz_common_source_files}
)
target_include_directories(rviz_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rviz_common
  rviz_rendering::rviz_rendering
  Qt5::Widgets
)

ament_export_interfaces(rviz_common)
ament_export_dependencies(rviz_rendering)

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
endif()

ament_package()
