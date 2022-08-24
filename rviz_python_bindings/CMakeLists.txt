cmake_minimum_required(VERSION 3.8)
project(rviz_python_bindings)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(rviz_shiboken_QT_COMPONENTS
  QtCore
  QtGui
  QtWidgets
)

find_package(ament_cmake REQUIRED)
find_package(python_qt_binding REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
include(shiboken_helper.cmake)

if(shiboken_helper_FOUND)
  set(_configure_shiboken TRUE)
  if(_configure_shiboken)

    # To add a new rviz class to the bindings, add it:
    # - in rviz_shiboken_SRCS below, like rviz_myclass_wrapper.cpp
    # - in rviz_HDRS below, like ../rviz/my_class.h
    # - in global.h, like #include <rviz/my_class.h>
    # - in typesystem.xml, like <object-type name="rviz::MyClass"/>

    set(rviz_shiboken_SRCS
      librviz_shiboken/librviz_shiboken_module_wrapper.cpp
      librviz_shiboken/rviz_common_visualizationframe_wrapper.cpp
      librviz_shiboken/rviz_common_visualizationmanager_wrapper.cpp
      librviz_shiboken/rviz_common_display_wrapper.cpp
      librviz_shiboken/rviz_common_displaygroup_wrapper.cpp
      librviz_shiboken/rviz_rendering_ogrelogging_wrapper.cpp
      librviz_shiboken/rviz_common_properties_property_wrapper.cpp
      librviz_shiboken/rviz_common_properties_boolproperty_wrapper.cpp
      librviz_shiboken/rviz_common_paneldockwidget_wrapper.cpp
      librviz_shiboken/rviz_common_viewmanager_wrapper.cpp
      librviz_shiboken/rviz_common_tool_wrapper.cpp
      librviz_shiboken/rviz_common_toolmanager_wrapper.cpp
      librviz_shiboken/rviz_common_config_wrapper.cpp
      librviz_shiboken/rviz_common_config_mapiterator_wrapper.cpp
      librviz_shiboken/rviz_common_yamlconfigreader_wrapper.cpp
      librviz_shiboken/rviz_common_yamlconfigwriter_wrapper.cpp
      librviz_shiboken/rviz_common_wrapper.cpp
      librviz_shiboken/rviz_rendering_wrapper.cpp
      librviz_shiboken/rviz_common_properties_wrapper.cpp
    )
    

    set(rviz_HDRS
      global.hpp
      # ${rviz_common_INCLUDE_DIRS}/visualization_frame.hpp
      # ${rviz_common_INCLUDE_DIRS}/visualization_manager.hpp
      # ${rviz_common_INCLUDE_DIRS}/view_manager.hpp
      # ${rviz_common_INCLUDE_DIRS}/view_controller.hpp
      # ${rviz_common_INCLUDE_DIRS}/display.hpp
      # ${rviz_common_INCLUDE_DIRS}/display_group.hpp
      # ${rviz_common_INCLUDE_DIRS}/properties/property.hpp
      # ${rviz_common_INCLUDE_DIRS}/properties/bool_property.hpp
      # ${rviz_common_INCLUDE_DIRS}/panel_dock_widget.hpp
      # ${rviz_common_INCLUDE_DIRS}/tool.hpp
      # ${rviz_common_INCLUDE_DIRS}/tool_manager.hpp
      # ${rviz_rendering_INCLUDE_DIRS}/ogre_helpers/ogre_logging.hpp
      # ${rviz_common_INCLUDE_DIRS}/config.hpp
      # ${rviz_common_INCLUDE_DIRS}/yaml_config_reader.hpp
      # ${rviz_common_INCLUDE_DIRS}/yaml_config_writer.hpp
    )

    list(APPEND rviz_shiboken_INCLUDE_DIRS 
      ${rviz_common_INCLUDE_DIRS} 
      ${rviz_rendering_INCLUDE_DIRS} 
      ${Qt5Core_INCLUDE_DIRS} 
      ${Qt5Gui_INCLUDE_DIRS} 
      ${Qt5Widgets_INCLUDE_DIRS} 
      ${OGRE_INCLUDE_DIRS}
      ${rclcpp_INCLUDE_DIRS}
      ${rmw_INCLUDE_DIRS}
      ${rosidl_typesupport_interface_INCLUDE_DIRS}
      ${rcl_interfaces_INCLUDE_DIRS}
      ${tf2_msgs_INCLUDE_DIRS}
      ${pluginlib_INCLUDE_DIRS}
      )
    message(INFO ${PROJECT_SOURCE_DIR}/shiboken)
    shiboken_generator(
      librviz
      global.hpp
      typesystem.xml
      ${PROJECT_SOURCE_DIR}/
      "${rviz_shiboken_SRCS}"
      "${rviz_HDRS}"
      "${rviz_shiboken_INCLUDE_DIRS}"
      "${CMAKE_CURRENT_BINARY_DIR}"
    )

    include_directories(rviz_shiboken
    ${rviz_common_INCLUDE_DIRS} 
    ${rviz_rendering_INCLUDE_DIRS} 
    ${Qt5Core_INCLUDE_DIRS} 
    ${Qt5Gui_INCLUDE_DIRS} 
    ${Qt5Widgets_INCLUDE_DIRS} 
    ${OGRE_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rosidl_typesupport_interface_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    )

    shiboken_include_directories(rviz_shiboken "${rviz_shiboken_QT_COMPONENTS}")
    add_library(rviz_shiboken ${rviz_shiboken_SRCS})
    target_link_libraries(rviz_shiboken ${PROJECT_NAME})
    shiboken_target_link_libraries(rviz_shiboken "${rviz_shiboken_QT_COMPONENTS}")
    target_compile_features(rviz_shiboken PUBLIC c_std_99 cxx_std_17)
    target_compile_options(rviz_shiboken PUBLIC -fPIC -Wl,--no-undefined -w)

    install(TARGETS rviz_shiboken
      LIBRARY DESTINATION ${AMENT})
  endif()
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
