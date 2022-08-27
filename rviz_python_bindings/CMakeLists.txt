cmake_minimum_required(VERSION 3.8)
project(rviz_python_bindings)

set(bindings_library "rviz_shiboken")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Shiboken2 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)


add_library(rviz_python SHARED
  src/visualizer_frame_py.cpp
)

target_include_directories(rviz_python PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(rviz_python PUBLIC -Wl,--no-undefined)
ament_target_dependencies(rviz_python
  rviz_common
  Qt5
)

install(TARGETS rviz_python
  EXPORT export_rviz_python
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

message(STATUS "LIBRARY OUTPUT DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/rviz")
if(Shiboken2_FOUND)
  message(STATUS "Found Shiboken2 version ${Shiboken2_VERSION}")
  if(NOT ${Shiboken2_VERSION} VERSION_LESS "5.13")
    get_property(SHIBOKEN_INCLUDE_DIR TARGET Shiboken2::libshiboken PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    get_property(SHIBOKEN_LIBRARY TARGET Shiboken2::libshiboken PROPERTY LOCATION)
    set(SHIBOKEN_BINARY Shiboken2::shiboken2)
  endif()
  message(STATUS "Using SHIBOKEN_INCLUDE_DIR: ${SHIBOKEN_INCLUDE_DIR}")
  message(STATUS "Using SHIBOKEN_LIBRARY: ${SHIBOKEN_LIBRARY}")
  message(STATUS "Using SHIBOKEN_BINARY: ${SHIBOKEN_BINARY}")
endif()

find_package(PySide2 QUIET)
if(PySide2_FOUND)
  message(STATUS "Found PySide2 version ${PySide2_VERSION}")
  if(NOT ${PySide2_VERSION} VERSION_LESS "5.13")
    get_property(PYSIDE_INCLUDE_DIR TARGET PySide2::pyside2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    get_property(PYSIDE_LIBRARY TARGET PySide2::pyside2 PROPERTY LOCATION)
  endif()
  message(STATUS "Using PYSIDE_INCLUDE_DIR: ${PYSIDE_INCLUDE_DIR}")
  message(STATUS "Using PYSIDE_LIBRARY: ${PYSIDE_LIBRARY}")
  message(STATUS "Using PYSIDE_TYPESYSTEMS: ${PYSIDE_TYPESYSTEMS}")
endif()

set(generated_sources
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_shiboken_module_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_visualizerframepy_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_yamlconfigreader_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_yamlconfigwriter_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_config_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_visualizationframe_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_visualizationmanager_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_displaygroup_wrapper.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${bindings_library}/rviz_common_display_wrapper.cpp
    )

list(APPEND SHIBOKEN_HELPER_INCLUDE_DIRS
    ${QT_INCLUDE_DIR}
    ${PYSIDE_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIR}
    ${SHIBOKEN_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}
    )

  set(SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS "")
  foreach(dir ${SHIBOKEN_HELPER_INCLUDE_DIRS})
    set(SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS "${SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS}:${dir}")
  endforeach()

set(shiboken_options --generator-set=shiboken 
    --avoid-protected-hack
    --enable-pyside-extensions
    -I${CMAKE_SOURCE_DIR}:${SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS}
    -T${CMAKE_SOURCE_DIR}:${PYSIDE_TYPESYSTEMS}
    --output-directory=${CMAKE_CURRENT_BINARY_DIR}
    --language-level=c++17
    )


add_custom_command(OUTPUT ${generated_sources}
    COMMAND ${SHIBOKEN_BINARY}
    ${shiboken_options} ${CMAKE_CURRENT_SOURCE_DIR}/global.hpp ${CMAKE_CURRENT_SOURCE_DIR}/typesystem.xml
    DEPENDS ${generated_sources_dependencies}
    IMPLICIT_DEPENDS CXX global.hpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Running generator for ${typesystem_file}.")

ament_python_install_package(rviz)

add_library(${bindings_library} MODULE ${generated_sources})

# Apply relevant include and link flags.
target_include_directories(${bindings_library} PRIVATE 
    ${QT_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIR}
    ${SHIBOKEN_INCLUDE_DIR}
    ${PYSIDE_INCLUDE_DIR}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${PYSIDE_INCLUDE_DIR}/QtCore
    ${PYSIDE_INCLUDE_DIR}/QtGui
    ${PYSIDE_INCLUDE_DIR}/QtWidgets
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}
  )

target_link_libraries(${bindings_library} PRIVATE
  ${QT_LIBRARIES}
  ${PYTHON_LIBRARY}
  ${SHIBOKEN_LIBRARY}
  ${PYSIDE_LIBRARY}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  rviz_python
  ${rviz_common_LIBRARIES}
)

set_target_properties(${bindings_library} PROPERTIES
PREFIX "")


install(TARGETS ${bindings_library}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/rviz
)


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

ament_export_include_directories(
  include
)
ament_export_libraries(
  rviz_python
)
ament_export_targets(
  export_rviz_python
)

ament_package()
