cmake_minimum_required(VERSION 3.5)

project(rviz_rendering)

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
find_package(ament_cmake_gtest REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_index_cpp REQUIRED)
# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
# TODO(wjwwood): this block is to setup the windeployqt tool, could be removed later.
if(Qt5_FOUND AND WIN32 AND TARGET Qt5::qmake AND NOT TARGET Qt5::windeployqt)
  get_target_property(_qt5_qmake_location Qt5::qmake IMPORTED_LOCATION)

  execute_process(
    COMMAND "${_qt5_qmake_location}" -query QT_INSTALL_PREFIX
    RESULT_VARIABLE return_code
    OUTPUT_VARIABLE qt5_install_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  set(imported_location "${qt5_install_prefix}/bin/windeployqt.exe")

  if(EXISTS ${imported_location})
    add_executable(Qt5::windeployqt IMPORTED)

    set_target_properties(Qt5::windeployqt PROPERTIES
      IMPORTED_LOCATION ${imported_location}
    )
  endif()
endif()

# These need to be added in the add_library() call so AUTOMOC detects them.
set(headers_with_q_object
  include/rviz_rendering/render_window.hpp
)

add_library(rviz_rendering SHARED
  ${headers_with_q_object}
  src/rviz_rendering/apply_visibility_bits.cpp
  src/rviz_rendering/arrow.cpp
  src/rviz_rendering/axes.cpp
  src/rviz_rendering/billboard_line.cpp
  # src/rviz_rendering/initialization.cpp
  src/rviz_rendering/logging.cpp
  src/rviz_rendering/geometry.cpp
  src/rviz_rendering/movable_text.cpp
  src/rviz_rendering/object.cpp
  src/rviz_rendering/ogre_logging.cpp
  src/rviz_rendering/orthographic.cpp
  src/rviz_rendering/render_system.cpp
  src/rviz_rendering/render_window.cpp
  src/rviz_rendering/ogre_render_window_impl.cpp
  src/rviz_rendering/resource_config.cpp
  src/rviz_rendering/shape.cpp
  src/rviz_rendering/grid.cpp
  src/rviz_rendering/string_helper.cpp
  src/rviz_rendering/point_cloud.cpp
  src/rviz_rendering/point_cloud_renderable.cpp
  src/test/rviz_rendering/ogre_testing_environment.cpp
)
target_link_libraries(rviz_rendering
  PRIVATE rviz_ogre_vendor::OgreMain
  PRIVATE rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
  ament_index_cpp::ament_index_cpp
)
target_include_directories(rviz_rendering
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OGRE_INCLUDE_DIRS}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_rendering PRIVATE "RVIZ_RENDERING_BUILDING_LIBRARY")

ament_export_interfaces(rviz_rendering)
ament_export_dependencies(rviz_ogre_vendor)
ament_export_dependencies(ament_index_cpp)

install(
  TARGETS rviz_rendering
  EXPORT rviz_rendering
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

# add_executable(rendering_example src/examples/rendering_example.cpp)
# target_link_libraries(rendering_example rviz_rendering Qt5::Widgets)
# set_target_properties(rendering_example PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# # TODO(wjwwood): find a way to make this optional or to run without "deploying" the
# #                necessary dlls and stuff to the bin folder.
# #                see:
# # https://stackoverflow.com/questions/41193584/deploy-all-qt-dependencies-when-building#41199492
# if(TARGET Qt5::windeployqt)
#   # execute windeployqt in a tmp directory after build
#   add_custom_command(TARGET rendering_example
#     POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
#     COMMAND set PATH=%PATH%$<SEMICOLON>${qt5_install_prefix}/bin
#     COMMAND
#     Qt5::windeployqt
#     --dir "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
#     "$<TARGET_FILE_DIR:rendering_example>/$<TARGET_FILE_NAME:rendering_example>"
#   )

#   # copy deployment directory during installation
#   install(
#     DIRECTORY
#     "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
#     DESTINATION bin
#   )
# endif()

set(rviz_rendering_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake")
include(${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports_hook-extras.cmake)

install(DIRECTORY src/cmake
  DESTINATION share/${PROJECT_NAME})

# install(TARGETS rendering_example DESTINATION bin)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/ogre_media"
  DESTINATION "share/rviz_rendering"
  USE_SOURCE_PERMISSIONS)

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

  ament_add_gtest(test_rviz_rendering src/test/string_helper_test.cpp)
  if(TARGET test_rviz_rendering)
    target_link_libraries(test_rviz_rendering rviz_rendering)
  endif()

  # TODO(wjwwood): reenable this test when it can be run on CI
  # ament_add_gtest(point_cloud_test_target
  #   src/test/point_cloud_test.cpp)
  # if(TARGET point_cloud_test_target)
  #   target_link_libraries(point_cloud_test_target
  #     rviz_ogre_vendor::OgreMain
  #     rviz_rendering
  #     Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
  #   )
  # endif()

  # TODO(wjwwood): reenable this test when it can be run on CI
  # ament_add_gtest(point_cloud_renderable_test_target src/test/point_cloud_renderable_test.cpp)
  # if(TARGET point_cloud_renderable_test_target)
  #   target_link_libraries(point_cloud_renderable_test_target
  #     rviz_ogre_vendor::OgreMain
  #     rviz_rendering
  #     Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
  #   )
  # endif()

  # TODO(Martin-Idel-SI): reenable this test when it can be run on CI
  # ament_add_gtest(billboard_line_test_target src/test/billboard_line_test.cpp)
  # if(TARGET billboard_line_test_target)
  #   target_link_libraries(billboard_line_test_target
  #     rviz_ogre_vendor::OgreMain
  #     rviz_rendering
  #     Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
  # )
  # endif()

  # TODO(Martin-Idel-SI): reenable this test when it can be run on CI
  # ament_add_gtest(grid_test_target src/test/grid_test.cpp)
  # if(TARGET grid_test_target)
  #   target_link_libraries(grid_test_target
  #     rviz_ogre_vendor::OgreMain
  #     rviz_rendering
  #     Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
  #   )
  # endif()
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports_hook-extras.cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports.cmake")

ament_package()
