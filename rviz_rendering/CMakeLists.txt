cmake_minimum_required(VERSION 3.5)

project(rviz_rendering)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(ament_index_cpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)

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
  # src/rviz_rendering/initialization.cpp
  src/rviz_rendering/geometry.cpp
  src/rviz_rendering/logging.cpp
  src/rviz_rendering/material_manager.cpp
  src/rviz_rendering/mesh_loader.cpp
  src/rviz_rendering/ogre_logging.cpp
  src/rviz_rendering/ogre_render_window_impl.cpp
  src/rviz_rendering/orthographic.cpp
  src/rviz_rendering/render_system.cpp
  src/rviz_rendering/render_window.cpp
  src/rviz_rendering/resource_config.cpp
  src/rviz_rendering/mesh_loader_helpers/stl_loader.cpp
  src/rviz_rendering/mesh_loader_helpers/assimp_loader.cpp
  src/rviz_rendering/string_helper.cpp
  src/rviz_rendering/objects/arrow.cpp
  src/rviz_rendering/objects/axes.cpp
  src/rviz_rendering/objects/billboard_line.cpp
  src/rviz_rendering/objects/grid.cpp
  src/rviz_rendering/objects/line.cpp
  src/rviz_rendering/objects/movable_text.cpp
  src/rviz_rendering/objects/object.cpp
  src/rviz_rendering/objects/point_cloud.cpp
  src/rviz_rendering/objects/point_cloud_renderable.cpp
  src/rviz_rendering/objects/shape.cpp
  test/rviz_rendering/ogre_testing_environment.cpp
)

target_link_libraries(rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
  ament_index_cpp::ament_index_cpp
  resource_retriever::resource_retriever
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
ament_target_dependencies(rviz_rendering
  rviz_assimp_vendor)

ament_export_interfaces(rviz_rendering)
ament_export_dependencies(rviz_ogre_vendor
  resource_retriever
  ament_index_cpp)

ament_export_include_directories(include)

install(
  TARGETS rviz_rendering
  EXPORT rviz_rendering
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
  RUNTIME DESTINATION bin
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
  DESTINATION "share/${PROJECT_NAME}"
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

  ament_add_gmock(string_helper_test test/rviz_rendering/string_helper_test.cpp)
  if(TARGET string_helper_test)
    target_link_libraries(string_helper_test rviz_rendering)
  endif()

  if(DEFINED ENV{DISPLAY})
    set(DISPLAYPRESENT TRUE)
  endif()

  if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "TRUE")
    message(STATUS "Enabling tests requiring a display")

    ament_add_gmock(point_cloud_test_target
      test/rviz_rendering/objects/point_cloud_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET point_cloud_test_target)
      target_link_libraries(point_cloud_test_target
        rviz_ogre_vendor::OgreMain
        rviz_rendering
        Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
      )
    endif()

    ament_add_gmock(point_cloud_renderable_test_target
      test/rviz_rendering/objects/point_cloud_renderable_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET point_cloud_renderable_test_target)
      target_link_libraries(point_cloud_renderable_test_target
        rviz_ogre_vendor::OgreMain
        rviz_rendering
        Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
      )
    endif()

    ament_add_gmock(billboard_line_test_target
      test/rviz_rendering/objects/billboard_line_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET billboard_line_test_target)
      target_link_libraries(billboard_line_test_target
        rviz_ogre_vendor::OgreMain
        rviz_rendering
        Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
      )
    endif()

    ament_add_gmock(grid_test_target
      test/rviz_rendering/objects/grid_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET grid_test_target)
      target_link_libraries(grid_test_target
        rviz_ogre_vendor::OgreMain
        rviz_rendering
        Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
      )
    endif()

    ament_add_gmock(movable_text_test_target
      test/rviz_rendering/objects/movable_text_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET movable_text_test_target)
      target_link_libraries(movable_text_test_target
        rviz_ogre_vendor::OgreMain
        rviz_ogre_vendor::OgreOverlay
        rviz_rendering
        Qt5::Widgets
      )
    endif()

    ament_add_gmock(line_test_target
      test/rviz_rendering/objects/line_test.cpp
      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
    if(TARGET line_test_target)
      target_link_libraries(line_test_target
        rviz_ogre_vendor::OgreMain
        rviz_rendering
        Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
      )
    endif()
  endif()
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports_hook-extras.cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports.cmake")

ament_package()
