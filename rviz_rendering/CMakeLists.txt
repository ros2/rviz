cmake_minimum_required(VERSION 3.5)

project(rviz_rendering)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
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

find_package(ament_cmake_ros REQUIRED)

# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_assimp_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(resource_retriever REQUIRED)

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

# These need to be added in the add_library() call
qt5_wrap_cpp(rviz_rendering_moc_files include/rviz_rendering/render_window.hpp)

add_library(rviz_rendering SHARED
  ${rviz_rendering_moc_files}
  src/rviz_rendering/apply_visibility_bits.cpp
  src/rviz_rendering/geometry.cpp
  src/rviz_rendering/viewport_projection_finder.cpp
  src/rviz_rendering/logging.cpp
  src/rviz_rendering/material_manager.cpp
  src/rviz_rendering/mesh_loader.cpp
  src/rviz_rendering/objects/mesh_shape.cpp
  src/rviz_rendering/ogre_logging.cpp
  src/rviz_rendering/ogre_render_window_impl.cpp
  src/rviz_rendering/orthographic.cpp
  src/rviz_rendering/render_system.cpp
  src/rviz_rendering/render_window.cpp
  src/rviz_rendering/resource_config.cpp
  src/rviz_rendering/mesh_loader_helpers/assimp_loader.cpp
  src/rviz_rendering/string_helper.cpp
  src/rviz_rendering/objects/arrow.cpp
  src/rviz_rendering/objects/axes.cpp
  src/rviz_rendering/objects/billboard_line.cpp
  src/rviz_rendering/objects/covariance_visual.cpp
  src/rviz_rendering/objects/effort_visual.cpp
  src/rviz_rendering/objects/grid.cpp
  src/rviz_rendering/objects/line.cpp
  src/rviz_rendering/objects/movable_text.cpp
  src/rviz_rendering/objects/object.cpp
  src/rviz_rendering/objects/point_cloud.cpp
  src/rviz_rendering/objects/point_cloud_renderable.cpp
  src/rviz_rendering/objects/screw_visual.cpp
  src/rviz_rendering/objects/shape.cpp
  src/rviz_rendering/objects/wrench_visual.cpp
)

target_link_libraries(rviz_rendering PUBLIC
  Qt5::Widgets
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)
if(TARGET Eigen3::Eigen)
  # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
  target_link_libraries(rviz_rendering PUBLIC Eigen3::Eigen)
else()
  target_include_directories(rviz_rendering PUBLIC ${Eigen3_INCLUDE_DIRS})
endif()

target_link_libraries(rviz_rendering PRIVATE
  ament_index_cpp::ament_index_cpp
  assimp::assimp
  resource_retriever::resource_retriever
)

target_include_directories(rviz_rendering
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_rendering PRIVATE "RVIZ_RENDERING_BUILDING_LIBRARY")

ament_export_dependencies(
  Eigen3
  Qt5
  rviz_ogre_vendor
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(rviz_rendering)

# Export modern CMake targets
ament_export_targets(rviz_rendering)

install(
  TARGETS rviz_rendering
  EXPORT rviz_rendering
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

set(rviz_rendering_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake")
include(${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports_hook-extras.cmake)

install(DIRECTORY src/cmake
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ogre_media"
  DESTINATION "share/${PROJECT_NAME}"
  USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  add_library(rviz_rendering_test_utils
    test/rviz_rendering/ogre_testing_environment.cpp
    test/rviz_rendering/scene_graph_introspection.cpp
  )
  target_compile_definitions(rviz_rendering_test_utils PRIVATE
    "TEST_RVIZ_RENDERING_BUILDING_LIBRARY")
  target_link_libraries(rviz_rendering_test_utils
    PUBLIC
    rviz_ogre_vendor::OgreMain
    rviz_rendering
  )

  ament_add_gmock(string_helper_test test/rviz_rendering/string_helper_test.cpp)
  if(TARGET string_helper_test)
    target_link_libraries(string_helper_test rviz_rendering)
  endif()

  ament_add_gmock(point_cloud_test_target
    test/rviz_rendering/objects/point_cloud_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud_test_target)
    target_link_libraries(point_cloud_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(point_cloud_renderable_test_target
    test/rviz_rendering/objects/point_cloud_renderable_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud_renderable_test_target)
    target_link_libraries(point_cloud_renderable_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(billboard_line_test_target
    test/rviz_rendering/objects/billboard_line_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET billboard_line_test_target)
    target_link_libraries(billboard_line_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(covariance_visual_test_target
    test/rviz_rendering/objects/covariance_visual_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET covariance_visual_test_target)
    target_link_libraries(covariance_visual_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(effort_visual_test_target
    test/rviz_rendering/objects/effort_visual_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET effort_visual_test_target)
    target_link_libraries(effort_visual_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(grid_test_target
    test/rviz_rendering/objects/grid_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET grid_test_target)
    target_link_libraries(grid_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(movable_text_test_target
    test/rviz_rendering/objects/movable_text_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET movable_text_test_target)
    target_link_libraries(movable_text_test_target
      rviz_ogre_vendor::OgreMain
      rviz_ogre_vendor::OgreOverlay
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets
    )
  endif()

  ament_add_gmock(line_test_target
    test/rviz_rendering/objects/line_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET line_test_target)
    target_link_libraries(line_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(screw_visual_test_target
    test/rviz_rendering/objects/screw_visual_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET screw_visual_test_target)
    target_link_libraries(screw_visual_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(wrench_visual_test_target
    test/rviz_rendering/objects/wrench_visual_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET wrench_visual_test_target)
    target_link_libraries(wrench_visual_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()

  ament_add_gmock(mesh_shape_visual_test_target
    test/rviz_rendering/mesh_shape_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET mesh_shape_visual_test_target)
    target_link_libraries(mesh_shape_visual_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering
      rviz_rendering_test_utils
      Qt5::Widgets  # explicitly do this for include directories (not necessary for external use)
    )
  endif()
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports_hook-extras.cmake"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/cmake/register_rviz_ogre_media_exports.cmake")

ament_package(
  CONFIG_EXTRAS "rviz_rendering-extras.cmake"
)
