cmake_minimum_required(VERSION 3.5)

project(rviz_visual_tests)

# Options to decide whether or not the visual test will run and in which modality (TEST or REF)
option(EnableVisualTests "decides whether or not enable the tests")
option(GenerateReferenceImages "decides whether the reference images are updated")

# Option to set a non null threshold in the image comparison (in case of unidentical images).
set(MSEThreshold "0.01" CACHE STRING "default value of the threshold")
add_definitions(-D_MSE_THRESHOLD="${MSEThreshold}")

# Path variables needed to save and find screenshots for image comparison.
add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Creates a directory where the test screenshots will be saved.
file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)

# Decides whether the test or the reference screenshots will be taken.
if(GenerateReferenceImages STREQUAL "TRUE")
  add_definitions(-D_REF_IMAGES=1)
else()
  add_definitions(-D_REF_IMAGES=0)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(rviz_common REQUIRED)

find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(ament_cmake_gtest REQUIRED)

set(visual_test_framework_source_files
  tests/example_nodes.hpp
  include/rviz_visual_tests/page_objects/base_page_object.hpp
  src/page_objects/base_page_object.cpp
  include/rviz_visual_tests/page_objects/camera_display_page_object.hpp
  src/page_objects/camera_display_page_object.cpp
  include/rviz_visual_tests/page_objects/grid_display_page_object.hpp
  src/page_objects/grid_display_page_object.cpp
  include/rviz_visual_tests/page_objects/image_display_page_object.hpp
  src/page_objects/image_display_page_object.cpp
  include/rviz_visual_tests/page_objects/page_object_with_window.hpp
  src/page_objects/page_object_with_window.cpp
  include/rviz_visual_tests/page_objects/point_cloud_display_page_object.hpp
  src/page_objects/point_cloud_display_page_object.cpp
  include/rviz_visual_tests/page_objects/polygon_display_page_object.hpp
  src/page_objects/polygon_display_page_object.cpp
  src/internal/rviz_scene_test.cpp
  include/rviz_visual_tests/internal/rviz_scene_test.hpp
  include/rviz_visual_tests/internal/display_handler.hpp
  src/internal/display_handler.cpp
  src/internal/image_tester.cpp
  include/rviz_visual_tests/internal/image_tester.hpp
  include/rviz_visual_tests/internal/test_helpers.hpp
  src/internal/test_helpers.cpp
  include/rviz_visual_tests/internal/executor.hpp
  src/internal/executor.cpp
  include/rviz_visual_tests/internal/visual_test.hpp
  src/internal/visual_test.cpp
  include/rviz_visual_tests/visual_test_fixture.hpp
  src/visual_test_fixture.cpp)

set(visual_tests_target_libaries
  rviz_common::rviz_common
  Qt5::Widgets
  Qt5::Test)

add_library(rviz_visual_tests STATIC ${visual_test_framework_source_files})

target_include_directories(rviz_visual_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(rviz_visual_tests ${visual_tests_target_libaries} ${GTEST_LIBRARIES})

# export information to downstream packages
ament_export_interfaces(rviz_visual_tests)
ament_export_dependencies(Qt5)
ament_export_dependencies(rviz_common)

install(
  TARGETS rviz_visual_tests
  EXPORT rviz_visual_tests
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

  if(EnableVisualTests STREQUAL "TRUE")
    ament_add_gtest(
      example_test
      tests/example_test.cpp
      ${visual_test_framework_source_files})
    if(TARGET example_test)
      target_link_libraries(
        example_test
        ${visual_tests_target_libaries})
    endif()
  endif()
endif()

ament_package()
