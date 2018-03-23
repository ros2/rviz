cmake_minimum_required(VERSION 3.5)

project(rviz_visual_tests)

# Options to decide whether or not the visual test will run and in which modality (TEST or REF)
option(EnableVisualTests "decides whether or not enable the tests")
option(GenerateReferenceImages "decides whether the reference images are updated")

# Option to set a non null threshold in the image comparison (in case of unidentical images).
option(MseThreshold "The MSE index threshold")
set(MSEThreshold "0.0005" CACHE STRING "default value of the threshold")
add_definitions(-D_MSE_THRESHOLD="${MSEThreshold}")

# Path variables needed to save and find screenshots for image comparison.
add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Creates a directory where the test screenshots will be saved.
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
        src/page_objects/base_page_object.hpp
        src/page_objects/base_page_object.cpp
        src/page_objects/camera_display_page_object.hpp
        src/page_objects/camera_display_page_object.cpp
        src/page_objects/image_display_page_object.hpp
        src/page_objects/image_display_page_object.cpp
        src/page_objects/page_object_with_window.hpp
        src/page_objects/page_object_with_window.cpp
        src/page_objects/point_cloud_display_page_object.hpp
        src/page_objects/point_cloud_display_page_object.cpp
        src/page_objects/grid_display_page_object.hpp
        src/page_objects/grid_display_page_object.cpp
        src/page_objects/polygon_display_page_object.hpp
        src/page_objects/polygon_display_page_object.cpp
        src/internal/rviz_scene_test.cpp
        src/internal/rviz_scene_test.hpp
        src/internal/display_handler.hpp
        src/internal/display_handler.cpp
        src/internal/image_tester.cpp
        src/internal/image_tester.hpp
        src/internal/test_helpers.hpp
        src/internal/test_helpers.cpp
        src/internal/executor.hpp
        src/internal/executor.cpp
        src/internal/visual_test.hpp
        src/internal/visual_test.cpp
        src/visual_test_fixture.hpp
        src/visual_test_fixture.cpp)
      if(TARGET example_test)
        target_link_libraries(
          example_test
          rviz_common::rviz_common
          Qt5::Widgets
          Qt5::Test)
      endif()
    endif()
endif()

ament_package()
