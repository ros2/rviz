cmake_minimum_required(VERSION 3.5)

project(rviz_visual_testing_framework)

# Options to decide whether or not the visual test will run.
option(EnableVisualTests "decides whether or not enable the tests")

# Path variables needed to save and find screenshots for image comparison.
add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Creates a directory where the test and reference screenshots will be saved.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

find_package(rcutils REQUIRED)

find_package(rviz_common REQUIRED)

find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(ament_cmake_gtest REQUIRED)
ament_find_gtest()

set(visual_test_framework_source_files
  include/rviz_visual_testing_framework/page_objects/base_page_object.hpp
  src/page_objects/base_page_object.cpp
  include/rviz_visual_testing_framework/page_objects/page_object_with_window.hpp
  src/page_objects/page_object_with_window.cpp
  src/internal/rviz_scene_test.cpp
  include/rviz_visual_testing_framework/internal/rviz_scene_test.hpp
  include/rviz_visual_testing_framework/internal/display_handler.hpp
  src/internal/display_handler.cpp
  src/internal/image_tester.cpp
  include/rviz_visual_testing_framework/internal/image_tester.hpp
  include/rviz_visual_testing_framework/test_helpers.hpp
  src/test_helpers.cpp
  include/rviz_visual_testing_framework/internal/executor.hpp
  src/internal/executor.cpp
  include/rviz_visual_testing_framework/internal/transform_message_creator.hpp
  src/internal/transform_message_creator.cpp
  include/rviz_visual_testing_framework/internal/visual_test.hpp
  src/internal/visual_test.cpp
  include/rviz_visual_testing_framework/visual_test_fixture.hpp
  src/visual_test_fixture.cpp
  include/rviz_visual_testing_framework/visual_test_publisher.hpp
)

set(visual_tests_target_libaries
  rviz_common::rviz_common
  Qt5::Widgets
  Qt5::Test)

# TODO(wjwwood): this block is to setup the windeployqt tool, could be removed later.
if(Qt5_FOUND AND WIN32 AND TARGET Qt5::qmake AND NOT TARGET Qt5::windeployqt)
  get_target_property(_qt5_qmake_location Qt5::qmake IMPORTED_LOCATION)

  execute_process(
    COMMAND "${_qt5_qmake_location}" -query QT_INSTALL_PREFIX
    RESULT_VARIABLE return_code
    OUTPUT_VARIABLE qt5_install_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()

add_library(rviz_visual_testing_framework STATIC ${visual_test_framework_source_files})

target_include_directories(rviz_visual_testing_framework
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${GTEST_INCLUDE_DIRS})

target_link_libraries(rviz_visual_testing_framework
  ${visual_tests_target_libaries})

# export information to downstream packages
ament_export_dependencies(Qt5)
ament_export_dependencies(rviz_common)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz_visual_testing_framework)

install(
  TARGETS rviz_visual_testing_framework
  EXPORT rviz_visual_testing_framework
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
