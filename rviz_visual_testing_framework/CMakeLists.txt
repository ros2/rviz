cmake_minimum_required(VERSION 3.10)

project(rviz_visual_testing_framework)

# Options to decide whether or not the visual test will run.
option(EnableVisualTests "decides whether or not enable the tests")

# Path variables needed to save and find screenshots for image comparison.
add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# Creates a directory where the test and reference screenshots will be saved.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# The C++ standard (C++20) is provided by ament_cmake_ros_core via the
# ament_cmake_ros_core::ament_ros_defaults interface target, which is linked
# PUBLIC into the rviz_visual_testing_framework target below (and thus
# propagated to consumers).

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt6 6.4 REQUIRED COMPONENTS Test Widgets)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(ament_cmake_gtest REQUIRED)
ament_find_gtest()

# TODO(wjwwood): this block is to setup the windeployqt tool, could be removed later.
if(Qt6_FOUND AND WIN32 AND TARGET Qt6::qmake AND NOT TARGET Qt6::windeployqt)
  get_target_property(_qt_qmake_location Qt6::qmake IMPORTED_LOCATION)

  execute_process(
    COMMAND "${_qt_qmake_location}" -query QT6_INSTALL_PREFIX
    RESULT_VARIABLE return_code
    OUTPUT_VARIABLE qt_install_prefix
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
endif()

add_library(rviz_visual_testing_framework STATIC
  src/page_objects/base_page_object.cpp
  src/page_objects/page_object_with_window.cpp
  src/internal/rviz_scene_test.cpp
  src/internal/display_handler.cpp
  src/internal/image_tester.cpp
  src/test_helpers.cpp
  src/internal/executor.cpp
  src/internal/transform_message_creator.cpp
  src/internal/visual_test.cpp
  src/transform_publisher.cpp
  src/visual_test_fixture.cpp
  src/visual_test_publisher.cpp
)

target_include_directories(rviz_visual_testing_framework
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(rviz_visual_testing_framework PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  geometry_msgs::geometry_msgs
  Qt6::Test
  Qt6::Widgets
  rclcpp::rclcpp
  rcutils::rcutils
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
  std_msgs::std_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  GTest::gtest
)

# export information to downstream packages
ament_export_dependencies(
  ament_cmake_ros_core
  geometry_msgs
  Qt6
  rclcpp
  rcutils
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  std_msgs
  tf2
  tf2_ros
)

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
