cmake_minimum_required(VERSION 3.5)

project(rviz_default_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)

find_package(ignition-math6 REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

set(rviz_default_plugins_headers_to_moc
  include/rviz_default_plugins/displays/accel/accel_display.hpp
  include/rviz_default_plugins/displays/axes/axes_display.hpp
  include/rviz_default_plugins/displays/camera/camera_display.hpp
  include/rviz_default_plugins/displays/depth_cloud/depth_cloud_display.hpp
  include/rviz_default_plugins/displays/effort/effort_display.hpp
  include/rviz_default_plugins/displays/fluid_pressure/fluid_pressure_display.hpp
  include/rviz_default_plugins/displays/grid/grid_display.hpp
  include/rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp
  include/rviz_default_plugins/displays/illuminance/illuminance_display.hpp
  include/rviz_default_plugins/displays/image/image_display.hpp
  include/rviz_default_plugins/displays/laser_scan/laser_scan_display.hpp
  include/rviz_default_plugins/displays/map/map_display.hpp
  include/rviz_default_plugins/displays/marker/marker_common.hpp
  include/rviz_default_plugins/displays/odometry/odometry_display.hpp
  include/rviz_default_plugins/transformation/transformer_guard.hpp
  include/rviz_default_plugins/displays/interactive_markers/integer_action.hpp
  include/rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp
  include/rviz_default_plugins/displays/interactive_markers/interactive_marker_display.hpp
  include/rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property.hpp
  include/rviz_default_plugins/displays/path/path_display.hpp
  include/rviz_default_plugins/displays/point/point_stamped_display.hpp
  include/rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp
  include/rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp
  include/rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp
  include/rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp
  include/rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp
  include/rviz_default_plugins/displays/polygon/polygon_display.hpp
  include/rviz_default_plugins/displays/pose/pose_display.hpp
  include/rviz_default_plugins/displays/pose_array/pose_array_display.hpp
  include/rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp
  include/rviz_default_plugins/displays/range/range_display.hpp
  include/rviz_default_plugins/displays/relative_humidity/relative_humidity_display.hpp
  include/rviz_default_plugins/displays/robot_model/robot_model_display.hpp
  include/rviz_default_plugins/displays/temperature/temperature_display.hpp
  include/rviz_default_plugins/displays/tf/frame_info.hpp
  include/rviz_default_plugins/displays/tf/tf_display.hpp
  include/rviz_default_plugins/displays/twist/twist_display.hpp
  include/rviz_default_plugins/displays/wrench/wrench_display.hpp
  include/rviz_default_plugins/robot/robot.hpp
  include/rviz_default_plugins/robot/robot_joint.hpp
  include/rviz_default_plugins/robot/robot_link.hpp
  include/rviz_default_plugins/robot/robot_element_base_class.hpp
  include/rviz_default_plugins/tools/measure/measure_tool.hpp
  include/rviz_default_plugins/tools/goal_pose/goal_tool.hpp
  include/rviz_default_plugins/tools/interaction/interaction_tool.hpp
  include/rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp
  include/rviz_default_plugins/tools/point/point_tool.hpp
  include/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp
  include/rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp
  include/rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp
)

foreach(header "${rviz_default_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz_default_plugins_moc_files "${header}")
endforeach()

set(rviz_default_plugins_source_files
  src/rviz_default_plugins/displays/accel/accel_display.cpp
  src/rviz_default_plugins/displays/axes/axes_display.cpp
  src/rviz_default_plugins/displays/camera/camera_display.cpp
  src/rviz_default_plugins/displays/depth_cloud/depth_cloud_display.cpp
  src/rviz_default_plugins/displays/effort/effort_display.cpp
  src/rviz_default_plugins/displays/grid/grid_display.cpp
  src/rviz_default_plugins/displays/grid_cells/grid_cells_display.cpp
  src/rviz_default_plugins/displays/fluid_pressure/fluid_pressure_display.cpp
  src/rviz_default_plugins/displays/illuminance/illuminance_display.cpp
  src/rviz_default_plugins/displays/image/get_transport_from_topic.cpp
  src/rviz_default_plugins/displays/image/image_display.cpp
  src/rviz_default_plugins/displays/image/ros_image_texture.cpp
  src/rviz_default_plugins/displays/interactive_markers/integer_action.cpp
  src/rviz_default_plugins/displays/interactive_markers/interactive_marker_control.cpp
  src/rviz_default_plugins/displays/interactive_markers/interactive_marker.cpp
  src/rviz_default_plugins/displays/interactive_markers/interactive_marker_display.cpp
  src/rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property.cpp
  src/rviz_default_plugins/displays/laser_scan/laser_scan_display.cpp
  src/rviz_default_plugins/displays/map/map_display.cpp
  src/rviz_default_plugins/displays/map/palette_builder.cpp
  src/rviz_default_plugins/displays/map/swatch.cpp
  src/rviz_default_plugins/displays/marker/markers/arrow_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/line_list_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/line_marker_base.cpp
  src/rviz_default_plugins/displays/marker/markers/line_strip_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/marker_base.cpp
  src/rviz_default_plugins/displays/marker/markers/marker_selection_handler.cpp
  src/rviz_default_plugins/displays/marker/markers/mesh_resource_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/text_view_facing_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/points_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/shape_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/triangle_list_marker.cpp
  src/rviz_default_plugins/displays/marker/markers/marker_factory.cpp
  src/rviz_default_plugins/displays/marker/marker_common.cpp
  src/rviz_default_plugins/displays/marker/marker_display.cpp
  src/rviz_default_plugins/displays/marker_array/marker_array_display.cpp
  src/rviz_default_plugins/displays/odometry/odometry_display.cpp
  src/rviz_default_plugins/displays/path/path_display.cpp
  src/rviz_default_plugins/displays/point/point_stamped_display.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformer_factory.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_display.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.cpp
  src/rviz_default_plugins/displays/polygon/polygon_display.cpp
  src/rviz_default_plugins/displays/pose/pose_display.cpp
  src/rviz_default_plugins/displays/pose/pose_display_selection_handler.cpp
  src/rviz_default_plugins/displays/pose_array/pose_array_display.cpp
  src/rviz_default_plugins/displays/pose_array/flat_arrows_array.cpp
  src/rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.cpp
  src/rviz_default_plugins/displays/pose_covariance/pose_with_cov_selection_handler.cpp
  src/rviz_default_plugins/displays/range/range_display.cpp
  src/rviz_default_plugins/displays/relative_humidity/relative_humidity_display.cpp
  src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp
  src/rviz_default_plugins/displays/screw/screw_display.cpp
  src/rviz_default_plugins/displays/temperature/temperature_display.cpp
  src/rviz_default_plugins/displays/tf/frame_info.cpp
  src/rviz_default_plugins/displays/tf/frame_selection_handler.cpp
  src/rviz_default_plugins/displays/tf/tf_display.cpp
  src/rviz_default_plugins/displays/wrench/wrench_display.cpp
  src/rviz_default_plugins/displays/twist/twist_display.cpp
  src/rviz_default_plugins/robot/robot.cpp
  src/rviz_default_plugins/robot/robot_joint.cpp
  src/rviz_default_plugins/robot/robot_link.cpp
  src/rviz_default_plugins/robot/robot_element_base_class.cpp
  src/rviz_default_plugins/robot/tf_link_updater.cpp
  src/rviz_default_plugins/tools/interaction/interaction_tool.cpp
  src/rviz_default_plugins/tools/measure/measure_tool.cpp
  src/rviz_default_plugins/tools/focus/focus_tool.cpp
  src/rviz_default_plugins/tools/move/move_tool.cpp
  src/rviz_default_plugins/tools/goal_pose/goal_tool.cpp
  src/rviz_default_plugins/tools/pose/pose_tool.cpp
  src/rviz_default_plugins/tools/pose_estimate/initial_pose_tool.cpp
  src/rviz_default_plugins/tools/point/point_tool.cpp
  src/rviz_default_plugins/tools/select/selection_tool.cpp
  src/rviz_default_plugins/transformation/tf_frame_transformer.cpp
  src/rviz_default_plugins/transformation/tf_wrapper.cpp
  src/rviz_default_plugins/view_controllers/follower/third_person_follower_view_controller.cpp
  src/rviz_default_plugins/view_controllers/fps/fps_view_controller.cpp
  src/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.cpp
  src/rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.cpp
  src/rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.cpp
)

add_library(rviz_default_plugins SHARED
  ${rviz_default_plugins_moc_files}
  ${rviz_default_plugins_source_files}
)

target_include_directories(rviz_default_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz_default_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_link_libraries(rviz_default_plugins PRIVATE
  ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_default_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_default_plugins
  PUBLIC
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_rendering
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz_default_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  rclcpp
  resource_retriever
  rviz_common
  rviz_ogre_vendor
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

install(
  TARGETS rviz_default_plugins
  EXPORT rviz_default_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(rviz_visual_testing_framework REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)

  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/ogre_media_resources/test_meshes"
    DESTINATION "share/rviz_default_plugins"
    USE_SOURCE_PERMISSIONS)

  set(TEST_INCLUDE_DIRS
    ${OGRE_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
  )
  ament_include_directories_order(TEST_INCLUDE_DIRS ${TEST_INCLUDE_DIRS})

  set(TEST_TARGET_DEPENDENCIES
    map_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    urdf
    visualization_msgs
  )

  # display_test_fixture.cpp is a piece of code that is used by every test.
  # Instead of recompiling it for every test, just compile it once as a library
  # and reuse it.  Note that we need to call ament_find_gmock() by hand here to
  # find GMOCK_INCLUDE_DIRS and GMOCK_LIBRARIES so we can compile this file.
  ament_find_gmock()
  add_library(display_test_fixture
    test/rviz_default_plugins/displays/display_test_fixture.cpp
    test/rviz_default_plugins/ogre_testing_environment.cpp
    test/rviz_default_plugins/scene_graph_introspection.cpp
  )
  target_include_directories(display_test_fixture PRIVATE ${GMOCK_INCLUDE_DIRS} ${TEST_INCLUDE_DIRS})
  target_link_libraries(display_test_fixture ${GMOCK_LIBRARIES})
  ament_target_dependencies(display_test_fixture ${TEST_TARGET_DEPENDENCIES} rviz_rendering rviz_common)

  set(TEST_LINK_LIBRARIES
    display_test_fixture
    rviz_default_plugins
    Qt5::Widgets
  )

  # This code is used by many tests, so compile it just once
  add_library(pointcloud_messages
    test/rviz_default_plugins/pointcloud_messages.cpp)
  target_include_directories(pointcloud_messages PUBLIC test)
  target_link_libraries(pointcloud_messages PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  # This code is used by many tests, so compile it just once
  add_library(point_cloud_common_page_object
    test/rviz_default_plugins/page_objects/point_cloud_common_page_object.cpp)
  target_include_directories(point_cloud_common_page_object PUBLIC
    test/page_objects
    ${TEST_INCLUDE_DIRS})
  target_link_libraries(point_cloud_common_page_object PUBLIC
    rviz_visual_testing_framework::rviz_visual_testing_framework)

  ament_add_gmock(fps_view_controller_test
    test/rviz_default_plugins/view_controllers/fps/fps_view_controller_test.cpp
    test/rviz_default_plugins/view_controllers/view_controller_test_fixture.hpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET fps_view_controller_test)
    target_include_directories(fps_view_controller_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(fps_view_controller_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(fps_view_controller_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(frame_info_test
    test/rviz_default_plugins/displays/tf/frame_info_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET frame_info_test)
    target_include_directories(frame_info_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(frame_info_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(frame_info_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(get_transport_from_topic_test
    test/rviz_default_plugins/displays/image/get_transport_from_topic_test.cpp)
  if(TARGET get_transport_from_topic_test)
    target_include_directories(get_transport_from_topic_test PUBLIC ${TEST_INCLUDE_DIRS} ${rviz_common_INCLUDE_DIRS})
    target_link_libraries(get_transport_from_topic_test ${TEST_LINK_LIBRARIES} ${rviz_common})
  endif()

  ament_add_gmock(grid_cells_display_test
    test/rviz_default_plugins/displays/grid_cells/grid_cells_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET grid_cells_display_test)
    target_include_directories(grid_cells_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(grid_cells_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(grid_cells_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(image_display_test
    test/rviz_default_plugins/displays/image/image_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET image_display_test)
    target_include_directories(image_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(image_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(image_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(marker_test
    test/rviz_default_plugins/displays/marker/markers/arrow_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/line_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/mesh_resource_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/points_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/shape_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/text_view_facing_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/triangle_list_marker_test.cpp
    test/rviz_default_plugins/displays/marker/markers/markers_test_fixture.cpp
    test/rviz_default_plugins/displays/marker/marker_messages.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET marker_test)
    target_include_directories(marker_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(marker_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(marker_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(marker_common_test
    test/rviz_default_plugins/displays/marker/marker_common_test.cpp
    test/rviz_default_plugins/displays/marker/marker_messages.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET marker_common_test)
    target_include_directories(marker_common_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(marker_common_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(marker_common_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(map_display_test
    test/rviz_default_plugins/displays/map/map_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET map_display_test)
    target_include_directories(map_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(map_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(map_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(measure_tool_test
    test/rviz_default_plugins/tools/measure/measure_tool_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET measure_tool_test)
    target_include_directories(measure_tool_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(measure_tool_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(measure_tool_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(odometry_display_test
    test/rviz_default_plugins/displays/odometry/odometry_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET odometry_display_test)
    target_include_directories(odometry_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(odometry_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(odometry_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(odometry_ogre_helper_test
    test/rviz_default_plugins/displays/odometry/quaternion_helper_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET odometry_ogre_helper_test)
    target_include_directories(odometry_ogre_helper_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(odometry_ogre_helper_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(odometry_ogre_helper_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(orbit_view_controller_test
    test/rviz_default_plugins/view_controllers/orbit/orbit_view_controller_test.cpp
    test/rviz_default_plugins/view_controllers/view_controller_test_fixture.hpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET orbit_view_controller_test)
    target_include_directories(orbit_view_controller_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(orbit_view_controller_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(orbit_view_controller_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(ortho_view_controller_test
    test/rviz_default_plugins/view_controllers/ortho/ortho_view_controller_test.cpp
    test/rviz_default_plugins/view_controllers/view_controller_test_fixture.hpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET ortho_view_controller_test)
    target_include_directories(ortho_view_controller_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(ortho_view_controller_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(ortho_view_controller_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(palette_builder_test
    test/rviz_default_plugins/displays/map/palette_builder_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET palette_builder_test)
    target_include_directories(palette_builder_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(palette_builder_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(palette_builder_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(path_display_test
    test/rviz_default_plugins/displays/path/path_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET path_display_test)
    target_include_directories(path_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(path_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(path_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(point_cloud2_display_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud2_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud2_display_test)
    target_include_directories(point_cloud2_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(point_cloud2_display_test
      ${TEST_LINK_LIBRARIES}
      pointcloud_messages)
    ament_target_dependencies(point_cloud2_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(point_cloud_common_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud_common_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud_common_test)
    target_include_directories(point_cloud_common_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(point_cloud_common_test
      ${TEST_LINK_LIBRARIES}
      pointcloud_messages)
    ament_target_dependencies(point_cloud_common_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(point_cloud_scalar_display_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud_scalar_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud_scalar_display_test)
    target_include_directories(point_cloud_scalar_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(point_cloud_scalar_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(point_cloud_scalar_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(point_cloud_transformers_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/axis_color_pc_transformer_test.cpp
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/flat_color_pc_transformer_test.cpp
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/intensity_pc_transformer_test.cpp
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgb8_pc_transformer_test.cpp
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgbf32_pc_transformer_test.cpp
    test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/xyz_pc_transformer_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_cloud_transformers_test)
    target_include_directories(point_cloud_transformers_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(point_cloud_transformers_test
      ${TEST_LINK_LIBRARIES}
      pointcloud_messages)
    ament_target_dependencies(point_cloud_transformers_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(point_display_test
    test/rviz_default_plugins/displays/point/point_stamped_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET point_display_test)
    target_include_directories(point_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(point_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(point_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(pose_array_display_test
    test/rviz_default_plugins/displays/pose_array/pose_array_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET pose_array_display_test)
    target_include_directories(pose_array_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(pose_array_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(pose_array_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(pose_tool_test
    test/rviz_default_plugins/tools/pose/pose_tool_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET pose_tool_test)
    target_include_directories(pose_tool_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(pose_tool_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(pose_tool_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(range_display_test
    test/rviz_default_plugins/displays/range/range_display_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET range_display_test)
    target_include_directories(range_display_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(range_display_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(range_display_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(robot_test
    test/rviz_default_plugins/robot/robot_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET robot_test)
    target_include_directories(robot_test PUBLIC ${TEST_INCLUDE_DIRS})
    # TODO(clalancette): Figure out why putting resource_retriever in the
    # TEST_TARGET_DEPENDENCIES doesn't work.
    target_link_libraries(robot_test resource_retriever::resource_retriever ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(robot_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(ros_image_texture_test
    test/rviz_default_plugins/displays/image/ros_image_texture_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET ros_image_texture_test)
    target_include_directories(ros_image_texture_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(ros_image_texture_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(ros_image_texture_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(selection_tool_test
    test/rviz_default_plugins/tools/select/selection_tool_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET selection_tool_test)
    target_include_directories(selection_tool_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(selection_tool_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(selection_tool_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(xy_orbit_view_controller_test
    test/rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller_test.cpp
    test/rviz_default_plugins/view_controllers/view_controller_test_fixture.hpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET xy_orbit_view_controller_test)
    target_include_directories(xy_orbit_view_controller_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(xy_orbit_view_controller_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(xy_orbit_view_controller_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(frame_transformer_tf_test
    test/rviz_default_plugins/transformation/frame_transformer_tf_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET frame_transformer_tf_test)
    target_include_directories(frame_transformer_tf_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(frame_transformer_tf_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(frame_transformer_tf_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gmock(transformer_guard_test
    test/rviz_default_plugins/transformation/transformer_guard_test.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET transformer_guard_test)
    target_include_directories(transformer_guard_test PUBLIC ${TEST_INCLUDE_DIRS})
    target_link_libraries(transformer_guard_test ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(transformer_guard_test ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gtest(accel_display_visual_test
    test/rviz_default_plugins/displays/accel/accel_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/accel_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET accel_display_visual_test)
    target_include_directories(accel_display_visual_test PRIVATE test)
    target_link_libraries(accel_display_visual_test
      rviz_visual_testing_framework::rviz_visual_testing_framework
      ${geometry_msgs_TARGETS}
      ${std_msgs_TARGETS}
      rclcpp::rclcpp
    )
  endif()

  ament_add_gtest(axes_display_visual_test
    test/rviz_default_plugins/displays/axes/axes_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/axes_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET axes_display_visual_test)
    target_include_directories(axes_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(axes_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(camera_display_visual_test
    test/rviz_default_plugins/displays/camera/camera_display_visual_test.cpp
    test/rviz_default_plugins/publishers/camera_info_publisher.hpp
    test/rviz_default_plugins/page_objects/camera_display_page_object.cpp
    test/rviz_default_plugins/publishers/image_publisher.hpp
    test/rviz_default_plugins/publishers/point_cloud_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET camera_display_visual_test)
    target_include_directories(camera_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(camera_display_visual_test
      point_cloud_common_page_object
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(depth_cloud_display_visual_test
    test/rviz_default_plugins/displays/depth_cloud/depth_cloud_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/depth_cloud_page_object.cpp
    test/rviz_default_plugins/publishers/camera_info_publisher.hpp
    test/rviz_default_plugins/publishers/image_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET depth_cloud_display_visual_test)
    target_include_directories(depth_cloud_display_visual_test PRIVATE test)
    target_link_libraries(depth_cloud_display_visual_test
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(effort_display_visual_test
    test/rviz_default_plugins/displays/effort/effort_display_visual_test.cpp
    test/rviz_default_plugins/publishers/effort_publisher.hpp
    test/rviz_default_plugins/page_objects/effort_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET effort_display_visual_test)
    target_include_directories(effort_display_visual_test PRIVATE test)
    target_link_libraries(effort_display_visual_test
      rviz_visual_testing_framework::rviz_visual_testing_framework
      rclcpp::rclcpp
      ${sensor_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(fluid_pressure_display_visual_test
    test/rviz_default_plugins/displays/fluid_pressure/fluid_pressure_display_visual_test.cpp
    test/rviz_default_plugins/publishers/fluid_pressure_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET fluid_pressure_display_visual_test)
    target_include_directories(fluid_pressure_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(fluid_pressure_display_visual_test
      point_cloud_common_page_object
      pointcloud_messages
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(grid_display_visual_test
    test/rviz_default_plugins/displays/grid/grid_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/grid_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET grid_display_visual_test)
    target_include_directories(grid_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(grid_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(grid_cells_display_visual_test
    test/rviz_default_plugins/displays/grid_cells/grid_cells_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/grid_cells_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET grid_cells_display_visual_test)
    target_include_directories(grid_cells_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(grid_cells_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(illuminance_display_visual_test
    test/rviz_default_plugins/displays/illuminance/illuminance_display_visual_test.cpp
    test/rviz_default_plugins/publishers/illuminance_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET illuminance_display_visual_test)
    target_include_directories(illuminance_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(illuminance_display_visual_test
      point_cloud_common_page_object
      pointcloud_messages
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(image_display_visual_test
    test/rviz_default_plugins/displays/image/image_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/image_display_page_object.cpp
    test/rviz_default_plugins/publishers/image_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET image_display_visual_test)
    target_include_directories(image_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(image_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(interactive_marker_namespace_property_test
    test/rviz_default_plugins/displays/interactive_markers/interactive_marker_namespace_property_test.cpp)
  if(TARGET interactive_marker_namespace_property_test)
    target_include_directories(interactive_marker_namespace_property_test PUBLIC
      ${TEST_INCLUDE_DIRS})
    target_link_libraries(interactive_marker_namespace_property_test
      ${TEST_LINK_LIBRARIES})
    ament_target_dependencies(interactive_marker_namespace_property_test
      ${TEST_TARGET_DEPENDENCIES})
  endif()

  ament_add_gtest(laser_scan_display_visual_test
    test/rviz_default_plugins/displays/laser_scan/laser_scan_display_visual_test.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET laser_scan_display_visual_test)
    target_include_directories(laser_scan_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(laser_scan_display_visual_test
      point_cloud_common_page_object
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(map_display_visual_test
    test/rviz_default_plugins/displays/map/map_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/map_display_page_object.cpp
    test/rviz_default_plugins/page_objects/marker_display_page_object.cpp
    test/rviz_default_plugins/publishers/map_publisher.hpp
    test/rviz_default_plugins/publishers/single_marker_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET map_display_visual_test)
    target_include_directories(map_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(map_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(marker_array_display_visual_test
    test/rviz_default_plugins/displays/marker_array/marker_array_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/marker_array_display_page_object.cpp
    test/rviz_default_plugins/page_objects/marker_display_page_object.cpp
    test/rviz_default_plugins/publishers/marker_array_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET marker_array_display_visual_test)
    target_include_directories(marker_array_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(marker_array_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(marker_display_visual_test
    test/rviz_default_plugins/displays/marker/marker_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/marker_display_page_object.cpp
    test/rviz_default_plugins/publishers/marker_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET marker_display_visual_test)
    target_include_directories(marker_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(marker_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(odometry_display_visual_test
    test/rviz_default_plugins/displays/odometry/odometry_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/odometry_display_page_object.cpp
    test/rviz_default_plugins/publishers/odometry_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET odometry_display_visual_test)
    target_include_directories(odometry_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(odometry_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(path_display_visual_test
    test/rviz_default_plugins/displays/path/path_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/path_display_page_object.cpp
    test/rviz_default_plugins/publishers/path_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET path_display_visual_test)
    target_include_directories(path_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(path_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(point_display_visual_test
    test/rviz_default_plugins/displays/point/point_stamped_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/point_display_page_object.cpp
    test/rviz_default_plugins/publishers/point_stamped_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET point_display_visual_test)
    target_include_directories(point_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(point_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(point_cloud_display_visual_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud_display_visual_test.cpp
    test/rviz_default_plugins/publishers/point_cloud_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET point_cloud_display_visual_test)
    target_include_directories(point_cloud_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(point_cloud_display_visual_test
      point_cloud_common_page_object
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(point_cloud2_display_visual_test
    test/rviz_default_plugins/displays/pointcloud/point_cloud2_display_visual_test.cpp
    test/rviz_default_plugins/publishers/point_cloud2_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET point_cloud2_display_visual_test)
    target_include_directories(point_cloud2_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(point_cloud2_display_visual_test
      point_cloud_common_page_object
      pointcloud_messages
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(pose_array_display_visual_test
    test/rviz_default_plugins/displays/pose_array/pose_array_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/pose_array_display_page_object.cpp
    test/rviz_default_plugins/page_objects/pose_display_page_object.cpp
    test/rviz_default_plugins/publishers/pose_array_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET pose_array_display_visual_test)
    target_include_directories(pose_array_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(pose_array_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(pose_display_visual_test
    test/rviz_default_plugins/displays/pose/pose_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/pose_display_page_object.cpp
    test/rviz_default_plugins/publishers/pose_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET pose_display_visual_test)
    target_include_directories(pose_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(pose_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(pose_with_covariance_display_visual_test
          test/rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display_visual_test.cpp
          test/rviz_default_plugins/page_objects/pose_with_covariance_display_page_object.cpp
          test/rviz_default_plugins/publishers/pose_with_covariance_publisher.hpp
          ${SKIP_VISUAL_TESTS}
          TIMEOUT 180)
  if(TARGET pose_with_covariance_display_visual_test)
    target_include_directories(pose_with_covariance_display_visual_test PUBLIC
            ${TEST_INCLUDE_DIRS}
            ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(pose_with_covariance_display_visual_test
            ${TEST_LINK_LIBRARIES}
            rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(range_display_visual_test
    test/rviz_default_plugins/displays/range/range_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/range_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET range_display_visual_test)
    target_include_directories(range_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(range_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(relative_humidity_display_visual_test
    test/rviz_default_plugins/displays/relative_humidity/relative_humidity_display_visual_test.cpp
    test/rviz_default_plugins/publishers/relative_humidity_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET relative_humidity_display_visual_test)
    target_include_directories(relative_humidity_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(relative_humidity_display_visual_test
      point_cloud_common_page_object
      pointcloud_messages
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(robot_model_display_visual_test
    test/rviz_default_plugins/displays/robot_model/robot_model_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/robot_model_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET robot_model_display_visual_test)
    target_include_directories(robot_model_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(robot_model_display_visual_test
      ${TEST_LINK_LIBRARIES}
      ament_index_cpp::ament_index_cpp
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(temperature_display_visual_test
    test/rviz_default_plugins/displays/temperature/temperature_display_visual_test.cpp
    test/rviz_default_plugins/publishers/temperature_publisher.hpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET temperature_display_visual_test)
    target_include_directories(temperature_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(temperature_display_visual_test
      point_cloud_common_page_object
      pointcloud_messages
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(tf_display_visual_test
    test/rviz_default_plugins/displays/tf/tf_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/tf_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET tf_display_visual_test)
    target_include_directories(tf_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(tf_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()

  ament_add_gtest(twist_display_visual_test
    test/rviz_default_plugins/displays/twist/twist_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/twist_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET twist_display_visual_test)
    target_include_directories(twist_display_visual_test PRIVATE test)
    target_link_libraries(twist_display_visual_test
      rviz_visual_testing_framework::rviz_visual_testing_framework
      ${geometry_msgs_TARGETS}
      ${std_msgs_TARGETS}
      rclcpp::rclcpp
    )
  endif()

  ament_add_gtest(wrench_display_visual_test
    test/rviz_default_plugins/displays/wrench/wrench_stamped_display_visual_test.cpp
    test/rviz_default_plugins/page_objects/wrench_display_page_object.cpp
    ${SKIP_VISUAL_TESTS}
    TIMEOUT 180)
  if(TARGET wrench_display_visual_test)
    target_include_directories(wrench_display_visual_test PUBLIC
      ${TEST_INCLUDE_DIRS}
      ${rviz_visual_testing_framework_INCLUDE_DIRS})
    target_link_libraries(wrench_display_visual_test
      ${TEST_LINK_LIBRARIES}
      rviz_visual_testing_framework::rviz_visual_testing_framework)
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_default_plugins-extras.cmake"
)

