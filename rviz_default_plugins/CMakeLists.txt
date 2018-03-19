cmake_minimum_required(VERSION 3.5)

project(rviz_default_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(resource_retriever REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

# These need to be added in the add_library() call so AUTOMOC detects them.
set(rviz_default_plugins_headers_to_moc
  src/rviz_default_plugins/displays/camera/camera_display.hpp
  src/rviz_default_plugins/displays/grid/grid_display.hpp
  src/rviz_default_plugins/displays/image/image_display.hpp
  src/rviz_default_plugins/displays/marker/marker_display.hpp
  src/rviz_default_plugins/displays/path/path_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp
  src/rviz_default_plugins/displays/polygon/polygon_display.hpp
  src/rviz_default_plugins/displays/pose/pose_display.hpp
  src/rviz_default_plugins/displays/pose_array/pose_array_display.hpp
  src/rviz_default_plugins/displays/robot_model/robot_model_display.hpp
  src/rviz_default_plugins/displays/tf/frame_info.hpp
  src/rviz_default_plugins/displays/tf/tf_display.hpp
  src/rviz_default_plugins/robot/robot.hpp
  src/rviz_default_plugins/robot/robot_joint.hpp
  src/rviz_default_plugins/robot/robot_link.hpp
  src/rviz_default_plugins/robot/robot_element_base_class.hpp
)

set(rviz_default_plugins_source_files
  src/rviz_default_plugins/displays/camera/camera_display.cpp
  src/rviz_default_plugins/displays/grid/grid_display.cpp
  src/rviz_default_plugins/displays/image/image_display.cpp
  src/rviz_default_plugins/displays/image/ros_image_texture.cpp
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
  src/rviz_default_plugins/displays/marker/marker_display.cpp
  src/rviz_default_plugins/displays/path/path_display.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_to_point_cloud2.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud_display.cpp
  src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.cpp
  src/rviz_default_plugins/displays/polygon/polygon_display.cpp
  src/rviz_default_plugins/displays/pose/pose_display.cpp
  src/rviz_default_plugins/displays/pose/pose_display_selection_handler.cpp
  src/rviz_default_plugins/displays/pose_array/pose_array_display.cpp
  src/rviz_default_plugins/displays/pose_array/flat_arrows_array.cpp
  src/rviz_default_plugins/displays/robot_model/robot_model_display.cpp
  src/rviz_default_plugins/displays/tf/frame_info.cpp
  src/rviz_default_plugins/displays/tf/frame_selection_handler.cpp
  src/rviz_default_plugins/displays/tf/tf_display.cpp
  src/rviz_default_plugins/robot/robot.cpp
  src/rviz_default_plugins/robot/robot_joint.cpp
  src/rviz_default_plugins/robot/robot_link.cpp
  src/rviz_default_plugins/robot/robot_element_base_class.cpp
  src/rviz_default_plugins/robot/tf_link_updater.cpp
  src/rviz_default_plugins/tools/move/move_tool.cpp
  src/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.cpp
)

add_library(rviz_default_plugins SHARED
  ${rviz_default_plugins_headers_to_moc}
  ${rviz_default_plugins_source_files}
)
target_include_directories(rviz_default_plugins
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include>
)
target_link_libraries(rviz_default_plugins
  rviz_rendering::rviz_rendering
  rviz_common::rviz_common
  resource_retriever::resource_retriever
)
ament_target_dependencies(rviz_default_plugins
  # geometry_msgs
  nav_msgs
  rclcpp
  resource_retriever
  urdf
  visualization_msgs
  # tf2
  # tf2_geometry_msgs
  # tf2_ros
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# prevent pluginlib from using boost
target_compile_definitions(rviz_default_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_default_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

ament_export_interfaces(rviz_default_plugins)
ament_export_dependencies(Qt5)
ament_export_dependencies(rviz_rendering)
ament_export_dependencies(
  # geometry_msgs
  rclcpp
  # tf2
  # tf2_geometry_msgs
  # tf2_ros
)

install(
  TARGETS rviz_default_plugins
  EXPORT rviz_default_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gtest(point_cloud2_display_test
#      test/rviz_default_plugins/displays/pointcloud/message_creators.hpp
#      test/rviz_default_plugins/displays/pointcloud/message_creators.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud2_display_test.cpp
#      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET point_cloud2_display_test)
#    target_link_libraries(point_cloud2_display_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gtest(point_cloud_transformers_test
#      test/rviz_default_plugins/displays/pointcloud/message_creators.hpp
#      test/rviz_default_plugins/displays/pointcloud/message_creators.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/axis_color_pc_transformer_test.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/flat_color_pc_transformer_test.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/intensity_pc_transformer_test.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgb8_pc_transformer_test.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/rgbf32_pc_transformer_test.cpp
#      test/rviz_default_plugins/displays/pointcloud/point_cloud_transformers/xyz_pc_transformer_test.cpp
#      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET point_cloud_transformers_test)
#    target_link_libraries(point_cloud_transformers_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(ros_image_texture_test
#      test/rviz_default_plugins/displays/image/ros_image_texture_test.cpp
#      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET ros_image_texture_test)
#    target_link_libraries(ros_image_texture_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(image_display_test
#      test/rviz_default_plugins/displays/image/image_display_test.cpp
#      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET image_display_test)
#    target_link_libraries(image_display_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(robot_test
#      test/rviz_default_plugins/robot/robot_test.cpp
#      APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET robot_test)
#    target_link_libraries(robot_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(marker_test
#    test/rviz_default_plugins/displays/marker/markers/arrow_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/line_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/mesh_resource_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/points_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/shape_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/text_view_facing_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/triangle_list_marker_test.cpp
#    test/rviz_default_plugins/displays/marker/markers/markers_test_fixture.cpp
#    test/rviz_default_plugins/displays/display_test_fixture.cpp
#    test/rviz_default_plugins/displays/marker/marker_messages.cpp
#    test/rviz_default_plugins/scene_graph_introspection.cpp
#    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;
#    ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET marker_test)
#    target_link_libraries(marker_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(marker_display_test
#    test/rviz_default_plugins/displays/marker/marker_display_test.cpp
#    test/rviz_default_plugins/displays/display_test_fixture.cpp
#    test/rviz_default_plugins/displays/marker/marker_messages.cpp
#    test/rviz_default_plugins/scene_graph_introspection.cpp
#    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;
#    ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET marker_display_test)
#    target_link_libraries(marker_display_test rviz_default_plugins)
#  endif()

# TODO(greimela): reenable these tests once they can be run on the ci
#  ament_add_gmock(pose_array_display_test
#    test/rviz_default_plugins/displays/pose_array/pose_array_display_test.cpp
#    test/rviz_default_plugins/displays/display_test_fixture.cpp
#    test/rviz_default_plugins/scene_graph_introspection.cpp
#    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX} PATH=${CMAKE_INSTALL_PREFIX}/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor/bin;${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor/bin;
#    ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor/bin)
#  if(TARGET pose_array_display_test)
#    target_link_libraries(pose_array_display_test rviz_default_plugins)
#  endif()
endif()

ament_package()
