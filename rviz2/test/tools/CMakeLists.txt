find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(send_lots_of_points_node send_lots_of_points_node.cpp)
target_link_libraries(send_lots_of_points_node PRIVATE
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

get_filename_component(script_path "${PROJECT_SOURCE_DIR}/scripts/rviz1_to_rviz2.py" ABSOLUTE)
get_filename_component(config_path "${CMAKE_CURRENT_SOURCE_DIR}/configs" ABSOLUTE BASE_DIR)
ament_add_pytest_test(rviz1_to_2_check rviz1_to_2_check.py
  ENV "_SCRIPT_PATH=${script_path}"
  ENV "_CONFIG_PATH=${config_path}"
  TIMEOUT 10
)
