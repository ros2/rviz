cmake_minimum_required(VERSION 3.10)

project(rviz2)

# The C++ standard (C++20) is provided by ament_cmake_ros_core via the
# ament_cmake_ros_core::ament_ros_defaults interface target, which is linked
# into the rviz2 target below.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

find_package(rviz_common REQUIRED)

find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt6 6.4 REQUIRED COMPONENTS Widgets)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
  ament_cmake_ros_core::ament_ros_defaults
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt6::Widgets
)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/rviz1_to_rviz2.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

ament_package()
