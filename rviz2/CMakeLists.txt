cmake_minimum_required(VERSION 3.10)

project(rviz2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)

find_package(rviz_ogre_vendor REQUIRED)

# Avoid find_package(QT NAMES Qt6 Qt5 ...) due to CMake's default ascending path resolution
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 5)
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt${QT_VERSION_MAJOR}::Widgets
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
