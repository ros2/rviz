cmake_minimum_required(VERSION 3.5)

project(rviz2)

# TODO(wjwood): remove when windows build is fixed
if(DEFINED ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE})
  set(RVIZ_BUILD_ON_WINDOWS_OVERRIDE "$ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE}")
endif()
if(WIN32 AND NOT DEFINED RVIZ_BUILD_ON_WINDOWS_OVERRIDE)
  message(STATUS "${PROJECT_NAME} is not supported on Windows, skipping...")
  return()
endif()

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

find_package(rviz_common REQUIRED)

find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(${PROJECT_NAME}_app
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_app
  rviz_common::rviz_common
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}_app DESTINATION bin)

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

  add_subdirectory(test)
endif()

set_target_properties(${PROJECT_NAME}_app PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

ament_package()
