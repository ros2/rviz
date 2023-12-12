cmake_minimum_required(VERSION 3.5)

project(rviz_rendering_tests)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
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

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  # do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
  find_package(rviz_ogre_vendor REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  find_package(rviz_rendering REQUIRED)
  find_package(resource_retriever REQUIRED)

  # Adds shallow folders containing ogre_media such as meshes and scripts to Rviz
  # so that they can be found at runtime. Requires rviz_rendering.
  register_rviz_ogre_media_exports(
    DIRECTORIES
    "ogre_media_resources/scripts"
    "ogre_media_resources/meshes"
  )

  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ogre_media_resources/test_meshes"
    DESTINATION "share/rviz_rendering_tests"
    USE_SOURCE_PERMISSIONS)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  ament_add_gmock(mesh_loader_test_target
    test/mesh_loader_test.cpp
    test/ogre_testing_environment.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET mesh_loader_test_target)
    target_link_libraries(mesh_loader_test_target
      rviz_ogre_vendor::OgreMain
      rviz_rendering::rviz_rendering
      resource_retriever::resource_retriever
    )
  endif()

  ament_add_gtest(test_rviz_rendering_tests
    test/test_rviz_ogre_media_exports.cpp
    ${SKIP_DISPLAY_TESTS})
  if(TARGET test_rviz_rendering_tests)
    target_include_directories(test_rviz_rendering_tests
      PUBLIC src/rviz_rendering_tests)
    target_link_libraries(test_rviz_rendering_tests
      ament_index_cpp::ament_index_cpp)
  endif()
endif()

ament_package()
