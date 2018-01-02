cmake_minimum_required(VERSION 3.5)

project(rviz_rendering_tests)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

find_package(rviz_rendering REQUIRED)
# Adds shallow folders containing ogre_media such as meshes and scripts to Rviz
# so that they can be found at runtime. Requires rviz_rendering.
register_rviz_ogre_media_exports(
  DIRECTORIES
    "ogre_media_resources/scripts"
    "ogre_media_resources/meshes"
)

if(BUILD_TESTING)
  # TODO(Martin-Idel-SI): replace this with ament_lint_auto() and/or add the copyright linter back
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  ament_cppcheck()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  # TODO(wjwwood): reenable when this test is can run on CI
  # ament_add_gtest(test_rviz_rendering_tests test/test_rviz_ogre_media_exports.cpp)
  # if(TARGET test_rviz_rendering_tests)
  #   target_include_directories(test_rviz_rendering_tests PUBLIC src/rviz_rendering_tests)
  #   target_link_libraries(test_rviz_rendering_tests ament_index_cpp::ament_index_cpp)
  # endif()
endif()

ament_package()
