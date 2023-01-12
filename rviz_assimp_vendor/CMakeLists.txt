cmake_minimum_required(VERSION 3.10)
project(rviz_assimp_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# Override ON so that the following CMake logic in assimp 5.0.1 and older
# doesn't result in a CMake warning: if(ON)
#   https://github.com/ros2/rviz/issues/524
#   https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1869405
set(ON 1)

# TODO: Switch to version range in find_package in CMake 3.19
find_package(assimp QUIET)
if(NOT assimp_FOUND OR "${assimp_VERSION}" VERSION_LESS 4.1.0)
  set(assimp_FOUND FALSE)
endif()

if(MSVC)
  set(ASSIMP_CXX_FLAGS "/EHsc ${CMAKE_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "/wd4700 ${ASSIMP_CXX_FLAGS}")
else()
  set(ASSIMP_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "-Wno-address-of-packed-member ${ASSIMP_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "-Wno-catch-value ${ASSIMP_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "-Wno-class-memaccess ${ASSIMP_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "-Wno-range-loop-construct ${ASSIMP_CXX_FLAGS}")
  set(ASSIMP_CXX_FLAGS "-Wno-tautological-compare ${ASSIMP_CXX_FLAGS}")
endif()

ament_vendor(assimp_vendor
  SATISFIED ${assimp_FOUND}
  VCS_URL https://github.com/assimp/assimp.git
  VCS_VERSION v4.1.0
  CMAKE_ARGS
    -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF
    -DASSIMP_BUILD_TESTS:BOOL=OFF
    -DASSIMP_BUILD_SAMPLES:BOOL=OFF
    "-DCMAKE_CXX_FLAGS=${ASSIMP_CXX_FLAGS}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_assimp_vendor-extras.cmake.in"
)
