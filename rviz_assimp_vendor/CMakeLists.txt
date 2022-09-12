# lint_cmake: -readability/wonkycase
cmake_minimum_required(VERSION 3.5)

project(rviz_assimp_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build assimp from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

macro(build_assimp)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(WIN32)
    # the /EHsc flag is needed to prevent warnings within the assimp build
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=/w /EHsc")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -std=c++14 -w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=${CMAKE_C_FLAGS} -w")
  endif()

  list(APPEND extra_cmake_args "-DASSIMP_BUILD_ASSIMP_TOOLS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_SAMPLES=OFF")
  list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")

  if(WIN32 AND NOT ${CMAKE_VERBOSE_MAKEFILE})
    set(should_log ON)  # prevent warnings in Windows CI
  else()
    set(should_log OFF)
  endif()

  include(ExternalProject)
  ExternalProject_Add(assimp-4.1.0
    URL https://github.com/assimp/assimp/archive/v4.1.0.tar.gz
    URL_MD5 83b53a10c38d964bd1e69da0606e2727
    TIMEOUT 600
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/assimp_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/assimp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor
    USE_SOURCE_PERMISSIONS
  )
endmacro()

# Override ON so that the following CMake logic in assimp 5.0.1 and older
# doesn't result in a CMake warning: if(ON)
#   https://github.com/ros2/rviz/issues/524
#   https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1869405
set(ON 1)

find_package(assimp QUIET)

if(FORCE_BUILD_VENDOR_PKG OR NOT assimp_FOUND OR "${assimp_VERSION}" VERSION_LESS 4.1.0)
  build_assimp()

  if(WIN32)
    ament_environment_hooks(env_hook/rviz_assimp_vendor_library_path.bat)
    set(ENV_VAR_NAME "PATH")
    set(ENV_VAR_VALUE "opt\\rviz_assimp_vendor\\bin")
  else()
    ament_environment_hooks(env_hook/rviz_assimp_vendor_library_path.sh)
    if(APPLE)
      set(ENV_VAR_NAME "DYLD_LIBRARY_PATH")
    else()
      set(ENV_VAR_NAME "LD_LIBRARY_PATH")
    endif()
    set(ENV_VAR_VALUE "opt/rviz_assimp_vendor/lib")
  endif()
  ament_environment_hooks(env_hook/rviz_assimp_vendor_library_path.dsv.in)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_assimp_vendor-extras.cmake.in"
)
