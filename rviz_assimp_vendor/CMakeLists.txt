cmake_minimum_required(VERSION 3.5)

project(rviz_assimp_vendor)

find_package(ament_cmake REQUIRED)

set(PACKAGE_VERSION "1.0.0")

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
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=-w")
  endif()

  list(APPEND extra_cmake_args "-DASSIMP_BUILD_ASSIMP_TOOLS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_SAMPLES=OFF")

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
  )
endmacro()

# Copy and fix the assimp config files on Ubuntu, in order to suppress a warning.
# This should be removed once upstream is updated to assimp-0.5.1, see:
#   https://github.com/ros2/rviz/issues/524
#   https://bugs.launchpad.net/ubuntu/+source/assimp/+bug/1869405
### BEGIN HACKS
set(IS_UBUNTU_FOCAL FALSE)
if(UNIX AND NOT APPLE)
  find_program(LSB_RELEASE_EXEC lsb_release)
  if(EXISTS "${LSB_RELEASE_EXEC}")
    execute_process(COMMAND ${LSB_RELEASE_EXEC} -is
        OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(COMMAND ${LSB_RELEASE_EXEC} -sr
      OUTPUT_VARIABLE LSB_RELEASE_RELEASE_SHORT
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  else()
    set(LSB_RELEASE_ID_SHORT "Unknown")
    set(LSB_RELEASE_RELEASE_SHORT "Unknown")
  endif()

  if(${LSB_RELEASE_ID_SHORT} STREQUAL "Ubuntu" AND ${LSB_RELEASE_RELEASE_SHORT} STREQUAL "20.04")
    set(IS_UBUNTU_FOCAL TRUE)
  endif()
endif()

if(IS_UBUNTU_FOCAL)
  file(GLOB_RECURSE assimp_target_files "/usr/lib/*/cmake/assimp-5.0/assimpTargets.cmake")
  list(LENGTH assimp_target_files assimp_target_files_len)
  if(assimp_target_files_len EQUAL 0)
    message(FATAL_ERROR "failed to find assimpTargets.cmake as expected")
  endif()
  if(NOT assimp_target_files_len EQUAL 1)
    message(FATAL_ERROR "found multiple assimpTargets.cmake files, unexpectedly")
  endif()
  list(GET assimp_target_files 0 assimp_target_file)

  get_filename_component(assimp_target_dir "${assimp_target_file}" DIRECTORY)
  set(new_assimp_target_dir "${CMAKE_CURRENT_BINARY_DIR}/assimp-0.5")

  file(READ "${assimp_target_dir}/assimpTargets.cmake" assimp_targets_content)
  string(REPLACE
    "if(ON)"
    "set(WORKAROUND ON)\nif(WORKAROUND)"
    assimp_targets_content
    ${assimp_targets_content})
  file(WRITE "${new_assimp_target_dir}/assimpTargets.cmake" "${assimp_targets_content}")
  file(READ "${assimp_target_dir}/assimpTargets-release.cmake" assimp_targets_content)
  string(REPLACE
    "if(ON)"
    "set(WORKAROUND ON)\nif(WORKAROUND)"
    assimp_targets_content
    ${assimp_targets_content})
  file(WRITE "${new_assimp_target_dir}/assimpTargets-release.cmake" "${assimp_targets_content}")
  file(
    COPY "${assimp_target_dir}/assimp-config.cmake"
    DESTINATION "${new_assimp_target_dir}")
  file(
    COPY "${assimp_target_dir}/assimp-config-version.cmake"
    DESTINATION "${new_assimp_target_dir}")
  set(assimp_DIR "${new_assimp_target_dir}")

  install(
    DIRECTORY
      ${new_assimp_target_dir}/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_assimp_vendor_custom_config
  )
endif()
### END HACKS
find_package(assimp QUIET)

if(NOT assimp_FOUND OR "${assimp_VERSION}" VERSION_LESS 4.1.0)
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

ament_package(
  CONFIG_EXTRAS "rviz_assimp_vendor-extras.cmake.in"
)
