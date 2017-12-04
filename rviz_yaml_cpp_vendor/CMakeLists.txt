cmake_minimum_required(VERSION 3.5)

project(rviz_yaml_cpp_vendor)

# TODO(wjwood): remove when windows build is fixed
if(DEFINED ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE})
  set(RVIZ_BUILD_ON_WINDOWS_OVERRIDE "$ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE}")
endif()
if(WIN32 AND NOT DEFINED RVIZ_BUILD_ON_WINDOWS_OVERRIDE)
  message(STATUS "${PROJECT_NAME} is not supported on Windows, skipping...")
  return()
endif()

set(PACKAGE_VERSION "1.0.0")

macro(build_yaml_cpp)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(NOT WIN32)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=-w")
  endif()

  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_TOOLS=OFF")
  list(APPEND extra_cmake_args "-DYAML_CPP_BUILD_CONTRIB=OFF")
  list(APPEND extra_cmake_args "-DBUILD_SHARED_LIBS=ON")

  include(ExternalProject)
  # This specific version (past the current latest release of 0.5.3) is required to make
  # yaml-cpp relocatable, hopefully it is released again soon.
  # See: https://github.com/jbeder/yaml-cpp/pull/538
  ExternalProject_Add(yaml_cpp-86ae3a5
    URL https://github.com/jbeder/yaml-cpp/archive/86ae3a5aa7e2109d849b2df89176d6432a35265d.zip
    URL_MD5 28091f7f66086a84b14da1f5d798c6a6
    TIMEOUT 600
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_install
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/yaml_cpp_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_yaml_cpp_vendor
  )
endmacro()

build_yaml_cpp()

configure_file(rviz_yaml_cpp_vendorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_yaml_cpp_vendorConfig.cmake" @ONLY)
configure_file(rviz_yaml_cpp_vendorConfig-version.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_yaml_cpp_vendorConfig-version.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/rviz_yaml_cpp_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/rviz_yaml_cpp_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
