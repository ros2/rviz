cmake_minimum_required(VERSION 3.5)

project(rviz_assimp_vendor)

set(PACKAGE_VERSION "1.0.0")

macro(build_assimp)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(NOT WIN32)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=-w")
  endif()

  list(APPEND extra_cmake_args "-DASSIMP_BUILD_ASSIMP_TOOLS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DASSIMP_BUILD_SAMPLES=OFF")

  include(ExternalProject)
  ExternalProject_Add(assimp-4.0.1
    URL https://github.com/assimp/assimp/archive/v4.0.1.tar.gz
    URL_MD5 23a6301c728a413aafbfa1cca19ba91f
    TIMEOUT 600
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

build_assimp()

configure_file(rviz_assimp_vendorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_assimp_vendorConfig.cmake" @ONLY)
configure_file(rviz_assimp_vendorConfig-version.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_assimp_vendorConfig-version.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/rviz_assimp_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/rviz_assimp_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
