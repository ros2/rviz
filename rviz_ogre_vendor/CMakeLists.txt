cmake_minimum_required(VERSION 3.5)

project(rviz_ogre_vendor)

# TODO(wjwood): remove when windows build is fixed
if(DEFINED ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE})
  set(RVIZ_BUILD_ON_WINDOWS_OVERRIDE "$ENV{RVIZ_BUILD_ON_WINDOWS_OVERRIDE}")
endif()
if(WIN32 AND NOT DEFINED RVIZ_BUILD_ON_WINDOWS_OVERRIDE)
  message(STATUS "${PROJECT_NAME} is not supported on Windows, skipping...")
  return()
endif()

set(PACKAGE_VERSION "1.0.0")

macro(build_ogre)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(NOT WIN32 AND NOT APPLE)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14")
  elseif(APPLE)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -stdlib=libc++")
    list(APPEND extra_cmake_args "-DCMAKE_OSX_ARCHITECTURES='x86_64'")
  endif()
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL:BOOL=TRUE")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D11:BOOL=OFF")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D9:BOOL=OFF")

  # Uncomment this line to enable the GL3PLUS render engine
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL3PLUS:BOOL=TRUE")

  # These next lines can be used to disable some of the components or plugins for OGRE
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_COMPONENT_BITES:BOOL=OFF")
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_COMPONENT_HLMS:BOOL=ON")
  list(APPEND extra_cmake_args "-DOGRE_CONFIG_ENABLE_ZIP:BOOL=OFF")

  include(ExternalProject)
  ExternalProject_Add(ogre-master-ca665a6
    URL https://github.com/OGRECave/ogre/archive/ca665a69a5ca47608ee0ad304b2596ff866616da.zip
    URL_MD5 a627f447d0013a9cf0889117ac76bc5f
    TIMEOUT 600
    CMAKE_ARGS
      -DOGRE_STATIC:BOOL=ON
      -DOGRE_DEPENDENCIES_DIR=${CMAKE_CURRENT_BINARY_DIR}/ogredeps
      -DOGRE_INSTALL_PDB:BOOL=OFF
      -DOGRE_BUILD_DEPENDENCIES:BOOL=ON
      -DOGRE_BUILD_TESTS:BOOL=OFF
      -DOGRE_INSTALL_DOCS:BOOL=FALSE
      -DOGRE_BUILD_SAMPLES:BOOL=FALSE
      -DOGRE_INSTALL_SAMPLES:BOOL=FALSE
      -DOGRE_INSTALL_SAMPLES_SOURCE:BOOL=FALSE
      -DOGRE_INSTALL_GENERIC_PLUGINS_CFG:BOOL=TRUE
      -DOGRE_CONFIG_THREADS:STRING=0
      -DOGRE_RESOURCEMANAGER_STRICT:BOOL=ON
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/ogre_install
      -DOGRE_BUILD_LIBS_AS_FRAMEWORKS:BOOL=OFF
      ${extra_cmake_args}
      -Wno-dev
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/ogre_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
  )
endmacro()

build_ogre()

configure_file(rviz_ogre_vendorConfig.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_ogre_vendorConfig.cmake" @ONLY)
configure_file(rviz_ogre_vendorConfig-version.cmake.in
  "${PROJECT_BINARY_DIR}/rviz_ogre_vendorConfig-version.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}/rviz_ogre_vendorConfig.cmake"
  "${PROJECT_BINARY_DIR}/rviz_ogre_vendorConfig-version.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake)
