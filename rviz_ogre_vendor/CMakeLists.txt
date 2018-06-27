cmake_minimum_required(VERSION 3.5)

project(rviz_ogre_vendor)

find_package(ament_cmake REQUIRED)

set(PACKAGE_VERSION "1.0.0")

if(WIN32)
  set(BUILDING_FREETYPE_LOCALLY ON)
  set(BUILDING_ZLIB_LOCALLY ON)
else()
  set(BUILDING_FREETYPE_LOCALLY OFF)
  set(BUILDING_ZLIB_LOCALLY OFF)
endif()

if(WIN32 AND NOT ${CMAKE_VERBOSE_MAKEFILE})
  set(should_log ON)  # prevent warnings in Windows CI
else()
  set(should_log OFF)
endif()

macro(build_freetype)
  set(extra_cmake_args)

  if(WIN32)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=/w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w")
  endif()

  include(ExternalProject)
  ExternalProject_Add(freetype-2.8.1
    URL https://git.savannah.gnu.org/cgit/freetype/freetype2.git/snapshot/freetype2-VER-2-6-5.tar.gz
    URL_MD5 0dd62a0125895431e074a3df29f94d69
    TIMEOUT 600
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/freetype_install
      ${extra_cmake_args}
      -Wno-dev
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/freetype_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
  )
endmacro()

macro(build_zlib)
  set(extra_cmake_args)

  if(WIN32)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=/w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w")
  endif()

  include(ExternalProject)
  ExternalProject_Add(zlib-1.2.11
    URL https://www.zlib.net/fossils/zlib-1.2.11.tar.gz
    URL_MD5 1c9f62f0778697a09d36121ead88e08e
    TIMEOUT 600
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/zlib-install
      ${extra_cmake_args}
      -Wno-dev
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/zlib-install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
  )
endmacro()

macro(build_ogre)
  set(extra_cmake_args)
  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()
  if(WIN32)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=/w /EHsc")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w /EHsc")
  elseif(APPLE)
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -stdlib=libc++ -w")
    list(APPEND extra_cmake_args "-DCMAKE_OSX_ARCHITECTURES='x86_64'")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=-std=c++14 -w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=-w")
  endif()
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL:BOOL=TRUE")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D11:BOOL=OFF")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D9:BOOL=OFF")

  # Uncomment this line to enable the GL3PLUS render engine
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL3PLUS:BOOL=TRUE")

  # These next lines can be used to disable some of the components or plugins for OGRE
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_COMPONENT_BITES:BOOL=OFF")
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_COMPONENT_HLMS:BOOL=ON")
  list(APPEND extra_cmake_args "-DOGRE_CONFIG_ENABLE_ZIP:BOOL=ON")

  if(BUILDING_FREETYPE_LOCALLY)
    list(APPEND extra_cmake_args
      "-DFREETYPE_HOME=${CMAKE_CURRENT_BINARY_DIR}/freetype_install")
  endif()
  if(BUILDING_ZLIB_LOCALLY)
    list(APPEND extra_cmake_args
      "-DZLIB_ROOT=${CMAKE_CURRENT_BINARY_DIR}/zlib_install")
  endif()

  if(WIN32)
    set(patch_exe patch.exe)
  else()
    set(patch_exe patch)
  endif()

  include(ExternalProject)
  ExternalProject_Add(ogre-master-ca665a6
    URL https://github.com/OGRECave/ogre/archive/v1.10.11.zip
    URL_MD5 6ffd5048fae72805e287bfc0b462b2ea
    TIMEOUT 1200
    LOG_CONFIGURE ${should_log}
    LOG_BUILD ${should_log}
    CMAKE_ARGS
      -DOGRE_STATIC:BOOL=OFF
      -DOGRE_DEPENDENCIES_DIR=${CMAKE_CURRENT_BINARY_DIR}/ogredeps
      -DOGRE_INSTALL_PDB:BOOL=OFF
      -DOGRE_BUILD_DEPENDENCIES:BOOL=OFF
      -DOGRE_BUILD_TESTS:BOOL=OFF
      -DOGRE_BUILD_SAMPLES:BOOL=FALSE
      -DOGRE_INSTALL_SAMPLES:BOOL=FALSE
      -DOGRE_INSTALL_SAMPLES_SOURCE:BOOL=FALSE
      -DOGRE_CONFIG_THREADS:STRING=0
      -DOGRE_RESOURCEMANAGER_STRICT:BOOL=ON
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/ogre_install
      -DOGRE_BUILD_LIBS_AS_FRAMEWORKS:BOOL=OFF
      ${extra_cmake_args}
      -Wno-dev
    PATCH_COMMAND
      ${patch_exe} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/ogre-ca665a6-patch.diff
  )

  if(BUILDING_FREETYPE_LOCALLY)
    add_dependencies(ogre-master-ca665a6 freetype-2.8.1)
  endif()
  if(BUILDING_ZLIB_LOCALLY)
    add_dependencies(ogre-master-ca665a6 zlib-1.2.11)
  endif()

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/ogre_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
  )
endmacro()

if(BUILDING_FREETYPE_LOCALLY)
  build_freetype()
endif()
if(BUILDING_ZLIB_LOCALLY)
  build_zlib()
endif()

build_ogre()

if(WIN32)
  ament_environment_hooks(env_hook/rviz_ogre_vendor_library_path.bat)
else()
  ament_environment_hooks(env_hook/rviz_ogre_vendor_library_path.sh)
endif()

ament_package(
  CONFIG_EXTRAS "rviz_ogre_vendor-extras.cmake.in"
)
