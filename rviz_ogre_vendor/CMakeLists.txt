cmake_minimum_required(VERSION 3.10)

project(rviz_ogre_vendor)

find_package(ament_cmake REQUIRED)

set(PACKAGE_VERSION "1.0.0")

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

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
  set(FREETYPE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

  if(WIN32)
    list(APPEND FREETYPE_CXX_FLAGS "/w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w")
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${FREETYPE_CXX_FLAGS}")

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
      -DWITH_ZLIB=OFF
      -DWITH_BZip2=OFF
      -DWITH_PNG=OFF
      -DWITH_HarfBuzz=OFF
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/freetype_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
    USE_SOURCE_PERMISSIONS
  )
endmacro()

macro(build_zlib)
  set(extra_cmake_args)
  set(ZLIB_CXX_FLAGS ${CMAKE_CXX_FLAGS})

  if(WIN32)
    list(APPEND ZLIB_CXX_FLAGS "/w")
    list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=/w")
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${ZLIB_CXX_FLAGS}")

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
    USE_SOURCE_PERMISSIONS
  )
endmacro()

macro(build_ogre)
  set(extra_cmake_args)
  set(OGRE_C_FLAGS ${CMAKE_C_FLAGS})
  set(OGRE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  # standard library is important for linking, but the other cxx flags are not
  if(CMAKE_CXX_FLAGS MATCHES "-stdlib=libc\\+\\+")
    set(OGRE_CXX_FLAGS "${OGRE_CXX_FLAGS} -stdlib=libc++")
  endif()

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  if(WIN32)
    set(OGRE_C_FLAGS "${OGRE_C_FLAGS} /w /EHsc")
    set(OGRE_CXX_FLAGS "${OGRE_CXX_FLAGS} /w /EHsc")
  elseif(APPLE)
    set(OGRE_CXX_FLAGS "${OGRE_CXX_FLAGS} -std=c++14 -stdlib=libc++ -w")
    list(APPEND extra_cmake_args "-DCMAKE_OSX_ARCHITECTURES='x86_64'")
  else()  # Linux
    set(OGRE_C_FLAGS "${OGRE_C_FLAGS} -w")
    # include Clang -Wno-everything to disable warnings in that build. GCC doesn't mind it
    set(OGRE_CXX_FLAGS "${OGRE_CXX_FLAGS} -std=c++14 -w -Wno-everything")
  endif()
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL:BOOL=TRUE")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D11:BOOL=OFF")
  list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_D3D9:BOOL=OFF")
  list(APPEND extra_cmake_args "-DCMAKE_C_FLAGS=${OGRE_C_FLAGS}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${OGRE_CXX_FLAGS}")

  # Uncomment this line to enable the GL3PLUS render engine
  # list(APPEND extra_cmake_args "-DOGRE_BUILD_RENDERSYSTEM_GL3PLUS:BOOL=TRUE")

  # These next lines can be used to disable some of the components or plugins for OGRE
  list(APPEND extra_cmake_args "-DOGRE_CONFIG_ENABLE_ZIP:BOOL=ON")

  if(BUILDING_FREETYPE_LOCALLY)
    list(APPEND extra_cmake_args
      "-DFREETYPE_HOME=${CMAKE_CURRENT_BINARY_DIR}/freetype_install")
  endif()
  if(BUILDING_ZLIB_LOCALLY)
    list(APPEND extra_cmake_args
      "-DZLIB_ROOT=${CMAKE_CURRENT_BINARY_DIR}/zlib-install")
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  else()
    list(APPEND extra_cmake_args "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}")
    list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  endif()

  find_package(Patch REQUIRED)
  include(ExternalProject)
  ExternalProject_Add(ogre-v1.12.1
    URL https://github.com/OGRECave/ogre/archive/v1.12.1.zip
    URL_MD5 cdbea4006d223c173e0a93864111b936
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
      -DOGRE_RESOURCEMANAGER_STRICT:STRING=2
      -DCMAKE_SKIP_INSTALL_RPATH:BOOL=ON
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/ogre_install
      -DOGRE_BUILD_LIBS_AS_FRAMEWORKS:BOOL=OFF
      -DOGRE_BUILD_COMPONENT_PYTHON:BOOL=FALSE
      -DOGRE_BUILD_COMPONENT_JAVA:BOOL=FALSE
      -DOGRE_BUILD_COMPONENT_CSHARP:BOOL=FALSE
      -DOGRE_BUILD_COMPONENT_BITES:BOOL=FALSE
      ${extra_cmake_args}
      -Wno-dev
    PATCH_COMMAND
      ${Patch_EXECUTABLE} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/pragma-patch.diff &&
      ${Patch_EXECUTABLE} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/fix-arm64.diff &&
      ${Patch_EXECUTABLE} -p1 -N < ${CMAKE_CURRENT_SOURCE_DIR}/relocatable.patch
    COMMAND
      ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/FindFreetype.cmake ${CMAKE_CURRENT_BINARY_DIR}/ogre-v1.12.1-prefix/src/ogre-v1.12.1/CMake/Packages/FindFreetype.cmake
  )

  if(BUILDING_FREETYPE_LOCALLY)
    add_dependencies(ogre-v1.12.1 freetype-2.8.1)
  endif()
  if(BUILDING_ZLIB_LOCALLY)
    add_dependencies(ogre-v1.12.1 zlib-1.2.11)
  endif()

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/ogre_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/rviz_ogre_vendor
    USE_SOURCE_PERMISSIONS
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
  set(ENV_VAR_NAME "PATH")
  set(ENV_VAR_VALUE "opt\\rviz_ogre_vendor\\bin")
else()
  ament_environment_hooks(env_hook/rviz_ogre_vendor_library_path.sh)
  if(APPLE)
    set(ENV_VAR_NAME "DYLD_LIBRARY_PATH")
  else()
    set(ENV_VAR_NAME "LD_LIBRARY_PATH")
  endif()
  set(ENV_VAR_VALUE "opt/rviz_ogre_vendor/lib")
endif()
ament_environment_hooks(env_hook/rviz_ogre_vendor_library_path.dsv.in)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "rviz_ogre_vendor-extras.cmake.in"
)
