cmake_minimum_required(VERSION 3.10)
project(rviz_ogre_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

if(WIN32)
  set(FREETYPE_C_FLAGS "${CMAKE_C_FLAGS}")
  if(MSVC)
    set(FREETYPE_C_FLAGS "/wd4244 ${FREETYPE_C_FLAGS}")
    set(FREETYPE_C_FLAGS "/wd4267 ${FREETYPE_C_FLAGS}")
    set(FREETYPE_C_FLAGS "/wd4312 ${FREETYPE_C_FLAGS}")
    set(FREETYPE_C_FLAGS "/wd4996 ${FREETYPE_C_FLAGS}")
  endif()

  ament_vendor(freetype_vendor
    VCS_URL https://git.savannah.gnu.org/git/freetype/freetype2.git
    VCS_VERSION VER-2-11-1
    CMAKE_ARGS
      -DFT_DISABLE_ZLIB:BOOL=ON
      -DFT_DISABLE_BZIP2:BOOL=ON
      -DFT_DISABLE_PNG:BOOL=ON
      -DFT_DISABLE_HARFBUZZ:BOOL=ON
      "-DCMAKE_C_FLAGS=${FREETYPE_C_FLAGS}"
  )

  set(ZLIB_C_FLAGS "${ZLIB_C_FLAGS}")
  if(MSVC)
    set(ZLIB_C_FLAGS "/wd4267 ${ZLIB_C_FLAGS}")
  endif()

  ament_vendor(zlib_vendor
    VCS_URL https://github.com/madler/zlib.git
    VCS_VERSION v1.2.13
    CMAKE_ARGS
      # zlib doesn't use CMAKE_INSTALL_PREFIX correctly, so we need to override
      -DINSTALL_BIN_DIR=<INSTALL_DIR>/bin
      -DINSTALL_LIB_DIR=<INSTALL_DIR>/lib
      -DINSTALL_INC_DIR=<INSTALL_DIR>/include
      -DINSTALL_MAN_DIR=<INSTALL_DIR>/share/man
      -DINSTALL_PKGCONFIG_DIR=<INSTALL_DIR>/share/pkgconfig
      "-DCMAKE_C_FLAGS=${ZLIB_C_FLAGS}"
  )
endif()

set(OGRE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
if(MSVC)
  set(OGRE_CXX_FLAGS "/wd4101 ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "/wd4244 ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "/wd4267 ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "/wd4334 ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "/wd4477 ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "/wd4996 ${OGRE_CXX_FLAGS}")
else()
  set(OGRE_CXX_FLAGS "-Wno-deprecated-declarations ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "-Wno-mismatched-new-delete ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "-Wno-range-loop-construct ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "-Wno-undef ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "-Wno-misleading-indentation ${OGRE_CXX_FLAGS}")
  set(OGRE_CXX_FLAGS "-Wno-implicit-const-int-float-conversion ${OGRE_CXX_FLAGS}")

  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(OGRE_CXX_FLAGS "-Wno-maybe-uninitialized ${OGRE_CXX_FLAGS}")
    set(OGRE_CXX_FLAGS "-Wno-format-overflow ${OGRE_CXX_FLAGS}")
    set(OGRE_CXX_FLAGS "-Wno-dangling-pointer ${OGRE_CXX_FLAGS}")
    set(OGRE_CXX_FLAGS "-Wno-array-compare ${OGRE_CXX_FLAGS}")
  endif()
endif()

set(OGRE_CMAKE_ARGS)
if(NOT WIN32)
  list(APPEND OGRE_CMAKE_ARGS -DCMAKE_SKIP_INSTALL_RPATH:BOOL=ON)
endif()

ament_vendor(ogre_vendor
  VCS_URL https://github.com/OGRECave/ogre.git
  VCS_VERSION v1.12.10
  CMAKE_ARGS
    -DOGRE_STATIC:BOOL=OFF
    -DOGRE_INSTALL_PDB:BOOL=OFF
    -DOGRE_BUILD_DEPENDENCIES:BOOL=OFF
    -DOGRE_BUILD_TESTS:BOOL=OFF
    -DOGRE_BUILD_SAMPLES:BOOL=FALSE
    -DOGRE_INSTALL_SAMPLES:BOOL=FALSE
    -DOGRE_INSTALL_SAMPLES_SOURCE:BOOL=FALSE
    -DOGRE_CONFIG_THREADS:STRING=0
    -DOGRE_RESOURCEMANAGER_STRICT:STRING=2
    -DOGRE_BUILD_LIBS_AS_FRAMEWORKS:BOOL=OFF
    -DOGRE_BUILD_COMPONENT_PYTHON:BOOL=FALSE
    -DOGRE_BUILD_COMPONENT_JAVA:BOOL=FALSE
    -DOGRE_BUILD_COMPONENT_CSHARP:BOOL=FALSE
    -DOGRE_BUILD_COMPONENT_BITES:BOOL=FALSE
    -DOGRE_BUILD_PLUGIN_DOT_SCENE:BOOL=FALSE
    -DOGRE_BUILD_TOOLS:BOOL=FALSE
    -DOGRE_CONFIG_ENABLE_ZIP:BOOL=ON
    -DOGRE_BUILD_RENDERSYSTEM_GL:BOOL=TRUE
    -DOGRE_BUILD_RENDERSYSTEM_D3D11:BOOL=OFF
    -DOGRE_BUILD_RENDERSYSTEM_D3D9:BOOL=OFF
    -DCMAKE_POLICY_DEFAULT_CMP0074=NEW
    "-DCMAKE_CXX_FLAGS=${OGRE_CXX_FLAGS}"
    ${OGRE_CMAKE_ARGS}
  PATCHES patches
)

if(TARGET ogre_vendor)
  if(TARGET freetype_vendor)
    ExternalProject_Add_StepDependencies(ogre_vendor configure freetype_vendor)
  endif()

  if(TARGET zlib_vendor)
    ExternalProject_Add_StepDependencies(ogre_vendor configure zlib_vendor)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS_POST "rviz_ogre_vendor-extras.cmake.in"
)
