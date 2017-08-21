cmake_minimum_required(VERSION 3.5)

project(rviz_rendering)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor QUIET REQUIRED)
find_package(OGRE REQUIRED)

# TODO(wjwwood): have ogre include linking for FreeImage and FreeType
find_library(FREEIMAGE_LIBRARIES freeimage)
if(NOT FREEIMAGE_LIBRARIES)
  message(FATAL_ERROR "Could not find freeimage library")
endif()
find_package(Freetype REQUIRED)
list(APPEND OGRE_LIBRARIES ${FREEIMAGE_LIBRARIES} ${FREETYPE_LIBRARIES})
# endtodo

find_package(Qt5Widgets REQUIRED)

include_directories(include ${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})

# These need to be added in the add_library() call so AUTOMOC detects them.
set(headers_with_q_object
  include/rviz_rendering/render_window.hpp
)

add_library(${PROJECT_NAME}
  ${headers_with_q_object}
  src/rviz_rendering/logging.cpp
  src/rviz_rendering/ogre_logging.cpp
  src/rviz_rendering/render_system.cpp
  src/rviz_rendering/render_window.cpp
  src/rviz_rendering/render_window_impl.cpp
  # src/rviz_rendering/render_widget.cpp
  src/rviz_rendering/resource_config.cpp
)
target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES} Qt5::Widgets)
# This will give the library a default directory to fallback to, but it should be overridden.
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "-DRVIZ_RENDERING_DEFAULT_RESOURCE_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\""
  PRIVATE "-DRVIZ_RENDERING_OGRE_PLUGIN_DIR=\"${OGRE_PLUGIN_DIR}\""
)

add_executable(rendering_example src/examples/rendering_example.cpp)
target_link_libraries(rendering_example ${PROJECT_NAME} ${OGRE_LIBRARIES} Qt5::Widgets)

add_executable(test_QTOgreWindow src/test_QTOgreWindow.cpp src/QTOgreWindow.cpp)
target_link_libraries(test_QTOgreWindow ${OGRE_LIBRARIES} Qt5::Widgets)
target_compile_options(test_QTOgreWindow PUBLIC -Wno-error)

ament_package()
