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

# TODO(wjwwood): have ogre include linking for FreeImage, FreeType, OpenGL, and the X11 libraries
find_library(FREEIMAGE_LIBRARIES freeimage)
if(NOT FREEIMAGE_LIBRARIES)
  message(STATUS "Could not find freeimage library")
else()
  list(APPEND OGRE_LIBRARIES ${FREEIMAGE_LIBRARIES})
endif()
find_package(Freetype REQUIRED)
find_package(OpenGL REQUIRED)
list(APPEND OGRE_LIBRARIES ${FREETYPE_LIBRARIES} ${OPENGL_LIBRARIES} ${CMAKE_DL_LIBS})
if(UNIX AND NOT APPLE)
  find_package(X11 REQUIRED)
  find_library(XAW_LIBRARY NAMES Xaw Xaw7 PATHS ${DEP_LIB_SEARCH_DIR} ${X11_LIB_SEARCH_PATH})
  list(APPEND OGRE_LIBRARIES ${X11_LIBRARIES} ${X11_Xt_LIB} ${X11_Xrandr_LIB} ${XAW_LIBRARY})
endif()
# if(APPLE)
#   list(APPEND OGRE_LIBRARIES "-framework Cocoa" "-framework OpenGL")
# endif()
# endtodo

find_package(Qt5 REQUIRED COMPONENTS Widgets)
# TODO(wjwwood): this block is to setup the windeployqt tool, could be removed later.
if(Qt5_FOUND AND WIN32 AND TARGET Qt5::qmake AND NOT TARGET Qt5::windeployqt)
    get_target_property(_qt5_qmake_location Qt5::qmake IMPORTED_LOCATION)

    execute_process(
        COMMAND "${_qt5_qmake_location}" -query QT_INSTALL_PREFIX
        RESULT_VARIABLE return_code
        OUTPUT_VARIABLE qt5_install_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(imported_location "${qt5_install_prefix}/bin/windeployqt.exe")

    if(EXISTS ${imported_location})
        add_executable(Qt5::windeployqt IMPORTED)

        set_target_properties(Qt5::windeployqt PROPERTIES
            IMPORTED_LOCATION ${imported_location}
        )
    endif()
endif()

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

# TODO(wjwwood): find a way to make this optional or to run without "deploying" the
#                necessary dlls and stuff to the bin folder.
#                see:
# https://stackoverflow.com/questions/41193584/deploy-all-qt-dependencies-when-building#41199492
if(TARGET Qt5::windeployqt)
    # execute windeployqt in a tmp directory after build
    add_custom_command(TARGET rendering_example
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
        COMMAND set PATH=%PATH%$<SEMICOLON>${qt5_install_prefix}/bin
        COMMAND Qt5::windeployqt --dir "${CMAKE_CURRENT_BINARY_DIR}/windeployqt" "$<TARGET_FILE_DIR:rendering_example>/$<TARGET_FILE_NAME:rendering_example>"
    )

    # copy deployment directory during installation
    install(
        DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
        DESTINATION bin
    )
endif()

install(TARGETS rendering_example DESTINATION bin)

# add_executable(test_QTOgreWindow src/test_QTOgreWindow.cpp src/QTOgreWindow.cpp)
# target_link_libraries(test_QTOgreWindow ${OGRE_LIBRARIES} Qt5::Widgets)
# target_compile_options(test_QTOgreWindow PUBLIC -Wno-error)

ament_package()
