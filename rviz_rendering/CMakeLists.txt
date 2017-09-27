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

find_package(ament_index_cpp REQUIRED)
# do find_package(rviz_ogre_vendor) first to make sure the custom OGRE is found
find_package(rviz_ogre_vendor REQUIRED)

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

# These need to be added in the add_library() call so AUTOMOC detects them.
set(headers_with_q_object
  include/rviz_rendering/render_window.hpp
)

add_library(rviz_rendering
  ${headers_with_q_object}
  src/rviz_rendering/apply_visibility_bits.cpp
  src/rviz_rendering/arrow.cpp
  src/rviz_rendering/axes.cpp
  # src/rviz_rendering/initialization.cpp
  src/rviz_rendering/logging.cpp
  src/rviz_rendering/geometry.cpp
  src/rviz_rendering/movable_text.cpp
  src/rviz_rendering/object.cpp
  src/rviz_rendering/ogre_logging.cpp
  src/rviz_rendering/orthographic.cpp
  src/rviz_rendering/render_system.cpp
  src/rviz_rendering/render_window.cpp
  src/rviz_rendering/ogre_render_window_impl.cpp
  src/rviz_rendering/resource_config.cpp
  src/rviz_rendering/shape.cpp
  src/rviz_rendering/grid.cpp
)
target_link_libraries(rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_ogre_vendor::RenderSystem_GL
  rviz_ogre_vendor::OgreGLSupport
  Qt5::Widgets
  ament_index_cpp::ament_index_cpp
)
target_include_directories(rviz_rendering
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# This will give the library a default directory to fallback to, but it should be overridden.
target_compile_definitions(rviz_rendering
  # TODO(wjwwood): this might not be required anymore with static linking
  PRIVATE "-DRVIZ_RENDERING_OGRE_PLUGIN_DIR=\"${OGRE_PLUGIN_DIR}\""
)

ament_export_interfaces(rviz_rendering)
ament_export_dependencies(rviz_ogre_vendor)
ament_export_dependencies(ament_index_cpp)

install(
  TARGETS rviz_rendering
  EXPORT rviz_rendering
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(rendering_example src/examples/rendering_example.cpp)
target_link_libraries(rendering_example rviz_rendering Qt5::Widgets)

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
        COMMAND
          Qt5::windeployqt
          --dir "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
          "$<TARGET_FILE_DIR:rendering_example>/$<TARGET_FILE_NAME:rendering_example>"
    )

    # copy deployment directory during installation
    install(
        DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
        DESTINATION bin
    )
endif()

install(TARGETS rendering_example DESTINATION bin)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/ogre_media"
        DESTINATION "share/rviz_rendering"
        USE_SOURCE_PERMISSIONS)

if(BUILD_TESTING)
    # TODO(wjwwood): replace this with ament_lint_auto() and/or add the copyright linter back
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    ament_cppcheck()
    ament_cpplint()
    ament_lint_cmake()
    ament_uncrustify()
endif()

ament_package()
