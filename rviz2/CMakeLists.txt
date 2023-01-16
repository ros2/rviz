cmake_minimum_required(VERSION 3.5)

project(rviz2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)

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

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  Qt5::Widgets
)

# TODO(wjwwood): find a way to make this optional or to run without "deploying" the
#                necessary dlls and stuff to the bin folder.
#                see:
# https://stackoverflow.com/questions/41193584/deploy-all-qt-dependencies-when-building#41199492
if(TARGET Qt5::windeployqt)
  # execute windeployqt in a tmp directory after build
  add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
    COMMAND set PATH=%PATH%$<SEMICOLON>${qt5_install_prefix}/bin
    COMMAND
    Qt5::windeployqt
    --dir "${CMAKE_CURRENT_BINARY_DIR}/windeployqt"
    "$<TARGET_FILE_DIR:${PROJECT_NAME}>/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
  )

  # copy deployment directory during installation
  install(
    DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
    DESTINATION bin
  )
  install(
    DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}/windeployqt/"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/rviz1_to_rviz2.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

ament_package()
