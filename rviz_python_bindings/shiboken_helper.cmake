find_package(PythonInterp "3.3" REQUIRED)

if(__PYTHON_QT_BINDING_SHIBOKEN_HELPER_INCLUDED)
  return()
endif()
set(__PYTHON_QT_BINDING_SHIBOKEN_HELPER_INCLUDED TRUE)

set(PYTHON_SUFFIX ".cpython-${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}m")
set(PYTHON_EXTENSION_SUFFIX "${PYTHON_SUFFIX}-${CMAKE_CXX_LIBRARY_ARCHITECTURE}")

find_package(Shiboken2 QUIET)
if(Shiboken2_FOUND)
  message(STATUS "Found Shiboken2 version ${Shiboken2_VERSION}")
  if(NOT ${Shiboken2_VERSION} VERSION_LESS "5.13")
    get_property(SHIBOKEN_INCLUDE_DIR TARGET Shiboken2::libshiboken PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    get_property(SHIBOKEN_LIBRARY TARGET Shiboken2::libshiboken PROPERTY LOCATION)
    set(SHIBOKEN_BINARY Shiboken2::shiboken2)
  endif()
  message(STATUS "Using SHIBOKEN_INCLUDE_DIR: ${SHIBOKEN_INCLUDE_DIR}")
  message(STATUS "Using SHIBOKEN_LIBRARY: ${SHIBOKEN_LIBRARY}")
  message(STATUS "Using SHIBOKEN_BINARY: ${SHIBOKEN_BINARY}")
endif()
set(PYTHON_BASENAME "${PYTHON_SUFFIX}-${CMAKE_CXX_LIBRARY_ARCHITECTURE}")

find_package(PySide2 QUIET)
if(PySide2_FOUND)
  message(STATUS "Found PySide2 version ${PySide2_VERSION}")
  if(NOT ${PySide2_VERSION} VERSION_LESS "5.13")
    get_property(PYSIDE_INCLUDE_DIR TARGET PySide2::pyside2 PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    get_property(PYSIDE_LIBRARY TARGET PySide2::pyside2 PROPERTY LOCATION)
  endif()
  message(STATUS "Using PYSIDE_INCLUDE_DIR: ${PYSIDE_INCLUDE_DIR}")
  message(STATUS "Using PYSIDE_LIBRARY: ${PYSIDE_LIBRARY}")
endif()

set(Python_ADDITIONAL_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

if(Shiboken2_FOUND AND PySide2_FOUND AND PYTHONLIBS_FOUND)
  if(${CMAKE_VERSION} VERSION_LESS "3.14")
    # the shiboken invocation needs CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES
    # which is broken before CMake 3.14
    # see https://gitlab.kitware.com/cmake/cmake/issues/18394
    message(STATUS "Shiboken binding generator available but CMake version is older than 3.14.")
    set(shiboken_helper_NOTFOUND TRUE)
  else()
    message(STATUS "Shiboken binding generator available.")
    set(shiboken_helper_FOUND TRUE)
  endif()
else()
  message(STATUS "Shiboken binding generator NOT available.")
  set(shiboken_helper_NOTFOUND TRUE)
endif()


macro(_shiboken_generator_command VAR GLOBAL TYPESYSTEM INCLUDE_PATH BUILD_DIR)
  # Add includes from current directory, Qt, PySide and compiler specific dirs
  get_directory_property(SHIBOKEN_HELPER_INCLUDE_DIRS INCLUDE_DIRECTORIES)
  list(APPEND SHIBOKEN_HELPER_INCLUDE_DIRS
    ${QT_INCLUDE_DIR}
    ${PYSIDE_INCLUDE_DIR}
    ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
  # See ticket https://code.ros.org/trac/ros-pkg/ticket/5219
  set(SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS "")
  foreach(dir ${SHIBOKEN_HELPER_INCLUDE_DIRS})
    set(SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS "${SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS}:${dir}")
  endforeach()
  string(REPLACE ";" ":" INCLUDE_PATH_WITH_COLONS "${INCLUDE_PATH}")
  set(${VAR} ${SHIBOKEN_BINARY}
    --enable-pyside-extensions
    --generator-set=shiboken --enable-parent-ctor-heuristic
    --enable-return-value-heuristic --use-isnull-as-nb_nonzero
    --avoid-protected-hack
    --include-paths=${INCLUDE_PATH_WITH_COLONS}${SHIBOKEN_HELPER_INCLUDE_DIRS_WITH_COLONS}
    --typesystem-paths=${PYSIDE_TYPESYSTEMS}
    --output-directory=${BUILD_DIR}
    --language-level=c++17
    ${GLOBAL} ${TYPESYSTEM}
    )
endmacro()


#
# Run the Shiboken generator.
#
# :param PROJECT_NAME: The name of the shiboken project is only use for
#   the custom command comment
# :type PROJECT_NAME: string
# :param GLOBAL: the SIP file
# :type GLOBAL: string
# :param TYPESYSTEM: the typesystem file
# :type TYPESYSTEM: string
# :param WORKING_DIR: the working directory
# :type WORKING_DIR: string
# :param GENERATED_SRCS: the generated source files
# :type GENERATED_SRCS: list of strings
# :param HDRS: the processed header files
# :type HDRS: list of strings
# :param INCLUDE_PATH: the include path
# :type INCLUDE_PATH: list of strings
# :param BUILD_DIR: the build directory
# :type BUILD_DIR: string
#
function(shiboken_generator PROJECT_NAME GLOBAL TYPESYSTEM WORKING_DIR GENERATED_SRCS HDRS INCLUDE_PATH BUILD_DIR)
    _shiboken_generator_command(COMMAND "${GLOBAL}" "${TYPESYSTEM}" "${INCLUDE_PATH}" "${BUILD_DIR}")
    add_custom_command(
        OUTPUT ${GENERATED_SRCS}
        COMMAND ${COMMAND}
      DEPENDS ${GLOBAL} ${TYPESYSTEM} ${HDRS}
      WORKING_DIRECTORY ${WORKING_DIR}
      COMMENT "Running Shiboken generator for ${PROJECT_NAME} Python bindings..."
    )
endfunction()


#
# Add the Shiboken/PySide specific include directories.
#
# :param PROJECT_NAME: The namespace of the binding
# :type PROJECT_NAME: string
# :param QT_COMPONENTS: the Qt components
# :type QT_COMPONENTS: list of strings
#
function(shiboken_include_directories PROJECT_NAME QT_COMPONENTS)
    set(shiboken_INCLUDE_DIRECTORIES
        ${PYTHON_INCLUDE_DIR}
        ${SHIBOKEN_INCLUDE_DIR}
        ${PYSIDE_INCLUDE_DIR}
        ${PYSIDE_INCLUDE_DIR}/QtCore
        ${PYSIDE_INCLUDE_DIR}/QtGui
    )

    foreach(component ${QT_COMPONENTS})
        set(shiboken_INCLUDE_DIRECTORIES ${shiboken_INCLUDE_DIRECTORIES} ${PYSIDE_INCLUDE_DIR}/${component})
    endforeach()

    include_directories(${PROJECT_NAME} ${shiboken_INCLUDE_DIRECTORIES})
endfunction()


#
# Add the Shiboken/PySide specific link libraries.
#
# :param PROJECT_NAME: The target name of the binding library
# :type PROJECT_NAME: string
# :param QT_COMPONENTS: the Qt components
# :type QT_COMPONENTS: list of strings
#
function(shiboken_target_link_libraries PROJECT_NAME QT_COMPONENTS)
    set(shiboken_LINK_LIBRARIES
        ${SHIBOKEN_PYTHON_LIBRARIES}
        ${SHIBOKEN_LIBRARY}
        ${PYSIDE_LIBRARY}
    )

    foreach(component ${QT_COMPONENTS})
        string(TOUPPER ${component} component)
        set(shiboken_LINK_LIBRARIES ${shiboken_LINK_LIBRARIES} ${QT_${component}_LIBRARY})
    endforeach()

    target_link_libraries(${PROJECT_NAME} ${shiboken_LINK_LIBRARIES})
endfunction()
