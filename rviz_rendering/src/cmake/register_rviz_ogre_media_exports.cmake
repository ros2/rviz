# Copyright (c) 2017, Bosch Software Innovations GmbH.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Register ogre_media resources and install the folders
#
# :param DIRECTORIES: list of directories to be added as
#   rviz_ogre_media_exports. Directories will only be
#   added as shallow directories and must be relative
#   to CMAKE_CURRENT_SOURCE_DIR.
# :type DIRECTORIES: string (muliple strings possible, relative paths)

function(register_rviz_ogre_media_exports)
  cmake_parse_arguments(ARGUMENTS "" "" "DIRECTORIES" ${ARGN})
  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "PROJECT_NAME not set. You must call project() before adding resources")
  endif()

  foreach(DIR ${ARGUMENTS_DIRECTORIES})
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${DIR}")
      message(FATAL_ERROR "Directory ${DIR} does not exist.
        Paths must be specified relative to CMAKE_CURRENT_SOURCE_DIR
        ${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
    list(APPEND OGRE_MEDIA_RESOURCE_DIRS ${DIR})
  endforeach()
  set(OGRE_MEDIA_RESOURCE_DIRS ${OGRE_MEDIA_RESOURCE_DIRS} PARENT_SCOPE)
endfunction()
