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

if(NOT "${OGRE_MEDIA_RESOURCE_DIRS}" STREQUAL "")
  message(STATUS "Deploy OGRE media")
  foreach(DIR ${OGRE_MEDIA_RESOURCE_DIRS})
    set(OGRE_MEDIA_RESOURCE_FILE "${OGRE_MEDIA_RESOURCE_FILE}${PROJECT_NAME}/${DIR}\n")

    set(_destination "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
    # in case that DIR is not only a directory name but a path
    # its dirname must be appended to the destination to keep the same relative path
    get_filename_component(_dirname "${DIR}" DIRECTORY)
    if(NOT "${_dirname}" STREQUAL "")
      set(_destination "${_destination}/${_dirname}")
    endif()

    install(DIRECTORY ${DIR}
      DESTINATION "${_destination}"
      USE_SOURCE_PERMISSIONS)
  endforeach()
  ament_index_register_resource(rviz_ogre_media_exports CONTENT ${OGRE_MEDIA_RESOURCE_FILE})
endif()
