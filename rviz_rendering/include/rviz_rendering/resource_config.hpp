/*
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_RENDERING__RESOURCE_CONFIG_HPP_
#define RVIZ_RENDERING__RESOURCE_CONFIG_HPP_

#include <string>

#include "rviz_rendering/visibility_control.hpp"

namespace rviz_rendering
{

/// Return the absolute path to the rviz_rendering resource directory.
RVIZ_RENDERING_PUBLIC
std::string
get_resource_directory();

/// Set the absolute path to the rviz_rendering resource directory.
/**
 * Should be called before initializing the rendering system, as this directory
 * is used to load configurations and other resources during setup.
 * This function is not thread-safe.
 */
RVIZ_RENDERING_PUBLIC
void
set_resource_directory(const std::string & resource_directory);

/// Return the absolute path to the Ogre plugin directory.
RVIZ_RENDERING_PUBLIC
std::string
get_ogre_plugin_directory();

/// Set the absolute path to the Ogre plugin directory.
/**
 * Should be called before initializing the rendering system, as this directory
 * is used to load configurations and other resources during setup.
 * This function is not thread-safe.
 */
RVIZ_RENDERING_PUBLIC
void
set_ogre_plugin_directory(const std::string & ogre_plugin_directory);

}  // namespace rviz_rendering

#endif  // RVIZ_RENDERING__RESOURCE_CONFIG_HPP_
