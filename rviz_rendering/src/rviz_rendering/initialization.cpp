/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "rviz_rendering/initialization.hpp"

#include <exception>
#include <stdexcept>

#include <OgreRenderSystem.h>
#include <OgreRoot.h>

#include "ros/package.h"

namespace rviz_rendering
{

void cleanupOgre()
{
  delete Ogre::Root::getSingletonPtr();
}

// This should be folded into RenderSystem, but it should work fine as
// is, so I'm leaving it for now.
void initializeResources(const V_string & resource_paths)
{
  V_string::const_iterator path_it = resource_paths.begin();
  V_string::const_iterator path_end = resource_paths.end();
  for (; path_it != path_end; ++path_it) {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(*path_it, "FileSystem",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}

}  // namespace rviz_rendering
