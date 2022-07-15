/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
#ifndef RVIZ_COMMON__MSG_CONVERSIONS_HPP_
#define RVIZ_COMMON__MSG_CONVERSIONS_HPP_

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace rviz_common
{

// This file contains some convenience functions for Ogre / geometry_msgs conversions.

static inline Ogre::Vector3 pointMsgToOgre(const geometry_msgs::msg::Point & m)
{
  return Ogre::Vector3(m.x, m.y, m.z);
}

static inline Ogre::Vector3 vector3MsgToOgre(const geometry_msgs::msg::Vector3 & m)
{
  return Ogre::Vector3(m.x, m.y, m.z);
}

static inline Ogre::Quaternion quaternionMsgToOgre(const geometry_msgs::msg::Quaternion & m)
{
  return Ogre::Quaternion(m.w, m.x, m.y, m.z);
}

static inline geometry_msgs::msg::Point pointOgreToMsg(const Ogre::Vector3 & o)
{
  geometry_msgs::msg::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

static inline geometry_msgs::msg::Vector3 vector3OgreToMsg(const Ogre::Vector3 & o)
{
  geometry_msgs::msg::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

static inline geometry_msgs::msg::Quaternion quaternionOgreToMsg(const Ogre::Quaternion & o)
{
  geometry_msgs::msg::Quaternion m;
  m.w = o.w; m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}

}  // namespace rviz_common

#endif  // RVIZ_COMMON__MSG_CONVERSIONS_HPP_
