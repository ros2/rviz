/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_BASE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_BASE_HPP_

#include <memory>
#include <set>
#include <string>
#include <utility>

#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
class Vector3;
class Quaternion;
class Entity;
}  // namespace Ogre

namespace rviz_common
{
class DisplayContext;
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{
class MarkerCommon;

namespace markers
{
class MarkerSelectionHandler;

typedef std::pair<std::string, int32_t> MarkerID;
typedef std::set<Ogre::MaterialPtr> S_MaterialPtr;

class MarkerBase
{
public:
  typedef visualization_msgs::msg::Marker Marker;
  typedef visualization_msgs::msg::Marker::ConstSharedPtr MarkerConstSharedPtr;

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  MarkerBase(
    MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  virtual ~MarkerBase();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setMessage(const Marker & message);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void setMessage(const MarkerConstSharedPtr & message);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  bool expired();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  void updateFrameLocked();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const MarkerConstSharedPtr & getMessage() const {return message_;}

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  MarkerID getID() {return MarkerID(message_->ns, message_->id);}

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  std::string getStringID()
  {
    return message_->ns + "/" + std::to_string(message_->id);
  }

  // TODO(Martin-Idel-SI): use again when interactive marker is ported
  /** @brief Associate an InteractiveObject with this MarkerBase. */
  // void setInteractiveObject(InteractiveObjectWPtr object);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  virtual void setPosition(const Ogre::Vector3 & position);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  virtual void setOrientation(const Ogre::Quaternion & orientation);

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const Ogre::Vector3 & getPosition();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  const Ogre::Quaternion & getOrientation();

  RVIZ_DEFAULT_PLUGINS_PUBLIC
  virtual S_MaterialPtr getMaterials() {return S_MaterialPtr();}

protected:
  bool transform(
    const MarkerConstSharedPtr & message,
    Ogre::Vector3 & pos,
    Ogre::Quaternion & orient,
    Ogre::Vector3 & scale);
  virtual void onNewMessage(
    const MarkerConstSharedPtr & old_message,
    const MarkerConstSharedPtr & new_message) = 0;

  void extractMaterials(Ogre::Entity * entity, S_MaterialPtr & materials);

  MarkerCommon * owner_;
  rviz_common::DisplayContext * context_;

  Ogre::SceneNode * scene_node_;

  MarkerConstSharedPtr message_;

  rclcpp::Time expiration_;

  std::shared_ptr<MarkerSelectionHandler> handler_;
};

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_BASE_HPP_
