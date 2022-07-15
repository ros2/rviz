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

#include <OgreVector.h>

#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interactive_object.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

// This is necessary because of using stl types with this display. Nevertheless, if you are
// experiencing problems when subclassing this class, please make sure ROS2 and your code were
// compiled with the same compiler and version
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace Ogre
{
class SceneNode;
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

class RVIZ_DEFAULT_PLUGINS_PUBLIC MarkerBase
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerConstSharedPtr = visualization_msgs::msg::Marker::ConstSharedPtr;

  using SharedPtr = std::shared_ptr<MarkerBase>;

  MarkerBase(
    MarkerCommon * owner, rviz_common::DisplayContext * context, Ogre::SceneNode * parent_node);

  virtual ~MarkerBase();

  void setMessage(const Marker & message);

  void setMessage(const MarkerConstSharedPtr & message);

  bool expired();

  void updateFrameLocked();

  const MarkerConstSharedPtr & getMessage() const {return message_;}

  MarkerID getID() {return MarkerID(message_->ns, message_->id);}

  std::string getStringID()
  {
    return message_->ns + "/" + std::to_string(message_->id);
  }

  /// Associate an InteractiveObject with this MarkerBase.
  void setInteractiveObject(rviz_common::InteractiveObjectWPtr object);

  virtual void setPosition(const Ogre::Vector3 & position);

  virtual void setOrientation(const Ogre::Quaternion & orientation);

  const Ogre::Vector3 & getPosition();

  const Ogre::Quaternion & getOrientation();

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

#ifdef _WIN32
# pragma warning(pop)
#endif

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__MARKER__MARKERS__MARKER_BASE_HPP_
