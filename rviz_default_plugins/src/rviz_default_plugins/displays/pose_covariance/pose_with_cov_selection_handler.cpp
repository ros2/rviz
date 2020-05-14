/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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

#include "pose_with_cov_selection_handler.hpp"

#include <memory>

#include <OgreEntity.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/covariance_visual.hpp"

#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/interaction/selection_handler.hpp"

#include "rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp"

namespace rviz_default_plugins
{
namespace displays
{

PoseWithCovSelectionHandler::PoseWithCovSelectionHandler(
  PoseWithCovarianceDisplay * display, rviz_common::DisplayContext * context)
: SelectionHandler(context),
  display_(display)
{}

void PoseWithCovSelectionHandler::createProperties(
  const rviz_common::interaction::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  (void) obj;
  auto cat = new rviz_common::properties::Property(
    "Pose " + display_->getName(), QVariant(), "", parent_property);
  properties_.push_back(cat);

  frame_property_ = new rviz_common::properties::StringProperty(
    "Frame", "", "", cat);
  frame_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO, "", cat);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "", cat);
  orientation_property_->setReadOnly(true);

  covariance_position_property_ = new rviz_common::properties::VectorProperty(
    "Covariance Position", Ogre::Vector3::ZERO, "", cat);
  covariance_position_property_->setReadOnly(true);

  covariance_orientation_property_ = new rviz_common::properties::VectorProperty(
    "Covariance Orientation", Ogre::Vector3::ZERO, "", cat);
  covariance_orientation_property_->setReadOnly(true);
}

rviz_common::interaction::V_AABB
PoseWithCovSelectionHandler::getAABBs(const rviz_common::interaction::Picked & obj)
{
  (void) obj;
  rviz_common::interaction::V_AABB aabbs;
  if (display_->pose_valid_) {
    if (display_->shape_property_->getOptionInt() == PoseWithCovarianceDisplay::Arrow) {
      aabbs.push_back(display_->arrow_->getHead()->getEntity()->getWorldBoundingBox());
      aabbs.push_back(display_->arrow_->getShaft()->getEntity()->getWorldBoundingBox());
    } else {
      aabbs.push_back(display_->axes_->getXShape().getEntity()->getWorldBoundingBox());
      aabbs.push_back(display_->axes_->getYShape().getEntity()->getWorldBoundingBox());
      aabbs.push_back(display_->axes_->getZShape().getEntity()->getWorldBoundingBox());
    }

    if (display_->covariance_property_->getBool()) {
      if (display_->covariance_property_->getUserData().position_visible) {
        aabbs.push_back(display_->covariance_->getPositionBoundingBox());
      }
      if (display_->covariance_property_->getUserData().orientation_visible) {
        auto orientation_aabbs = display_->covariance_->getOrientationBoundingBoxes();
        aabbs.insert(aabbs.end(), std::begin(orientation_aabbs), std::end(orientation_aabbs));
      }
    }
  }
  return aabbs;
}

void PoseWithCovSelectionHandler::setMessage(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  // properties_.size() should only be > 0 after createProperties()
  // and before destroyProperties(), during which frame_property_,
  // position_property_, and orientation_property_ should be valid
  // pointers.
  if (!properties_.empty()) {
    frame_property_->setStdString(message->header.frame_id);
    position_property_->setVector(rviz_common::pointMsgToOgre(message->pose.pose.position));
    orientation_property_->setQuaternion(
      rviz_common::quaternionMsgToOgre(message->pose.pose.orientation));
    covariance_position_property_->setVector(
      Ogre::Vector3(
        message->pose.covariance[0 + 0 * 6],
        message->pose.covariance[1 + 1 * 6],
        message->pose.covariance[2 + 2 * 6]));

    covariance_orientation_property_->setVector(
      Ogre::Vector3(
        message->pose.covariance[3 + 3 * 6],
        message->pose.covariance[4 + 4 * 6],
        message->pose.covariance[5 + 5 * 6]));
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins
