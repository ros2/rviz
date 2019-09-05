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

#include "rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp"

#include <memory>

#include <OgreEntity.h>

#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/objects/covariance_visual.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/covariance_property.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{
namespace displays
{

class PoseWithCovarianceDisplaySelectionHandler : public rviz_common::interaction::SelectionHandler
{
public:
  PoseWithCovarianceDisplaySelectionHandler(
    PoseWithCovarianceDisplay * display, rviz_common::DisplayContext * context)
  : SelectionHandler(context),
    display_(display)
  {}

  void createProperties(
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

  void getAABBs(
    const rviz_common::interaction::Picked & obj,
    rviz_common::interaction::V_AABB & aabbs)
  {
    (void) obj;
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
  }

  void setMessage(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
  {
    // properties_.size() should only be > 0 after createProperties()
    // and before destroyProperties(), during which frame_property_,
    // position_property_, and orientation_property_ should be valid
    // pointers.
    if (properties_.size() > 0) {
      frame_property_->setStdString(message->header.frame_id);
      position_property_->setVector(Ogre::Vector3(message->pose.pose.position.x,
        message->pose.pose.position.y,
        message->pose.pose.position.z));
      orientation_property_->setQuaternion(Ogre::Quaternion(message->pose.pose.orientation.w,
        message->pose.pose.orientation.x,
        message->pose.pose.orientation.y,
        message->pose.pose.orientation.z));
      covariance_position_property_->setVector(Ogre::Vector3(message->pose.covariance[0 + 0 * 6],
        message->pose.covariance[1 + 1 * 6],
        message->pose.covariance[2 + 2 * 6]));

      covariance_orientation_property_->setVector(Ogre::Vector3(message->pose.covariance[3 + 3 * 6],
        message->pose.covariance[4 + 4 * 6],
        message->pose.covariance[5 + 5 * 6]));
    }
  }

private:
  PoseWithCovarianceDisplay * display_;
  rviz_common::properties::StringProperty * frame_property_;
  rviz_common::properties::VectorProperty * position_property_;
  rviz_common::properties::QuaternionProperty * orientation_property_;
  rviz_common::properties::VectorProperty * covariance_position_property_;
  rviz_common::properties::VectorProperty * covariance_orientation_property_;
};

PoseWithCovarianceDisplay::PoseWithCovarianceDisplay()
: pose_valid_(false)
{
  shape_property_ = new rviz_common::properties::EnumProperty(
    "Shape", "Arrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
  shape_property_->addOption("Arrow", Arrow);
  shape_property_->addOption("Axes", Axes);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(255, 25, 0), "Color to draw the arrow.",
    this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1, "Amount of transparency to apply to the arrow.",
    this, SLOT(updateColorAndAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Length", 1, "Length of the arrow's shaft, in meters.",
    this, SLOT(updateArrowGeometry()));

  shaft_radius_property_ = new rviz_common::properties::FloatProperty(
    "Shaft Radius", 0.05f, "Radius of the arrow's shaft, in meters.",
    this, SLOT(updateArrowGeometry()));

  head_length_property_ = new rviz_common::properties::FloatProperty(
    "Head Length", 0.3f, "Length of the arrow's head, in meters.",
    this, SLOT(updateArrowGeometry()));

  head_radius_property_ = new rviz_common::properties::FloatProperty(
    "Head Radius", 0.1f, "Radius of the arrow's head, in meters.",
    this, SLOT(updateArrowGeometry()));

  axes_length_property_ = new rviz_common::properties::FloatProperty(
    "Axes Length", 1, "Length of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
    "Axes Radius", 0.1f, "Radius of each axis, in meters.", this, SLOT(updateAxisGeometry()));

  covariance_property_ = new rviz_common::properties::CovarianceProperty(
    "Covariance", true, "Whether or not the covariances of the messages should be shown.",
    this, SLOT(updateCovariance()));
}

void PoseWithCovarianceDisplay::onInitialize()
{
  MFDClass::onInitialize();

  arrow_ = std::make_shared<rviz_rendering::Arrow>(
    scene_manager_, scene_node_,
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  arrow_->setOrientation(Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

  axes_ = std::make_shared<rviz_rendering::Axes>(
    scene_manager_, scene_node_, axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());


  covariance_ = std::make_unique<rviz_rendering::CovarianceVisual>(
    scene_manager_, scene_node_->createChildSceneNode());

  updateShapeChoice();
  updateColorAndAlpha();

  coll_handler_.reset(new PoseWithCovarianceDisplaySelectionHandler(this, context_));
  coll_handler_->addTrackedObjects(arrow_->getSceneNode());
  coll_handler_->addTrackedObjects(axes_->getSceneNode());
  coll_handler_->addTrackedObjects(covariance_->getPositionSceneNode());
  coll_handler_->addTrackedObjects(covariance_->getOrientationSceneNode());
}

PoseWithCovarianceDisplay::~PoseWithCovarianceDisplay() = default;

void PoseWithCovarianceDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
}

void PoseWithCovarianceDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  arrow_->setColor(color);

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateArrowGeometry()
{
  arrow_->set(shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateAxisGeometry()
{
  axes_->set(axes_length_property_->getFloat(),
    axes_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateShapeChoice()
{
  bool use_arrow = (shape_property_->getOptionInt() == Arrow);

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);
  shaft_length_property_->setHidden(!use_arrow);
  shaft_radius_property_->setHidden(!use_arrow);
  head_length_property_->setHidden(!use_arrow);
  head_radius_property_->setHidden(!use_arrow);

  axes_length_property_->setHidden(use_arrow);
  axes_radius_property_->setHidden(use_arrow);

  updateShapeVisibility();

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateShapeVisibility()
{
  if (!pose_valid_) {
    arrow_->getSceneNode()->setVisible(false);
    axes_->getSceneNode()->setVisible(false);
    covariance_->setVisible(false);
  } else {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    arrow_->getSceneNode()->setVisible(use_arrow);
    axes_->getSceneNode()->setVisible(!use_arrow);
    covariance_->updateUserData(covariance_property_->getUserData());
  }
}

void PoseWithCovarianceDisplay::updateCovariance() {
  covariance_->updateUserData(covariance_property_->getUserData());
}

void PoseWithCovarianceDisplay::processMessage(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->pose.pose) ||
    !rviz_common::validateFloats(message->pose.covariance))
  {
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(message->header, message->pose.pose, position,
    orientation))
  {
    RVIZ_COMMON_LOG_ERROR_STREAM("Error transforming pose " << getNameStd() << " from frame '" <<
      message->header.frame_id << "' to frame '" << fixed_frame_.toStdString() << "'");
    return;
  }

  pose_valid_ = true;
  updateShapeVisibility();

  axes_->setPosition(position);
  axes_->setOrientation(orientation);

  arrow_->setPosition(position);
  arrow_->setOrientation(orientation * Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));

  covariance_->setPosition(position);
  covariance_->setOrientation(orientation);
  covariance_->setCovariance(
    rviz_common::quaternionMsgToOgre(message->pose.pose.orientation), message->pose.covariance);

  coll_handler_->setMessage(message);

  context_->queueRender();
}

void PoseWithCovarianceDisplay::reset()
{
  MFDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}


}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT: cpplint doesn't like include order
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::displays::PoseWithCovarianceDisplay,
  rviz_common::Display)
