/*
 * Copyright (c) 2017, Ellon Paiva Mendes @ LAAS-CNRS
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include "rviz_common/properties/covariance_property.hpp"

#include <memory>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz_rendering/objects/covariance_visual.hpp"

#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/enum_property.hpp"

namespace rviz_common
{
namespace properties
{

CovarianceProperty::CovarianceProperty(
  const QString & name,
  bool default_value,
  const QString & description,
  Property * parent,
  const char * changed_slot)
: BoolProperty(name, default_value, description, parent, changed_slot)
{
  position_property_ = new BoolProperty(
    "Position", true,
    "Whether or not to show the position part of covariances",
    this, changed_slot, parent);
  position_property_->setDisableChildrenIfFalse(true);

  position_color_property_ = new ColorProperty(
    "Color", QColor(204, 51, 204),
    "Color to draw the position covariance ellipse.",
    position_property_, changed_slot, parent);

  position_alpha_property_ = new FloatProperty(
    "Alpha", 0.3f,
    "0 is fully transparent, 1.0 is fully opaque.",
    position_property_, changed_slot, parent);
  position_alpha_property_->setMin(0);
  position_alpha_property_->setMax(1);

  position_scale_property_ = new FloatProperty(
    "Scale", 1.0f,
    "Scale factor to be applied to covariance ellipse. "
    "Corresponds to the number of standard deviations to display",
    position_property_, changed_slot, parent);
  position_scale_property_->setMin(0);

  orientation_property_ = new BoolProperty(
    "Orientation", true,
    "Whether or not to show the orientation part of covariances",
    this, changed_slot, parent);
  orientation_property_->setDisableChildrenIfFalse(true);

  orientation_frame_property_ = new EnumProperty(
    "Frame", "Local",
    "Frame used to display the orientation covariance.",
    orientation_property_, changed_slot, parent);
  orientation_frame_property_->addOption("Local", rviz_rendering::Local);
  orientation_frame_property_->addOption("Fixed", rviz_rendering::Fixed);

  orientation_colorstyle_property_ = new EnumProperty(
    "Color Style", "Unique",
    "Style to color the orientation covariance: "
    "XYZ with same unique color or following RGB order",
    orientation_property_, changed_slot, parent);
  orientation_colorstyle_property_->addOption("Unique", rviz_rendering::Unique);
  orientation_colorstyle_property_->addOption("RGB", rviz_rendering::RGB);
  connect(
    orientation_colorstyle_property_, SIGNAL(changed()),
    this, SLOT(updateColorStyleChoice()));

  orientation_color_property_ = new ColorProperty(
    "Color", QColor(255, 255, 127),
    "Color to draw the covariance ellipse.",
    orientation_property_, changed_slot, parent);

  orientation_alpha_property_ = new FloatProperty(
    "Alpha", 0.5f,
    "0 is fully transparent, 1.0 is fully opaque.",
    orientation_property_, changed_slot, parent);
  orientation_alpha_property_->setMin(0);
  orientation_alpha_property_->setMax(1);

  orientation_offset_property_ = new FloatProperty(
    "Offset", 1.0f,
    "For 3D poses: the distance where to position the ellipses representing "
    "orientation covariance. "
    "For 2D poses: the height of the triangle representing the variance on yaw",
    orientation_property_, changed_slot, parent);
  orientation_offset_property_->setMin(0);

  orientation_scale_property_ = new FloatProperty(
    "Scale", 1.0f,
    "Scale factor to be applied to orientation covariance shapes. "
    "Corresponds to the number of standard deviations to display",
    orientation_property_, changed_slot, parent);
  orientation_scale_property_->setMin(0);

  setDisableChildrenIfFalse(true);
}

void CovarianceProperty::updateColorStyleChoice()
{
  bool use_unique_color =
    (orientation_colorstyle_property_->getOptionInt() == rviz_rendering::Unique);
  orientation_color_property_->setHidden(!use_unique_color);
}

rviz_rendering::CovarianceUserData CovarianceProperty::getUserData()
{
  rviz_rendering::CovarianceUserData user_data;
  user_data.visible = getBool();

  user_data.position_visible = position_property_->getBool();
  auto position_color = position_color_property_->getColor();
  user_data.position_color = Ogre::ColourValue(
    position_color.redF(), position_color.greenF(), position_color.blueF(),
    position_alpha_property_->getFloat());
  user_data.position_scale = position_scale_property_->getFloat();

  user_data.orientation_visible = orientation_property_->getBool();
  user_data.orientation_frame = orientation_frame_property_->getOptionInt() ==
    rviz_rendering::Local ? rviz_rendering::Local : rviz_rendering::Fixed;
  user_data.orientation_color_style = orientation_colorstyle_property_->getOptionInt() ==
    rviz_rendering::Unique ? rviz_rendering::Unique : rviz_rendering::RGB;

  QColor ori_color = orientation_color_property_->getColor();
  user_data.orientation_color = Ogre::ColourValue(
    ori_color.redF(), ori_color.greenF(), ori_color.blueF(),
    orientation_alpha_property_->getFloat());
  user_data.orientation_scale = orientation_scale_property_->getFloat();
  user_data.orientation_offset = orientation_offset_property_->getFloat();
  return user_data;
}

}  // namespace properties
}  // namespace rviz_common
