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

#include "rviz_default_plugins/displays/map/q_map_display_object.hpp"

namespace rviz_default_plugins
{
namespace displays
{

QMapDisplayObject::QMapDisplayObject(rviz_common::properties::Property * parent)
: QObject(static_cast<QObject *>(parent)), update_profile_(rclcpp::QoS(5))
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  update_topic_property_ = new rviz_common::properties::RosTopicProperty(
    "Update Topic", "",
    "",
    "Topic where updates to this map display are received. "
    "This topic is automatically determined by the map topic. "
    "If the map is received on 'map_topic', the display assumes updates are received on "
    "'map_topic_updates'."
    "This can be overridden in the UI by clicking on the topic and setting the desired topic.",
    parent, SLOT(updateMapUpdateTopic()), this);

  update_profile_property_ = new rviz_common::properties::QosProfileProperty(
    update_topic_property_);

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.7f,
    "Amount of transparency to apply to the map.",
    parent, SLOT(updateAlpha()), this);
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);


  draw_under_property_ = new rviz_common::properties::BoolProperty(
    "Draw Behind", false,
    "Rendering option, controls whether or not the map is always"
    " drawn behind everything else.",
    parent, SLOT(updateDrawUnder()), this);

  resolution_property_ = new rviz_common::properties::FloatProperty(
    "Resolution", 0,
    "Resolution of the map. (not editable)", parent);
  resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 0,
    "Width of the map, in meters. (not editable)", parent);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 0,
    "Height of the map, in meters. (not editable)", parent);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of the bottom left corner of the map, in meters. (not editable)",
    parent);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, "Orientation of the map. (not editable)", parent);
  orientation_property_->setReadOnly(true);

  transform_timestamp_property_ = new rviz_common::properties::BoolProperty(
    "Use Timestamp", false,
    "Use map header timestamp when transforming", parent, SLOT(transformMap()), this);
}

void QMapDisplayObject::showMap()
{
  if (showMap_) {
    showMap_();
  }
}

void QMapDisplayObject::updateAlpha()
{
  if (updateAlpha_) {
    updateAlpha_();
  }
}

void QMapDisplayObject::updateDrawUnder()
{
  if (updateDrawUnder_) {
    updateDrawUnder_();
  }
}

void QMapDisplayObject::transformMap()
{
  if (transformMap_) {
    transformMap_();
  }
}

void QMapDisplayObject::updateMapUpdateTopic()
{
  if (updateMapUpdateTopic_) {
    updateMapUpdateTopic_();
  }
}

}  // namespace displays
}  // namespace rviz_default_plugins
