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

#include "rviz_default_plugins/displays/marker/markers/marker_selection_handler.hpp"

#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "rviz_common/msg_conversions.hpp"
#include "rviz_default_plugins/displays/marker/marker_display.hpp"
#include "rviz_default_plugins/displays/marker/markers/marker_base.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/quaternion_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{
namespace markers
{

MarkerSelectionHandler::MarkerSelectionHandler(
  const MarkerBase * marker, MarkerID id, rviz_common::DisplayContext * context)
: SelectionHandler(context),
  marker_(marker),
  marker_id_(QString::fromStdString(id.first) + "/" + QString::number(id.second))
{
}

MarkerSelectionHandler::~MarkerSelectionHandler()
{
}

Ogre::Vector3 MarkerSelectionHandler::getPosition()
{
  return rviz_common::pointMsgToOgre(marker_->getMessage()->pose.position);
}

Ogre::Quaternion MarkerSelectionHandler::getOrientation()
{
  return rviz_common::quaternionMsgToOgre(marker_->getMessage()->pose.orientation);
}

void MarkerSelectionHandler::createProperties(
  const rviz_common::interaction::Picked & obj, rviz_common::properties::Property * parent_property)
{
  (void) obj;

  rviz_common::properties::Property * group = new rviz_common::properties::Property(
    "Marker " + marker_id_, QVariant(), "", parent_property);
  properties_.push_back(group);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", getPosition(), "", group);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", getOrientation(), "", group);
  orientation_property_->setReadOnly(true);

  group->expand();
}

void MarkerSelectionHandler::updateProperties()
{
  position_property_->setVector(getPosition() );
  orientation_property_->setQuaternion(getOrientation() );
}

}  // namespace markers
}  // namespace displays
}  // namespace rviz_default_plugins
