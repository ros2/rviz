/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "rviz_default_plugins/displays/pointcloud/point_cloud_selection_handler.hpp"

#include <memory>
#include <set>
#include <string>
#include <utility>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>

#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

namespace rviz_default_plugins
{

uint64_t qHash(IndexAndMessage iam)
{
  return iam.index + (iam.message >> 32) + (iam.message & 0xffffffff);
}

bool operator==(IndexAndMessage a, IndexAndMessage b)
{
  return a.index == b.index && a.message == b.message;
}

PointCloudSelectionHandler::PointCloudSelectionHandler(
  float box_size,
  CloudInfo * cloud_info,
  rviz_common::DisplayContext * context)
: SelectionHandler(context),
  cloud_info_(cloud_info),
  box_size_(box_size)
{
}

PointCloudSelectionHandler::~PointCloudSelectionHandler()
{
  for (auto property_hash : property_hash_) {
    delete property_hash;
  }
}

void PointCloudSelectionHandler::preRenderPass(uint32_t pass)
{
  rviz_common::interaction::SelectionHandler::preRenderPass(pass);

  switch (pass) {
    case 0:
      cloud_info_->cloud_->setPickColor(
        rviz_common::interaction::SelectionManager::handleToColor(
          getHandle()));
      break;
    case 1:
      cloud_info_->cloud_->setColorByIndex(true);
      break;
    default:
      break;
  }
}

void PointCloudSelectionHandler::postRenderPass(uint32_t pass)
{
  rviz_common::interaction::SelectionHandler::postRenderPass(pass);

  if (pass == 1) {
    cloud_info_->cloud_->setColorByIndex(false);
  }
}

void PointCloudSelectionHandler::createProperties(
  const rviz_common::interaction::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  S_int indices = getIndicesOfSelectedPoints(obj);

  for (auto index : indices) {
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message = cloud_info_->message_;

    IndexAndMessage hash_key(index, message.get());
    if (!property_hash_.contains(hash_key)) {
      rviz_common::properties::Property * parent = createParentPropertyForPoint(
        parent_property, index, message);
      property_hash_.insert(hash_key, parent);

      addPositionProperty(parent, index);

      addAdditionalProperties(parent, index, message);
    }
  }
}

void PointCloudSelectionHandler::destroyProperties(
  const rviz_common::interaction::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  (void) parent_property;
  S_int indices = getIndicesOfSelectedPoints(obj);
  for (auto index : indices) {
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message = cloud_info_->message_;

    IndexAndMessage hash_key(index, message.get());

    rviz_common::properties::Property * prop = property_hash_.take(hash_key);
    delete prop;
  }
}


rviz_common::interaction::V_AABB PointCloudSelectionHandler::getAABBs(
  const rviz_common::interaction::Picked & obj)
{
  rviz_common::interaction::V_AABB aabbs;
  for (auto handle : obj.extra_handles) {
    auto find_it = boxes_.find(Handles(obj.handle, handle - 1));
    if (find_it != boxes_.end()) {
      Ogre::WireBoundingBox * box = find_it->second.box;

      aabbs.push_back(box->getWorldBoundingBox(true));  // calculate bounding boxes if absent
    }
  }
  return aabbs;
}

void PointCloudSelectionHandler::onSelect(const rviz_common::interaction::Picked & obj)
{
  for (auto handle : obj.extra_handles) {
    uint64_t index = handleToIndex(handle);

    sensor_msgs::msg::PointCloud2::ConstSharedPtr message = cloud_info_->message_;

    Ogre::Vector3 pos = cloud_info_->transformed_points_[index].position;
    pos = cloud_info_->scene_node_->convertLocalToWorldPosition(pos);

    float size = box_size_ * 0.5f;

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(Handles(obj.handle, index), aabb, "RVIZ/Cyan");
  }
}

void PointCloudSelectionHandler::onDeselect(const rviz_common::interaction::Picked & obj)
{
  for (auto handle : obj.extra_handles) {
    destroyBox(Handles(obj.handle, handleToIndex(handle)));
  }
}

rviz_common::properties::Property * PointCloudSelectionHandler::createParentPropertyForPoint(
  rviz_common::properties::Property * parent_property,
  uint64_t index,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
{
  return new rviz_common::properties::Property(
    QString("Point %1 [cloud 0x%2]").arg(index).arg((uint64_t) message.get()),
    QVariant(),
    "",
    parent_property);
}

void PointCloudSelectionHandler::addPositionProperty(
  rviz_common::properties::Property * parent, uint64_t index) const
{
  rviz_common::properties::VectorProperty * pos_prop =
    new rviz_common::properties::VectorProperty(
    "Position",
    cloud_info_->transformed_points_[index].position,
    "",
    parent);
  pos_prop->setReadOnly(true);
  // suppressing this memleak warning from cppcheck below
  // cppcheck-suppress memleak
}

void PointCloudSelectionHandler::addAdditionalProperties(
  rviz_common::properties::Property * parent,
  uint64_t index,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const
{
  for (size_t field = 0; field < message->fields.size(); ++field) {
    const sensor_msgs::msg::PointField & f = message->fields[field];
    const std::string & name = f.name;

    if (name == "x" || name == "y" || name == "z" || name == "X" || name == "Y" ||
      name == "Z")
    {
      continue;
    }
    if (name == "rgb" || name == "rgba") {
      uint32_t val = convertValueToColor(index, message, f);
      addColorProperty(parent, field, name, val);

      addAlphaProperty(parent, val);
    } else {
      auto val = valueFromCloud<float>(
        message, f.offset, f.datatype, message->point_step, index);
      addIntensityProperty(parent, field, name, val);
    }
  }
}

uint32_t PointCloudSelectionHandler::convertValueToColor(
  uint64_t index,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
  const sensor_msgs::msg::PointField & f) const
{
  auto float_val = valueFromCloud<float>(
    message, f.offset, f.datatype, message->point_step, index);
  // Conversion hack because rgb are stored in float (datatype=7) and valueFromCloud
  // can't cast float to uint32_t
  float * float_val_ptr = &float_val;
  uint32_t val = *reinterpret_cast<uint32_t *>(float_val_ptr);
  return val;
}

void PointCloudSelectionHandler::addColorProperty(
  rviz_common::properties::Property * parent,
  size_t field,
  const std::string & name,
  uint32_t val) const
{
  rviz_common::properties::ColorProperty * prop =
    new rviz_common::properties::ColorProperty(
    QString("%1: %2").arg(field).arg(QString::fromStdString(name)),
    QColor((val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff),
    "",
    parent);
  prop->setReadOnly(true);
}

void PointCloudSelectionHandler::addAlphaProperty(
  rviz_common::properties::Property * parent, uint32_t val) const
{
  rviz_common::properties::FloatProperty * aprop =
    new rviz_common::properties::FloatProperty(
    QString("alpha"), ((val >> 24) / 255.0), "", parent);
  aprop->setReadOnly(true);
}

void PointCloudSelectionHandler::addIntensityProperty(
  rviz_common::properties::Property * parent,
  size_t field,
  const std::string & name,
  float val) const
{
  rviz_common::properties::FloatProperty * prop =
    new rviz_common::properties::FloatProperty(
    QString("%1: %2").arg(field).arg(QString::fromStdString(name)),
    val,
    "",
    parent);
  prop->setReadOnly(true);
}
}  // namespace rviz_default_plugins
