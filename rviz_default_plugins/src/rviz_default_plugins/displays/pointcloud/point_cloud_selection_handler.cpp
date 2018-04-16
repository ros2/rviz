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

#include "point_cloud_selection_handler.hpp"
#include <memory>
#include <set>
#include <string>
#include <utility>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreWireBoundingBox.h>
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// #include <tf/transform_listener.h>

// TODO(wjwwood): revist file when pluginlib is available
// #include <pluginlib/class_loader.h>

#include "./point_cloud_transformer.hpp"
#include "point_cloud_helpers.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/uniform_string_stream.hpp"
#include "rviz_common/validate_floats.hpp"
#include "./point_cloud_common.hpp"

namespace rviz_default_plugins
{

uint qHash(IndexAndMessage iam)
{
  return
    ((uint) iam.index) +
    ((uint) (iam.message >> 32)) +
    ((uint) (iam.message & 0xffffffff));
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
  // delete all the Property objects on our way out.
  QHash<IndexAndMessage, rviz_common::properties::Property *>::const_iterator iter;
  for (iter = property_hash_.begin(); iter != property_hash_.end(); iter++) {
    delete iter.value();
  }
}

void PointCloudSelectionHandler::preRenderPass(uint32_t pass)
{
  rviz_common::selection::SelectionHandler::preRenderPass(pass);

  switch (pass) {
    case 0:
      cloud_info_->cloud_->setPickColor(rviz_common::selection::SelectionManager::handleToColor(
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
  rviz_common::selection::SelectionHandler::postRenderPass(pass);

  if (pass == 1) {
    cloud_info_->cloud_->setColorByIndex(false);
  }
}

Ogre::Vector3 pointFromCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t index)
{
  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint8_t type = cloud->fields[xi].datatype;
  const uint32_t point_step = cloud->point_step;
  float x = valueFromCloud<float>(cloud, xoff, type, point_step, index);
  float y = valueFromCloud<float>(cloud, yoff, type, point_step, index);
  float z = valueFromCloud<float>(cloud, zoff, type, point_step, index);
  return Ogre::Vector3(x, y, z);
}

void PointCloudSelectionHandler::createProperties(
  const rviz_common::selection::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  typedef std::set<int> S_int;
  S_int indices;
  {
    rviz_common::selection::S_uint64::const_iterator it = obj.extra_handles.begin();
    rviz_common::selection::S_uint64::const_iterator end = obj.extra_handles.end();
    for (; it != end; ++it) {
      uint64_t handle = *it;
      indices.insert((handle & 0xffffffff) - 1);
    }
  }

  {
    S_int::iterator it = indices.begin();
    S_int::iterator end = indices.end();
    for (; it != end; ++it) {
      int index = *it;
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message = cloud_info_->message_;

      IndexAndMessage hash_key(index, message.get());
      if (!property_hash_.contains(hash_key)) {
        rviz_common::properties::Property * cat = new rviz_common::properties::Property(
          QString("Point %1 [cloud 0x%2]").arg(index).arg((uint64_t) message.get()),
          QVariant(),
          "",
          parent_property);
        property_hash_.insert(hash_key, cat);

        // First add the position.
        rviz_common::properties::VectorProperty * pos_prop =
          new rviz_common::properties::VectorProperty(
          "Position",
          cloud_info_->transformed_points_[index].position,
          "",
          cat);
        pos_prop->setReadOnly(true);

        // Then add all other fields as well.
        for (size_t field = 0; field < message->fields.size(); ++field) {
          const sensor_msgs::msg::PointField & f = message->fields[field];
          const std::string & name = f.name;

          if (name == "x" || name == "y" || name == "z" || name == "X" || name == "Y" ||
            name == "Z")
          {
            continue;
          }
          if (name == "rgb" || name == "rgba") {
            float float_val = valueFromCloud<float>(message, f.offset, f.datatype,
                message->point_step, index);
            // Convertion hack because rgb are stored int float (datatype=7) and valueFromCloud
            // can't cast float to uint32_t
            float * float_val_ptr = &float_val;
            uint32_t val = *reinterpret_cast<uint32_t *>(float_val_ptr);
            rviz_common::properties::ColorProperty * prop =
              new rviz_common::properties::ColorProperty(
              QString("%1: %2").arg(field).arg(QString::fromStdString(name)),
              QColor((val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff),
              "",
              cat);
            prop->setReadOnly(true);

            rviz_common::properties::FloatProperty * aprop =
              new rviz_common::properties::FloatProperty(
              QString("alpha"), ((val >> 24) / 255.0), "", cat);
            aprop->setReadOnly(true);
          } else {
            float val = valueFromCloud<float>(message, f.offset, f.datatype, message->point_step,
                index);
            rviz_common::properties::FloatProperty * prop =
              new rviz_common::properties::FloatProperty(
              QString("%1: %2").arg(field).arg(QString::fromStdString(name)),
              val,
              "",
              cat);
            prop->setReadOnly(true);
          }
        }
        // suppressing this memleak warning from cppcheck below
        // cppcheck-suppress memleak
      }
    }
  }
}

void PointCloudSelectionHandler::destroyProperties(
  const rviz_common::selection::Picked & obj,
  rviz_common::properties::Property * parent_property)
{
  (void) parent_property;
  typedef std::set<int> S_int;
  S_int indices;
  {
    rviz_common::selection::S_uint64::const_iterator it = obj.extra_handles.begin();
    rviz_common::selection::S_uint64::const_iterator end = obj.extra_handles.end();
    for (; it != end; ++it) {
      uint64_t handle = *it;
      indices.insert((handle & 0xffffffff) - 1);
    }
  }

  {
    S_int::iterator it = indices.begin();
    S_int::iterator end = indices.end();
    for (; it != end; ++it) {
      int index = *it;
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message = cloud_info_->message_;

      IndexAndMessage hash_key(index, message.get());

      rviz_common::properties::Property * prop = property_hash_.take(hash_key);
      delete prop;
    }
  }
}

rviz_common::selection::V_AABB PointCloudSelectionHandler::getAABBs(
  const rviz_common::selection::Picked & obj)
{
  rviz_common::selection::V_AABB aabbs;
  rviz_common::selection::S_uint64::iterator it = obj.extra_handles.begin();
  rviz_common::selection::S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it) {
    M_HandleToBox::iterator find_it = boxes_.find(Handles(obj.handle, *it - 1));
    if (find_it != boxes_.end()) {
      Ogre::WireBoundingBox * box = find_it->second.box;

      aabbs.push_back(box->getWorldBoundingBox());
    }
  }
  return aabbs;
}

void PointCloudSelectionHandler::onSelect(const rviz_common::selection::Picked & obj)
{
  rviz_common::selection::S_uint64::iterator it = obj.extra_handles.begin();
  rviz_common::selection::S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it) {
    int index = (*it & 0xffffffff) - 1;

    sensor_msgs::msg::PointCloud2::ConstSharedPtr message = cloud_info_->message_;

    Ogre::Vector3 pos = cloud_info_->transformed_points_[index].position;
    pos = cloud_info_->scene_node_->convertLocalToWorldPosition(pos);

    float size = box_size_ * 0.5f;

    Ogre::AxisAlignedBox aabb(pos - size, pos + size);

    createBox(Handles(obj.handle, index), aabb, "RVIZ/Cyan");
  }
}

void PointCloudSelectionHandler::onDeselect(const rviz_common::selection::Picked & obj)
{
  rviz_common::selection::S_uint64::iterator it = obj.extra_handles.begin();
  rviz_common::selection::S_uint64::iterator end = obj.extra_handles.end();
  for (; it != end; ++it) {
    int global_index = (*it & 0xffffffff) - 1;

    destroyBox(Handles(obj.handle, global_index));
  }
}

}  // namespace rviz_default_plugins
