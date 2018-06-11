/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <algorithm>
#include <vector>

#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t AxisColorPCTransformer::supports(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  return PointCloudTransformer::Support_Color;
}

uint8_t AxisColorPCTransformer::score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  return 255;
}

bool AxisColorPCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  if (!(mask & PointCloudTransformer::Support_Color)) {
    return false;
  }

  int32_t xi = findChannelIndex(cloud, "x");
  int32_t yi = findChannelIndex(cloud, "y");
  int32_t zi = findChannelIndex(cloud, "z");

  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;
  uint8_t const * point = &cloud->data.front();

  // Fill a vector of floats with values based on the chosen axis.
  int axis = axis_property_->getOptionInt();
  std::vector<float> values;
  values.reserve(num_points);
  Ogre::Vector3 pos;
  if (use_fixed_frame_property_->getBool()) {
    for (uint32_t i = 0; i < num_points; ++i, point += point_step) {
      // TODO(anonymous): optimize this by only doing the multiplication needed
      // for the desired output value, instead of doing all of them
      // and then throwing most away.
      pos.x = *reinterpret_cast<const float *>(point + xoff);
      pos.y = *reinterpret_cast<const float *>(point + yoff);
      pos.z = *reinterpret_cast<const float *>(point + zoff);

      pos = transform * pos;
      values.push_back(pos[axis]);
    }
  } else {
    const uint32_t offsets[3] = {xoff, yoff, zoff};
    const uint32_t off = offsets[axis];
    for (uint32_t i = 0; i < num_points; ++i, point += point_step) {
      values.push_back(*reinterpret_cast<const float *>( point + off ));
    }
  }
  float min_value_current = 9999.0f;
  float max_value_current = -9999.0f;
  if (auto_compute_bounds_property_->getBool()) {
    for (uint32_t i = 0; i < num_points; i++) {
      float val = values[i];
      min_value_current = std::min(min_value_current, val);
      max_value_current = std::max(max_value_current, val);
    }
    min_value_property_->setFloat(min_value_current);
    max_value_property_->setFloat(max_value_current);
  } else {
    min_value_current = min_value_property_->getFloat();
    max_value_current = max_value_property_->getFloat();
  }

  float range = max_value_current - min_value_current;
  if (range == 0) {
    range = 0.001f;
  }
  for (uint32_t i = 0; i < num_points; ++i) {
    float value = 1.0 - (values[i] - min_value_current) / range;
    getRainbowColor(value, points_out[i].color);
  }

  return true;
}

void AxisColorPCTransformer::createProperties(
  rviz_common::properties::Property * parent_property,
  uint32_t mask,
  QList<rviz_common::properties::Property *> & out_props)
{
  if (mask & Support_Color) {
    axis_property_ = new rviz_common::properties::EnumProperty(
      "Axis", "Z",
      "The axis to interpolate the color along.",
      parent_property, SIGNAL(needRetransform()), this);
    axis_property_->addOption("X", AXIS_X);
    axis_property_->addOption("Y", AXIS_Y);
    axis_property_->addOption("Z", AXIS_Z);

    auto_compute_bounds_property_ = new rviz_common::properties::BoolProperty(
      "Autocompute Value Bounds", true,
      "Whether to automatically compute the value min/max values.",
      parent_property, SLOT(updateAutoComputeBounds()), this);

    min_value_property_ = new rviz_common::properties::FloatProperty(
      "Min Value", -10,
      "Minimum value value, used to interpolate the color of a point.",
      auto_compute_bounds_property_);

    max_value_property_ = new rviz_common::properties::FloatProperty(
      "Max Value", 10,
      "Maximum value value, used to interpolate the color of a point.",
      auto_compute_bounds_property_);

    use_fixed_frame_property_ = new rviz_common::properties::BoolProperty(
      "Use Fixed Frame", true,
      "Whether to color the cloud based on its fixed frame position or its local frame position.",
      parent_property, SIGNAL(needRetransform()), this);

    out_props.push_back(axis_property_);
    out_props.push_back(auto_compute_bounds_property_);
    out_props.push_back(use_fixed_frame_property_);

    updateAutoComputeBounds();
  }
}

void AxisColorPCTransformer::updateAutoComputeBounds()
{
  bool auto_compute = auto_compute_bounds_property_->getBool();
  min_value_property_->setHidden(auto_compute);
  max_value_property_->setHidden(auto_compute);
  if (auto_compute) {
    disconnect(min_value_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
    disconnect(max_value_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
  } else {
    connect(min_value_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
    connect(max_value_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
    auto_compute_bounds_property_->expand();
  }
  Q_EMIT needRetransform();
}

void AxisColorPCTransformer::hideUnusedProperties()
{
  updateAutoComputeBounds();
}

}  // end namespace rviz_default_plugins
