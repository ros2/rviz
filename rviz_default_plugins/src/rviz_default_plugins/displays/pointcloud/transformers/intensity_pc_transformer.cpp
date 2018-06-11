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
#include <string>

#include "rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp"

namespace rviz_default_plugins
{

uint8_t IntensityPCTransformer::supports(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  updateChannels(cloud);
  return Support_Color;
}

uint8_t IntensityPCTransformer::score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  (void) cloud;
  return 255;
}

bool IntensityPCTransformer::transform(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t mask,
  const Ogre::Matrix4 & transform,
  V_PointCloudPoint & points_out)
{
  (void) transform;
  if (!(mask & Support_Color)) {
    return false;
  }

  int32_t index = findChannelIndex(cloud, channel_name_property_->getStdString());

  if (index == -1) {
    if (channel_name_property_->getStdString() == "intensity") {
      index = findChannelIndex(cloud, "intensities");
      if (index == -1) {
        return false;
      }
    } else {
      return false;
    }
  }

  const uint32_t offset = cloud->fields[index].offset;
  const uint8_t type = cloud->fields[index].datatype;
  const uint32_t point_step = cloud->point_step;
  const uint32_t num_points = cloud->width * cloud->height;

  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;
  if (auto_compute_intensity_bounds_property_->getBool()) {
    for (uint32_t i = 0; i < num_points; ++i) {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      min_intensity = std::min(val, min_intensity);
      max_intensity = std::max(val, max_intensity);
    }

    min_intensity = std::max(-999999.0f, min_intensity);
    max_intensity = std::min(999999.0f, max_intensity);
    min_intensity_property_->setFloat(min_intensity);
    max_intensity_property_->setFloat(max_intensity);
  } else {
    min_intensity = min_intensity_property_->getFloat();
    max_intensity = max_intensity_property_->getFloat();
  }
  float diff_intensity = max_intensity - min_intensity;
  if (diff_intensity == 0) {
    // If min and max are equal, set the diff to something huge so
    // when we divide by it, we effectively get zero.  That way the
    // point cloud coloring will be predictably uniform when min and
    // max are equal.
    diff_intensity = 1e20f;
  }
  Ogre::ColourValue max_color = max_color_property_->getOgreColor();
  Ogre::ColourValue min_color = min_color_property_->getOgreColor();

  if (use_rainbow_property_->getBool()) {
    for (uint32_t i = 0; i < num_points; ++i) {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float value = 1.0f - (val - min_intensity) / diff_intensity;
      if (invert_rainbow_property_->getBool()) {
        value = 1.0f - value;
      }
      getRainbowColor(value, points_out[i].color);
    }
  } else {
    for (uint32_t i = 0; i < num_points; ++i) {
      float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
      float normalized_intensity = (val - min_intensity) / diff_intensity;
      normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
      points_out[i].color.r = max_color.r * normalized_intensity + min_color.r *
        (1.0f - normalized_intensity);
      points_out[i].color.g = max_color.g * normalized_intensity + min_color.g *
        (1.0f - normalized_intensity);
      points_out[i].color.b = max_color.b * normalized_intensity + min_color.b *
        (1.0f - normalized_intensity);
    }
  }

  return true;
}

void IntensityPCTransformer::createProperties(
  rviz_common::properties::Property * parent_property,
  uint32_t mask,
  QList<rviz_common::properties::Property *> & out_props)
{
  if (mask & Support_Color) {
    channel_name_property_ = new rviz_common::properties::EditableEnumProperty(
      "Channel Name",
      "intensity",
      "Select the channel to use to compute the intensity",
      parent_property, SIGNAL(needRetransform()), this);

    use_rainbow_property_ = new rviz_common::properties::BoolProperty(
      "Use rainbow", true,
      "Whether to use a rainbow of colors or interpolate between two",
      parent_property, SLOT(updateUseRainbow()), this);
    invert_rainbow_property_ = new rviz_common::properties::BoolProperty(
      "Invert Rainbow", false,
      "Whether to invert rainbow colors",
      parent_property, SLOT(updateUseRainbow()), this);

    min_color_property_ = new rviz_common::properties::ColorProperty(
      "Min Color", Qt::black,
      "Color to assign the points with the minimum intensity.  "
      "Actual color is interpolated between this and Max Color.",
      parent_property, SIGNAL(needRetransform()), this);

    max_color_property_ = new rviz_common::properties::ColorProperty(
      "Max Color", Qt::white,
      "Color to assign the points with the maximum intensity.  "
      "Actual color is interpolated between this and Min Color.",
      parent_property, SIGNAL(needRetransform()), this);

    auto_compute_intensity_bounds_property_ = new rviz_common::properties::BoolProperty(
      "Autocompute Intensity Bounds", true,
      "Whether to automatically compute the intensity min/max values.",
      parent_property, SLOT(updateAutoComputeIntensityBounds()), this);

    min_intensity_property_ = new rviz_common::properties::FloatProperty(
      "Min Intensity", 0,
      "Minimum possible intensity value, used to interpolate from Min Color "
      "to Max Color for a point.",
      parent_property);

    max_intensity_property_ = new rviz_common::properties::FloatProperty(
      "Max Intensity", 4096,
      "Maximum possible intensity value, used to interpolate from Min Color "
      "to Max Color for a point.",
      parent_property);

    out_props.push_back(channel_name_property_);
    out_props.push_back(use_rainbow_property_);
    out_props.push_back(invert_rainbow_property_);
    out_props.push_back(min_color_property_);
    out_props.push_back(max_color_property_);
    out_props.push_back(auto_compute_intensity_bounds_property_);
    out_props.push_back(min_intensity_property_);
    out_props.push_back(max_intensity_property_);

    updateUseRainbow();
    updateAutoComputeIntensityBounds();
  }
}

void IntensityPCTransformer::updateChannels(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
{
  V_string channels;
  for (size_t i = 0; i < cloud->fields.size(); ++i) {
    channels.push_back(cloud->fields[i].name);
  }
  std::sort(channels.begin(), channels.end());

  if (channels != available_channels_) {
    channel_name_property_->clearOptions();
    for (V_string::const_iterator it = channels.begin(); it != channels.end(); ++it) {
      const std::string & channel = *it;
      if (channel.empty()) {
        continue;
      }
      channel_name_property_->addOptionStd(channel);
    }
    available_channels_ = channels;
  }
}

void IntensityPCTransformer::hideUnusedProperties()
{
  updateAutoComputeIntensityBounds();
  updateUseRainbow();
}

void IntensityPCTransformer::updateAutoComputeIntensityBounds()
{
  bool auto_compute = auto_compute_intensity_bounds_property_->getBool();
  min_intensity_property_->setHidden(auto_compute);
  max_intensity_property_->setHidden(auto_compute);
  if (auto_compute) {
    disconnect(min_intensity_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
    disconnect(max_intensity_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
  } else {
    connect(min_intensity_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
    connect(max_intensity_property_, SIGNAL(changed()), this, SIGNAL(needRetransform()));
  }
  Q_EMIT needRetransform();
}

void IntensityPCTransformer::updateUseRainbow()
{
  bool use_rainbow = use_rainbow_property_->getBool();
  invert_rainbow_property_->setHidden(!use_rainbow);
  min_color_property_->setHidden(use_rainbow);
  max_color_property_->setHidden(use_rainbow);
  Q_EMIT needRetransform();
}

}  // end namespace rviz_default_plugins
