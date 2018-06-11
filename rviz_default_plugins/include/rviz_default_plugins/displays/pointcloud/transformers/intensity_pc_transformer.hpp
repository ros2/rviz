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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__INTENSITY_PC_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__INTENSITY_PC_TRANSFORMER_HPP_

#include "rviz_common/properties/editable_enum_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC IntensityPCTransformer : public PointCloudTransformer
{
  Q_OBJECT

public:
  uint8_t supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  bool transform(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    uint32_t mask,
    const Ogre::Matrix4 & transform,
    V_PointCloudPoint & points_out) override;

  uint8_t score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  void createProperties(
    rviz_common::properties::Property * parent_property,
    uint32_t mask,
    QList<rviz_common::properties::Property *> & out_props) override;

  void updateChannels(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud);

  void hideUnusedProperties() override;

private Q_SLOTS:
  void updateUseRainbow();

  void updateAutoComputeIntensityBounds();

private:
  V_string available_channels_;

  rviz_common::properties::ColorProperty * min_color_property_;
  rviz_common::properties::ColorProperty * max_color_property_;
  rviz_common::properties::BoolProperty * auto_compute_intensity_bounds_property_;
  rviz_common::properties::BoolProperty * use_rainbow_property_;
  rviz_common::properties::BoolProperty * invert_rainbow_property_;
  rviz_common::properties::FloatProperty * min_intensity_property_;
  rviz_common::properties::FloatProperty * max_intensity_property_;
  rviz_common::properties::EditableEnumProperty * channel_name_property_;
};

}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__INTENSITY_PC_TRANSFORMER_HPP_
