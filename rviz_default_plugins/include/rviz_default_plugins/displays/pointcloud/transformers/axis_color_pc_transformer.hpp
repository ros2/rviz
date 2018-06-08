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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__AXIS_COLOR_PC_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__AXIS_COLOR_PC_TRANSFORMER_HPP_

#include <vector>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_default_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC AxisColorPCTransformer : public PointCloudTransformer
{
  Q_OBJECT

public:
  uint8_t supports(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  bool transform(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    uint32_t mask,
    const Ogre::Matrix4 & transform,
    rviz_default_plugins::V_PointCloudPoint & points_out) override;

  void createProperties(
    rviz_common::properties::Property * parent_property,
    uint32_t mask,
    QList<rviz_common::properties::Property *> & out_props) override;

  uint8_t score(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud) override;

  void hideUnusedProperties() override;

  enum Axis
  {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
  };

private Q_SLOTS:
  void updateAutoComputeBounds();

private:
  rviz_common::properties::BoolProperty * auto_compute_bounds_property_;
  rviz_common::properties::FloatProperty * min_value_property_;
  rviz_common::properties::FloatProperty * max_value_property_;
  rviz_common::properties::EnumProperty * axis_property_;
  rviz_common::properties::BoolProperty * use_fixed_frame_property_;
};

}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__TRANSFORMERS__AXIS_COLOR_PC_TRANSFORMER_HPP_
