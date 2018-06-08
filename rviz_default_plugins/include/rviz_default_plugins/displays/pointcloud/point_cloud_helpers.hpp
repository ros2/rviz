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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_HELPERS_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_HELPERS_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rviz_common/properties/property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"

namespace rviz_common
{
namespace properties
{

class BoolProperty;
class ColorProperty;
class EditableEnumProperty;
class EnumProperty;
class FloatProperty;

}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

typedef std::vector<std::string> V_string;

inline int32_t findChannelIndex(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const std::string & channel)
{
  for (size_t i = 0; i < cloud->fields.size(); ++i) {
    if (cloud->fields[i].name == channel) {
      return static_cast<uint32_t>(i);
    }
  }

  return -1;
}

template<typename T>
inline T valueFromCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t offset,
  uint8_t type,
  uint32_t point_step,
  uint64_t index)
{
  const uint8_t * data = &cloud->data[(point_step * index) + offset];
  T ret = 0;

  switch (type) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      {
        uint8_t val = *reinterpret_cast<const uint8_t *>(data);
        ret = static_cast<T>(val);
        break;
      }

    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      {
        uint16_t val = *reinterpret_cast<const uint16_t *>(data);
        ret = static_cast<T>(val);
        break;
      }

    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
      {
        uint32_t val = *reinterpret_cast<const uint32_t *>(data);
        ret = static_cast<T>(val);
        break;
      }

    case sensor_msgs::msg::PointField::FLOAT32:
      {
        float val = *reinterpret_cast<const float *>(data);
        ret = static_cast<T>(val);
        break;
      }

    case sensor_msgs::msg::PointField::FLOAT64:
      {
        double val = *reinterpret_cast<const double *>(data);
        ret = static_cast<T>(val);
        break;
      }
    default:
      break;
  }

  return ret;
}

inline void getRainbowColor(float value, Ogre::ColourValue & color)
{
  // this is HSV color palette with hue values going only from 0.0 to 0.833333.

  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);

  float h = value * 5.0f + 1.0f;
  int i = floor(h);
  float f = h - i;
  if (!(i & 1) ) {
    f = 1 - f;             // if i is even
  }
  float n = 1 - f;

  if (i <= 1) {
    color[0] = n, color[1] = 0, color[2] = 1;
  } else if (i == 2) {
    color[0] = 0, color[1] = n, color[2] = 1;
  } else if (i == 3) {
    color[0] = 0, color[1] = 1, color[2] = n;
  } else if (i == 4) {
    color[0] = n, color[1] = 1, color[2] = 0;
  } else if (i >= 5) {
    color[0] = 1, color[1] = n, color[2] = 0;
  }
}

}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_HELPERS_HPP_
