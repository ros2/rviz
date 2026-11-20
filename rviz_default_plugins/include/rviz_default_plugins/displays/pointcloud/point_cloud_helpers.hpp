// Copyright (c) 2010, Willow Garage, Inc. Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_HELPERS_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_HELPERS_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
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

/// Size in bytes of a sensor_msgs::msg::PointField datatype, or 0 if it is unknown.
inline size_t pointFieldTypeSize(uint8_t type)
{
  switch (type) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

inline bool hostIsBigEndian()
{
  const uint16_t one = 1;
  uint8_t first_byte;
  std::memcpy(&first_byte, &one, 1);
  return first_byte == 0;
}

/// Read a FieldT stored at data, which need not be aligned, reversing its bytes if
/// swap_bytes is set.
template<typename FieldT>
inline FieldT readFieldValue(const uint8_t * data, bool swap_bytes)
{
  uint8_t bytes[sizeof(FieldT)];
  std::memcpy(bytes, data, sizeof(FieldT));
  if (swap_bytes) {
    std::reverse(bytes, bytes + sizeof(FieldT));
  }
  FieldT value;
  std::memcpy(&value, bytes, sizeof(FieldT));
  return value;
}

/// Read field `type` at `offset` of point `index`, converted to T. Returns 0 if the type
/// is unknown or the value would lie outside cloud->data.
template<typename T>
inline T valueFromCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  uint32_t offset,
  uint8_t type,
  uint32_t point_step,
  uint64_t index)
{
  const size_t size = pointFieldTypeSize(type);
  const uint64_t start = static_cast<uint64_t>(point_step) * index + offset;
  if (size == 0 || start > cloud->data.size() || cloud->data.size() - start < size) {
    return 0;
  }
  const uint8_t * data = &cloud->data[start];
  const bool swap_bytes = static_cast<bool>(cloud->is_bigendian) != hostIsBigEndian();

  switch (type) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<T>(readFieldValue<int8_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<T>(readFieldValue<uint8_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<T>(readFieldValue<int16_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<T>(readFieldValue<uint16_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<T>(readFieldValue<int32_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<T>(readFieldValue<uint32_t>(data, swap_bytes));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<T>(readFieldValue<float>(data, swap_bytes));
    case sensor_msgs::msg::PointField::FLOAT64:
      return static_cast<T>(readFieldValue<double>(data, swap_bytes));
    default:
      return 0;
  }
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
