// Copyright (c) 2026, Rahul Seervi.
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


#include <gmock/gmock.h>

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"

using namespace ::testing;  // NOLINT
using namespace rviz_default_plugins;  // NOLINT

namespace
{

template<typename FieldT>
float readSingleField(uint8_t datatype, FieldT value)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->height = 1;
  cloud->width = 1;
  cloud->point_step = sizeof(FieldT);
  cloud->row_step = cloud->point_step;

  sensor_msgs::msg::PointField field;
  field.name = "value";
  field.offset = 0;
  field.datatype = datatype;
  field.count = 1;
  cloud->fields.push_back(field);

  cloud->data.resize(sizeof(FieldT));
  std::memcpy(cloud->data.data(), &value, sizeof(FieldT));

  return valueFromCloud<float>(cloud, 0, datatype, cloud->point_step, 0);
}

// Store each value at byte `offset` of its own `point_step`-sized point, after
// other data, and read back the point at `index`.
template<typename FieldT>
float readFieldAtIndex(
  uint8_t datatype, const std::vector<FieldT> & values, uint32_t offset,
  uint32_t point_step, uint64_t index)
{
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->height = 1;
  cloud->width = static_cast<uint32_t>(values.size());
  cloud->point_step = point_step;
  cloud->row_step = point_step * cloud->width;

  sensor_msgs::msg::PointField field;
  field.name = "value";
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  cloud->fields.push_back(field);

  cloud->data.assign(cloud->row_step, 0xAB);
  for (size_t i = 0; i < values.size(); ++i) {
    std::memcpy(&cloud->data[i * point_step + offset], &values[i], sizeof(FieldT));
  }

  return valueFromCloud<float>(cloud, offset, datatype, point_step, index);
}

}  // namespace

TEST(PointCloudHelpers, valueFromCloud_keeps_the_sign_of_signed_integer_fields) {
  using sensor_msgs::msg::PointField;
  EXPECT_THAT(readSingleField<int8_t>(PointField::INT8, -1), FloatEq(-1.0f));
  EXPECT_THAT(readSingleField<int16_t>(PointField::INT16, -300), FloatEq(-300.0f));
  EXPECT_THAT(readSingleField<int32_t>(PointField::INT32, -1), FloatEq(-1.0f));
  EXPECT_THAT(readSingleField<int32_t>(PointField::INT32, 30), FloatEq(30.0f));
}

TEST(PointCloudHelpers, valueFromCloud_reads_unsigned_integer_fields_as_unsigned) {
  using sensor_msgs::msg::PointField;
  EXPECT_THAT(readSingleField<uint8_t>(PointField::UINT8, 255), FloatEq(255.0f));
  EXPECT_THAT(readSingleField<uint16_t>(PointField::UINT16, 65535), FloatEq(65535.0f));
  EXPECT_THAT(readSingleField<uint32_t>(PointField::UINT32, 4000000000u), FloatEq(4e9f));
}

TEST(PointCloudHelpers, valueFromCloud_reads_the_minimum_of_each_signed_type) {
  using sensor_msgs::msg::PointField;
  EXPECT_THAT(
    readSingleField<int8_t>(PointField::INT8, std::numeric_limits<int8_t>::min()),
    FloatEq(-128.0f));
  EXPECT_THAT(
    readSingleField<int16_t>(PointField::INT16, std::numeric_limits<int16_t>::min()),
    FloatEq(-32768.0f));
  EXPECT_THAT(
    readSingleField<int32_t>(PointField::INT32, std::numeric_limits<int32_t>::min()),
    FloatEq(-2147483648.0f));
}

TEST(PointCloudHelpers, valueFromCloud_reads_signed_fields_at_an_offset_and_index) {
  using sensor_msgs::msg::PointField;
  const std::vector<int32_t> values = {-1, 0, 1, 15, 30};
  for (uint64_t i = 0; i < values.size(); ++i) {
    EXPECT_THAT(
      readFieldAtIndex<int32_t>(PointField::INT32, values, 20, 32, i),
      FloatEq(static_cast<float>(values[i])));
  }
  EXPECT_THAT(
    readFieldAtIndex<int16_t>(PointField::INT16, {7, -2}, 6, 12, 1), FloatEq(-2.0f));
  EXPECT_THAT(
    readFieldAtIndex<int8_t>(PointField::INT8, {5, -3, 9}, 3, 4, 1), FloatEq(-3.0f));
}
