/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
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
 *     * Neither the name of the copyright holders nor the names of its
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

#include <gmock/gmock.h>

#include <memory>

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include "rviz_default_plugins/displays/pointcloud/point_cloud_scalar_display.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

/*
 * w.l.o.g. use illuminance message
 * all scalar messages have identical structure
 */

sensor_msgs::msg::Illuminance::ConstSharedPtr createIlluminanceMessage(
  const double illuminance = 100., const double variance = 1.)
{
  auto message = std::make_shared<sensor_msgs::msg::Illuminance>();
  message->header = std_msgs::msg::Header();
  message->header.frame_id = "illuminance_frame";
  message->header.stamp = rclcpp::Clock().now();

  message->illuminance = illuminance;
  message->variance = variance;

  return message;
}

class PointCloudScalarDisplayImplementation
  : public rviz_default_plugins::displays::PointCloudScalarDisplay<sensor_msgs::msg::Illuminance>
{
public:
  PointCloudScalarDisplayImplementation() {}

protected:
  void processMessage(sensor_msgs::msg::Illuminance::ConstSharedPtr) override {}
  void setInitialValues() override {}
  void hideUnneededProperties() override {}
};

class PointCloudScalarDisplayFixture : public DisplayTestFixture
{
public:
  PointCloudScalarDisplayFixture()
  : display_(new PointCloudScalarDisplayImplementation)
  {}

  ~PointCloudScalarDisplayFixture()
  {
    display_.reset();
  }

  std::shared_ptr<PointCloudScalarDisplayImplementation> display_;
};

TEST_F(PointCloudScalarDisplayFixture, translates_scalar_message_into_point_cloud_message_correctly)
{
  auto illuminance_message = createIlluminanceMessage();
  auto point_cloud_message =
    display_->createPointCloud2Message(
    illuminance_message->header, illuminance_message->illuminance, "illuminance");

  ASSERT_THAT(point_cloud_message->point_step, Eq(20u));

  ASSERT_THAT(point_cloud_message->header.frame_id, illuminance_message->header.frame_id);

  ASSERT_THAT(point_cloud_message->fields[3].name, Eq("illuminance"));

  uint8_t offset_ptr = point_cloud_message->fields[3].offset;
  double * scalar_value_ptr =
    reinterpret_cast<double *>(point_cloud_message->data.data() + offset_ptr);
  ASSERT_THAT(*scalar_value_ptr, Eq(100.));

  for (int i = 0; i < 3; i++) {
    uint8_t offset_ptr = point_cloud_message->fields[i].offset;
    float * coordinate_value_ptr =
      reinterpret_cast<float *>(point_cloud_message->data.data() + offset_ptr);
    ASSERT_THAT(*coordinate_value_ptr, Eq(0.f));
  }
}
