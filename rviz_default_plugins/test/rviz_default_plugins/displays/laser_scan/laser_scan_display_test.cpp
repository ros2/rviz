/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
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

#include "tf2_ros/buffer.h"

#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/properties/queue_size_property.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"
#include "rviz_default_plugins/displays/laser_scan/laser_scan_display.hpp"

#include "../display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class LaserScanTestFixture : public DisplayTestFixture
{
public:
  LaserScanTestFixture()
  {
    display_ = std::make_unique<rviz_default_plugins::displays::LaserScanDisplay>(context_.get());
    EXPECT_CALL(*context_, getFrameManager()).WillRepeatedly(Return(frame_manager_.get()));
  }

  bool allPropertiesAreHidden()
  {
    int properties_number = display_->numChildren();
    bool all_hidden = true;
    for (int i = 0; i < properties_number; ++i) {
      auto property = display_->childAt(i);
      if (property->objectName().contains("Status")) {
        continue;
      }
      all_hidden = all_hidden && property->getHidden();
    }

    return all_hidden;
  }

  bool defaultPropertiesAreVisible()
  {
    bool visible = true;
    visible = visible && !display_->findProperty("Topic")->getHidden();
    visible = visible && !display_->findProperty("Unreliable")->getHidden();
    visible = visible && !display_->findProperty("Selectable")->getHidden();
    visible = visible && !display_->findProperty("Style")->getHidden();
    visible = visible && !display_->findProperty("Size (m)")->getHidden();
    visible = visible && !display_->findProperty("Alpha")->getHidden();
    visible = visible && !display_->findProperty("Decay Time")->getHidden();
    visible = visible && !display_->findProperty("Queue Size")->getHidden();
    visible = visible && !display_->findProperty("Position Transformer")->getHidden();
    visible = visible && !display_->findProperty("Color Transformer")->getHidden();

    return visible;
  }

  std::unique_ptr<rviz_default_plugins::displays::LaserScanDisplay> display_;
};

TEST_F(
  LaserScanTestFixture,
  updateDisplayAccordingToTransformerType_hides_properties_if_transformer_is_not_tf) {
  EXPECT_CALL(*frame_manager_, getInternalPtr()).WillOnce(
    Return(std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>()));

  display_->updateDisplayAccordingToTransformerType();
  EXPECT_TRUE(allPropertiesAreHidden());
}

TEST_F(
  LaserScanTestFixture,
  updateDisplayAccordingToTransformerType_shows_default_properties_if_transformer_is_tf) {
  auto tf_wrapper = std::make_shared<rviz_default_plugins::transformation::TFWrapper>(
    std::make_shared<tf2_ros::Buffer>(), false);
  EXPECT_CALL(*frame_manager_, getInternalPtr())
  .WillOnce(Return(std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>()))
  .WillOnce(Return(std::weak_ptr<rviz_default_plugins::transformation::TFWrapper>(tf_wrapper)));
  auto pose_transformer_property = display_->findProperty("Position Transformer");
  pose_transformer_property->setValue("any_position_transformer");
  auto color_transformer_property = display_->findProperty("Color Transformer");
  color_transformer_property->setValue("any_color_transformer");
  auto topic_property = display_->findProperty("Topic");
  topic_property->setValue("laser_topic");

  display_->updateDisplayAccordingToTransformerType();
  display_->updateDisplayAccordingToTransformerType();
  EXPECT_TRUE(defaultPropertiesAreVisible());
  EXPECT_THAT(
    pose_transformer_property->getValue().toString().toStdString(), Eq("any_position_transformer"));
  EXPECT_THAT(
    color_transformer_property->getValue().toString().toStdString(), Eq("any_color_transformer"));
  EXPECT_THAT(
    topic_property->getValue().toString().toStdString(), Eq("laser_topic"));
}
