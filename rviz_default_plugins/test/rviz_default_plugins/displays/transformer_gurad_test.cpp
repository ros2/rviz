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

#include <QApplication>  // NOLINT

#include "tf2_ros/buffer.h"

#include <include/rviz_default_plugins/displays/pose/pose_display.hpp>
#include <include/rviz_default_plugins/displays/marker/marker_display.hpp>
#include <include/rviz_default_plugins/displays/path/path_display.hpp>

#include "rviz_common/display.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

#include "rviz_default_plugins/displays/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "display_test_fixture.hpp"
#include "../mock_display_context.hpp"

using namespace ::testing;  // NOLINT

class TransformerGuardTestFixture : public DisplayTestFixture
{
public:
  TransformerGuardTestFixture()
  {
    display_ = std::make_shared<rviz_common::Display>();
    display_->initialize(context_.get());
    EXPECT_CALL(*context_, getFrameManager()).WillRepeatedly(Return(frame_manager_.get()));

    transformer_guard_ = std::make_unique<
      rviz_default_plugins::displays::TransformerGuard<
        rviz_default_plugins::transformation::TFWrapper>>(display_.get(), "any_display", "TF");
    transformer_guard_->initialize(context_.get());

    display_->setEnabled(true);
  }

  std::shared_ptr<rviz_common::Display> display_;
  std::unique_ptr<rviz_default_plugins::displays::TransformerGuard<
    rviz_default_plugins::transformation::TFWrapper>> transformer_guard_;
};

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_makes_display_disabled_with_wrong_transformer) {
  auto base_transformer = std::make_shared<rviz_common::transformation::InternalFrameTransformer>();
  transformer_guard_->updateDisplayAccordingToTransformerType(
    std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>(base_transformer));

  EXPECT_FALSE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_makes_display_enabled_with_correct_transformer) {
  auto base_transformer = std::make_shared<rviz_common::transformation::InternalFrameTransformer>();
  auto tf_wrapper = std::make_shared<rviz_default_plugins::transformation::TFWrapper>(
    std::make_shared<tf2_ros::Buffer>(), false);
  transformer_guard_->updateDisplayAccordingToTransformerType(
    std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>(base_transformer));
  transformer_guard_->updateDisplayAccordingToTransformerType(
    std::weak_ptr<rviz_default_plugins::transformation::TFWrapper>(tf_wrapper));

  EXPECT_TRUE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_keeps_display_disabled_if_it_was_disabled_by_user) {
  auto base_transformer = std::make_shared<rviz_common::transformation::InternalFrameTransformer>();
  auto tf_wrapper = std::make_shared<rviz_default_plugins::transformation::TFWrapper>(
    std::make_shared<tf2_ros::Buffer>(), false);
  display_->setEnabled(false);
  transformer_guard_->updateDisplayAccordingToTransformerType(
    std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>(base_transformer));
  transformer_guard_->updateDisplayAccordingToTransformerType(
    std::weak_ptr<rviz_default_plugins::transformation::TFWrapper>(tf_wrapper));

  EXPECT_FALSE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  usingAllowedTransformer_returns_true_if_current_transformer_is_correct_one) {
  auto tf_wrapper = std::make_shared<rviz_default_plugins::transformation::TFWrapper>(
    std::make_shared<tf2_ros::Buffer>(), false);
  EXPECT_CALL(*frame_manager_, getInternalPtr())
    .WillOnce(Return(std::weak_ptr<rviz_default_plugins::transformation::TFWrapper>(tf_wrapper)));

  EXPECT_TRUE(transformer_guard_->usingAllowedTransformer());
}

TEST_F(
  TransformerGuardTestFixture,
  usingAllowedTransformer_returns_false_if_current_transformer_is_a_wrong_one) {
  auto base_transformer = std::make_shared<rviz_common::transformation::InternalFrameTransformer>();
  EXPECT_CALL(*frame_manager_, getInternalPtr()).WillOnce(Return(
      std::weak_ptr<rviz_common::transformation::InternalFrameTransformer>(base_transformer)));

  EXPECT_FALSE(transformer_guard_->usingAllowedTransformer());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}