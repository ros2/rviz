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

#include "rviz_common/display.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "../displays/display_test_fixture.hpp"
#include "../mock_display_context.hpp"
#include "./mock_frame_transformer.hpp"

using namespace ::testing;  // NOLINT

class TransformerGuardTestFixture : public DisplayTestFixture
{
public:
  TransformerGuardTestFixture()
  {
    display_ = std::make_shared<rviz_common::Display>();
    display_->initialize(context_.get());
    base_transformer_ = std::make_shared<MockFrameTransformer>();
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto tf_wrapper = std::make_shared<rviz_default_plugins::transformation::TFWrapper>();
    tf_wrapper->initializeBuffer(clock, node, false);
    tf_transformer_ = std::make_shared<rviz_default_plugins::transformation::TFFrameTransformer>(
      tf_wrapper);

    EXPECT_CALL(*context_, getFrameManager()).WillRepeatedly(Return(frame_manager_.get()));
    EXPECT_CALL(*frame_manager_, getTransformer()).WillOnce(Return(base_transformer_));

    transformer_guard_ = std::make_unique<
      rviz_default_plugins::transformation::TransformerGuard<
        rviz_default_plugins::transformation::TFFrameTransformer>>(display_.get(), "TF");
    transformer_guard_->initialize(context_.get());

    display_->setEnabled(true);
  }

  void setWrongTransformer()
  {
    transformer_guard_->updateDisplayAccordingToTransformerType(base_transformer_);
  }

  void setCorrectTransformer()
  {
    transformer_guard_->updateDisplayAccordingToTransformerType(tf_transformer_);
  }

  void setCorectTransformerAndEnableDisplay()
  {
    setCorrectTransformer();
    display_->setEnabled(true);
  }

  std::shared_ptr<rviz_common::Display> display_;
  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;
  std::shared_ptr<MockFrameTransformer> base_transformer_;
  std::shared_ptr<rviz_default_plugins::transformation::TFFrameTransformer> tf_transformer_;
};

TEST_F(TransformerGuardTestFixture, initialize_makes_display_disabled_if_transformer_is_wrong) {
  EXPECT_FALSE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_disables_display_on_wrong_transformer) {
  setCorectTransformerAndEnableDisplay();

  setWrongTransformer();

  EXPECT_FALSE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_enables_display_on_correct_transformer) {
  setCorectTransformerAndEnableDisplay();

  setWrongTransformer();
  setCorrectTransformer();

  EXPECT_TRUE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  updateDisplayAccordingToTransformerType_keeps_display_disabled_if_it_was_disabled_by_user) {
  setCorectTransformerAndEnableDisplay();
  display_->setEnabled(false);

  setWrongTransformer();
  setCorrectTransformer();

  EXPECT_FALSE(display_->isEnabled());
}

TEST_F(
  TransformerGuardTestFixture,
  usingAllowedTransformer_returns_true_if_current_transformer_is_correct_one) {
  EXPECT_CALL(*frame_manager_, getTransformer()).WillOnce(Return(tf_transformer_));

  EXPECT_TRUE(transformer_guard_->checkTransformer());
}

TEST_F(
  TransformerGuardTestFixture,
  usingAllowedTransformer_returns_false_if_current_transformer_is_a_wrong_one) {
  EXPECT_CALL(*frame_manager_, getTransformer()).WillOnce(Return(base_transformer_));

  EXPECT_FALSE(transformer_guard_->checkTransformer());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
