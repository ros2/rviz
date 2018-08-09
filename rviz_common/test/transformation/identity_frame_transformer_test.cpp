/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "src/rviz_common/transformation/identity_frame_transformer.hpp"
#include "transformation_test_helpers.hpp"

using namespace testing;  // NOLINT

class IdentityTransformerFixture : public testing::Test
{
public:
  IdentityTransformerFixture()
  {
    transformer_ = std::make_shared<rviz_common::transformation::IdentityFrameTransformer>();
  }

  std::shared_ptr<rviz_common::transformation::IdentityFrameTransformer> transformer_;
};


TEST_F(IdentityTransformerFixture, getAllFrameNames_returns_a_vector_containing_an_empty_string) {
  auto frame_names = transformer_->getAllFrameNames();

  ASSERT_THAT(frame_names, SizeIs(1));
  EXPECT_THAT(frame_names[0], Eq(""));
}

TEST_F(IdentityTransformerFixture, transform_returns_the_input_pose_if_orientation_is_valid) {
  auto pose_stamped = createRvizCommonPoseStamped(
    1, 3, "test", createRvizCommonPoint(0, 3, 6), createRvizCommonQuaternion(0, 1, 0, 0));
  auto transformed_pose = transformer_->transform(pose_stamped, "any_frame");

  EXPECT_THAT(transformed_pose.time_stamp.seconds, Eq(1));
  EXPECT_THAT(transformed_pose.time_stamp.nanoseconds, Eq(static_cast<uint32_t>(3)));
  EXPECT_THAT(transformed_pose.frame_id, Eq("test"));
  EXPECT_THAT(transformed_pose.position, PointEq(createRvizCommonPoint(0, 3, 6)));
  EXPECT_THAT(transformed_pose.orientation, QuaternionEq(createRvizCommonQuaternion(0, 1, 0, 0)));
}

TEST_F(
  IdentityTransformerFixture,
  transform_returns_the_input_pose_with_trivial_orientation_if_original_orientation_is_not_valid) {
  auto pose_stamped = createRvizCommonPoseStamped(
    1, 3, "test", createRvizCommonPoint(1, 3, 6), createRvizCommonQuaternion(0, 0, 0, 0));
  auto transformed_pose = transformer_->transform(pose_stamped, "any_frame");

  EXPECT_THAT(transformed_pose.time_stamp.seconds, Eq(1));
  EXPECT_THAT(transformed_pose.time_stamp.nanoseconds, Eq(static_cast<uint32_t>(3)));
  EXPECT_THAT(transformed_pose.frame_id, Eq("test"));
  EXPECT_THAT(transformed_pose.position, PointEq(createRvizCommonPoint(1, 3, 6)));
  EXPECT_THAT(transformed_pose.orientation, QuaternionEq(createRvizCommonQuaternion(1, 0, 0, 0)));
}

TEST_F(IdentityTransformerFixture, transformationIsAvailable_returns_true) {
  EXPECT_TRUE(transformer_->transformIsAvailable("any_target", "any_source"));
}

TEST_F(IdentityTransformerFixture, transformHasProblesm_returns_false) {
  std::string error = "";

  EXPECT_FALSE(transformer_->transformHasProblems(
      "any_target", "any_source", rclcpp::Clock().now(), error));
  EXPECT_THAT(error, Eq(""));
}

TEST_F(IdentityTransformerFixture, frameHasProblesm_returns_false) {
  std::string error = "";

  EXPECT_FALSE(transformer_->frameHasProblems("any_frame", error));
  EXPECT_THAT(error, Eq(""));
}
