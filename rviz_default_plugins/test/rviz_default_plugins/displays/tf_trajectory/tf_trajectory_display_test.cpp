// Copyright (c) 2015, JSK Lab
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

#include <memory>
#include <string>

#include <OgreBillboardChain.h>
#include <OgreSceneNode.h>

#include "rcl/time.h"
#include "rclcpp/clock.hpp"

#include "rviz_default_plugins/displays/tf_trajectory/tf_trajectory_display.hpp"

#include "../display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

namespace
{

/// A ROS-time clock whose value only changes when the test says so.
std::shared_ptr<rclcpp::Clock> createFrozenClock(int64_t nanoseconds)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_enable_ros_time_override(clock->get_clock_handle());
  rcl_set_ros_time_override(clock->get_clock_handle(), nanoseconds);
  return clock;
}

void setClock(const std::shared_ptr<rclcpp::Clock> & clock, int64_t nanoseconds)
{
  rcl_set_ros_time_override(clock->get_clock_handle(), nanoseconds);
}

size_t countTrajectoryPoints(Ogre::SceneNode * scene_node)
{
  auto * chain = rviz_default_plugins::findOneBillboardChain(scene_node);
  return chain ? chain->getNumChainElements(0) : 0u;
}

constexpr int64_t kOneSecond = 1000000000;

}  // namespace

class TFTrajectoryTestFixture : public DisplayTestFixture
{
public:
  TFTrajectoryTestFixture()
  {
    clock_ = createFrozenClock(100 * kOneSecond);
    EXPECT_CALL(*context_, getClock()).WillRepeatedly(Return(clock_));

    display_ = std::make_shared<rviz_default_plugins::displays::TFTrajectoryDisplay>(
      context_.get());
    display_->setName("TFTrajectory");
    display_->findProperty("Frame")->setValue("moving_frame");
  }

  /// The shared fixture only mocks the timestamped overloads; this display asks
  /// for the latest available transform, which is the three-argument one.
  void mockLatestTransform(bool available)
  {
    EXPECT_CALL(*frame_manager_, getTransform(_, _, _))  // NOLINT
    .WillRepeatedly(
      DoAll(
        SetArgReferee<1>(Ogre::Vector3(1, 2, 3)),
        SetArgReferee<2>(Ogre::Quaternion::IDENTITY),
        Return(available)));
    EXPECT_CALL(*frame_manager_, transformHasProblems(_, _))  // NOLINT
    .WillRepeatedly(Return(false));
  }

  void updateDisplay()
  {
    display_->update(std::chrono::nanoseconds(0), std::chrono::nanoseconds(0));
  }

  size_t drawnPoints()
  {
    return countTrajectoryPoints(scene_manager_->getRootSceneNode());
  }

  std::shared_ptr<rviz_default_plugins::displays::TFTrajectoryDisplay> display_;
};

TEST_F(TFTrajectoryTestFixture, update_draws_one_point_per_step_while_time_advances) {
  mockLatestTransform(true);

  for (int i = 1; i <= 3; ++i) {
    setClock(clock_, (100 + i) * kOneSecond);
    updateDisplay();
  }

  EXPECT_THAT(drawnPoints(), Eq(3u));
}

TEST_F(TFTrajectoryTestFixture, update_does_not_grow_the_history_while_the_clock_is_stopped) {
  mockLatestTransform(true);

  // Simulates use_sim_time with /clock paused: `now` never moves, so nothing
  // can ever expire. Appending a sample per render tick would grow without
  // bound, so a stopped clock must not add anything after the first sample.
  for (int i = 0; i < 20; ++i) {
    updateDisplay();
  }

  EXPECT_THAT(drawnPoints(), Eq(1u));
}

TEST_F(TFTrajectoryTestFixture, update_expires_the_history_when_the_transform_is_lost) {
  mockLatestTransform(true);
  setClock(clock_, 101 * kOneSecond);
  updateDisplay();
  setClock(clock_, 102 * kOneSecond);
  updateDisplay();
  ASSERT_THAT(drawnPoints(), Eq(2u));

  // The frame stops being published: the lookup fails from here on. The
  // history must still expire after Duration rather than staying on screen.
  mockLatestTransform(false);

  display_->findProperty("Duration")->setValue(5.0f);
  setClock(clock_, 200 * kOneSecond);
  updateDisplay();

  EXPECT_THAT(drawnPoints(), Eq(0u));
}

TEST_F(TFTrajectoryTestFixture, update_clears_the_history_when_time_jumps_backwards) {
  mockLatestTransform(true);
  setClock(clock_, 101 * kOneSecond);
  updateDisplay();
  setClock(clock_, 102 * kOneSecond);
  updateDisplay();
  ASSERT_THAT(drawnPoints(), Eq(2u));

  // A rosbag looping restarts simulated time; points stamped in the future
  // would otherwise outlive Duration until time caught up again.
  setClock(clock_, 10 * kOneSecond);
  updateDisplay();

  EXPECT_THAT(drawnPoints(), Eq(1u));
}

TEST_F(TFTrajectoryTestFixture, duration_property_is_clamped_to_a_representable_range) {
  auto * duration = display_->findProperty("Duration");

  // rclcpp::Duration::from_seconds() multiplies into an int64 of nanoseconds,
  // so an unbounded value overflows it.
  duration->setValue(1.0e9f);
  EXPECT_THAT(duration->getValue().toFloat(), Le(3600.0f));

  duration->setValue(-5.0f);
  EXPECT_THAT(duration->getValue().toFloat(), Ge(0.0f));
}
