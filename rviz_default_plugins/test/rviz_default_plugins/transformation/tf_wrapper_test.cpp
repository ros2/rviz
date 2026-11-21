// Copyright (c) 2026, jgdo
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

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

using rviz_default_plugins::transformation::TFFrameTransformer;
using rviz_default_plugins::transformation::TFWrapper;

namespace
{

geometry_msgs::msg::TransformStamped makeTransform(
  const std::string & parent, const std::string & child, int32_t sec, double x = 1.0)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.header.stamp.sec = sec;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

// Frame tree used in all tests (like a typical mobile robot):
//   map --(dynamic)--> odom --(dynamic)--> base --(static, via /tf_static)--> laser
//
// The test parameter selects whether the TransformListener spins in its own thread (as in rviz)
// or on the rviz node.
class TFWrapperClearFixture : public testing::TestWithParam<bool>
{
public:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rviz_ros_node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
      "tf_wrapper_test_node");
    publisher_node_ = std::make_shared<rclcpp::Node>("tf_wrapper_test_static_publisher");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(rviz_ros_node_->get_raw_node());
    executor_->add_node(publisher_node_);

    clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf_wrapper_ = std::make_shared<TFWrapper>();
    tf_wrapper_->initialize(clock_, rviz_ros_node_, GetParam());

    static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(publisher_node_);
    static_broadcaster_->sendTransform(makeTransform("base", "laser", 0));

    ASSERT_TRUE(waitForStaticLaserTransform(1.0))
      << "static transform was not received on /tf_static";

    setDynamicTransforms(10);
  }

  void TearDown() override
  {
    static_broadcaster_.reset();
    tf_wrapper_.reset();
    executor_.reset();
    publisher_node_.reset();
    rviz_ros_node_.reset();
  }

  bool spinUntil(
    const std::function<bool()> & condition,
    std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(10));
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    spinUntil([]() {return false;}, duration);
  }

  bool waitForStaticLaserTransform(double expected_x)
  {
    return spinUntil(
      [this, expected_x]() {
        return canTransform("base", "laser", tf2::TimePointZero) &&
               tf_wrapper_->lookupTransform(
          "base", "laser", tf2::TimePointZero).transform.translation.x == expected_x;
      });
  }

  void setDynamicTransforms(int32_t sec)
  {
    // getBuffer() has to be called every time, since clear() replaces the buffer
    auto buffer = tf_wrapper_->getBuffer();
    buffer->setTransform(makeTransform("map", "odom", sec), "test", false);
    buffer->setTransform(makeTransform("odom", "base", sec), "test", false);
  }

  bool canTransform(
    const std::string & target, const std::string & source, tf2::TimePoint time)
  {
    std::string error;
    return tf_wrapper_->canTransform(target, source, time, error);
  }

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<TFWrapper> tf_wrapper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

TEST_P(TFWrapperClearFixture, clear_removes_dynamic_transforms) {
  ASSERT_TRUE(canTransform("map", "base", tf2::timeFromSec(10)));

  tf_wrapper_->clear();

  EXPECT_FALSE(canTransform("map", "base", tf2::timeFromSec(10)));
  EXPECT_FALSE(tf_wrapper_->frameExists("odom"));
}

TEST_P(TFWrapperClearFixture, clear_resubscribes_so_static_transforms_are_received_again) {
  tf_wrapper_->clear();

  // /tf_static is transient local, so the still running publisher re-sends its transform to the
  // new subscription
  EXPECT_TRUE(waitForStaticLaserTransform(1.0));
}

TEST_P(TFWrapperClearFixture, clear_drops_static_transforms_of_publishers_that_are_gone) {
  {
    auto camera_node = std::make_shared<rclcpp::Node>("tf_wrapper_test_camera_publisher");
    auto camera_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(camera_node);
    camera_broadcaster->sendTransform(makeTransform("base", "camera", 0));
    ASSERT_TRUE(spinUntil([this]() {return tf_wrapper_->frameExists("camera");}));
  }  // camera publisher shuts down

  tf_wrapper_->clear();

  ASSERT_TRUE(waitForStaticLaserTransform(1.0));
  spinFor(std::chrono::milliseconds(500));
  EXPECT_FALSE(tf_wrapper_->frameExists("camera"));
  EXPECT_FALSE(canTransform("base", "camera", tf2::TimePointZero));
}

TEST_P(TFWrapperClearFixture, clear_receives_latest_version_of_updated_static_transform) {
  static_broadcaster_->sendTransform(makeTransform("base", "laser", 0, 2.0));
  ASSERT_TRUE(waitForStaticLaserTransform(2.0));

  tf_wrapper_->clear();

  EXPECT_TRUE(waitForStaticLaserTransform(2.0));
}

TEST_P(TFWrapperClearFixture, full_chain_is_transformable_again_after_clear) {
  tf_wrapper_->clear();
  setDynamicTransforms(20);

  EXPECT_TRUE(spinUntil([this]() {return canTransform("map", "laser", tf2::timeFromSec(20));}));
  EXPECT_NO_THROW(tf_wrapper_->lookupTransform("map", "laser", tf2::timeFromSec(20)));
}

// Regression test: after clear(), waitForTransform() used to throw tf2::LookupException
// ("Static cache is empty") synchronously, because tf2 reported the transform as immediately
// available although the static cache had been emptied.
TEST_P(TFWrapperClearFixture, waitForTransform_does_not_throw_after_clear) {
  for (bool with_new_dynamic_transforms : {false, true}) {
    for (int32_t stamp : {0, 20}) {
      SCOPED_TRACE(
        "new dynamic transforms: " + std::to_string(with_new_dynamic_transforms) +
        ", stamp: " + std::to_string(stamp));
      setDynamicTransforms(10);
      tf_wrapper_->clear();
      if (with_new_dynamic_transforms) {
        setDynamicTransforms(20);
      }

      EXPECT_NO_THROW(
      {
        auto future = tf_wrapper_->waitForTransform(
          "map", "laser", tf2::timeFromSec(stamp), tf2::durationFromSec(0.1),
          [](const tf2_ros::TransformStampedFuture &) {});
        tf_wrapper_->cancel(future);
      });
    }
  }
}

TEST_P(TFWrapperClearFixture, waitForTransform_succeeds_once_static_transforms_are_received_again) {
  tf_wrapper_->clear();
  setDynamicTransforms(20);

  std::atomic<bool> succeeded{false};
  std::atomic<bool> failed{false};
  tf_wrapper_->waitForTransform(
    "map", "laser", tf2::timeFromSec(20), tf2::durationFromSec(5.0),
    [&succeeded, &failed](const tf2_ros::TransformStampedFuture & future) {
      try {
        future.get();
        succeeded = true;
      } catch (const tf2::TransformException &) {
        failed = true;
      }
    });

  EXPECT_TRUE(spinUntil([&]() {return succeeded || failed;}, std::chrono::seconds(6)));
  EXPECT_TRUE(succeeded);
}

// Reproduces the original crash: rviz terminated with an uncaught tf2::LookupException when
// pressing "Reset" while e.g. a LaserScan display was subscribed, because the exception escaped
// from tf2_ros::MessageFilter::add(). The filter is set up like in rviz_common's
// MessageFilterDisplay, i.e. it accesses tf through the FrameTransformer.
TEST_P(TFWrapperClearFixture, message_filter_accepts_messages_after_clear) {
  TFFrameTransformer transformer(tf_wrapper_);
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan, rviz_common::transformation::FrameTransformer>
  filter(transformer, "map", 10, *rviz_ros_node_->get_raw_node(), std::chrono::seconds(5));
  int received = 0;
  filter.registerCallback(
    [&received](const sensor_msgs::msg::LaserScan::ConstSharedPtr &) {++received;});

  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.frame_id = "laser";
  scan->header.stamp.sec = 10;
  ASSERT_NO_THROW(filter.add(scan));
  EXPECT_EQ(1, received);

  filter.clear();  // like rviz: displays are reset before the transformer is cleared
  tf_wrapper_->clear();
  setDynamicTransforms(20);

  auto scan_after_reset = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan_after_reset->header.frame_id = "laser";
  scan_after_reset->header.stamp.sec = 20;
  ASSERT_NO_THROW(filter.add(scan_after_reset));
  EXPECT_TRUE(spinUntil([&received]() {return received == 2;}));
}

INSTANTIATE_TEST_SUITE_P(
  ListenerThreading, TFWrapperClearFixture, testing::Bool(),
  [](const testing::TestParamInfo<bool> & info) {
    return info.param ? std::string("dedicated_thread") : std::string("rviz_node");
  });
