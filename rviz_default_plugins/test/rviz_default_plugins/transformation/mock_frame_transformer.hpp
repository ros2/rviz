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

#ifndef RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__MOCK_FRAME_TRANSFORMER_HPP_
#define RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__MOCK_FRAME_TRANSFORMER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/transformation/frame_transformer.hpp"

class MockFrameTransformer : public rviz_common::transformation::FrameTransformer
{
public:
  MOCK_METHOD2(
    initialize, void(
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr, rclcpp::Clock::SharedPtr));
  MOCK_METHOD0(clear, void());
  MOCK_CONST_METHOD0(getAllFrameNames, std::vector<std::string>());
  MOCK_METHOD2(
    transform, geometry_msgs::msg::PoseStamped(
      const geometry_msgs::msg::PoseStamped &, const std::string &));
  MOCK_CONST_METHOD3(
    lookupTransform, geometry_msgs::msg::TransformStamped(
      const std::string &, const std::string &, const tf2::TimePoint &));
  MOCK_CONST_METHOD5(
    lookupTransform, geometry_msgs::msg::TransformStamped(
      const std::string &,
      const tf2::TimePoint &,
      const std::string &,
      const tf2::TimePoint &,
      const std::string &));
  MOCK_CONST_METHOD4(
    canTransform, bool(
      const std::string &, const std::string &, const tf2::TimePoint &, std::string *));
  MOCK_CONST_METHOD6(
    canTransform, bool(
      const std::string &,
      const tf2::TimePoint &,
      const std::string &,
      const tf2::TimePoint &,
      const std::string &,
      std::string *));
  MOCK_METHOD5(
    waitForTransform, tf2_ros::TransformStampedFuture(
      const std::string &,
      const std::string &,
      const tf2::TimePoint &,
      const tf2::Duration &,
      tf2_ros::TransformReadyCallback));
  MOCK_METHOD1(cancel, void(const tf2_ros::TransformStampedFuture &));
  MOCK_CONST_METHOD2(frameHasProblems, bool(const std::string &, std::string &));
  MOCK_METHOD0(
    getConnector,
    rviz_common::transformation::TransformationLibraryConnector::WeakPtr());
};

#endif  // RVIZ_DEFAULT_PLUGINS__TRANSFORMATION__MOCK_FRAME_TRANSFORMER_HPP_
