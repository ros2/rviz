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

#ifndef MOCK_FRAME_TRANSFORMER_HPP_
#define MOCK_FRAME_TRANSFORMER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <vector>

#include "rviz_common/frame_transformer.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

class MockFrameTransformer : public rviz_common::FrameTransformer
{
public:
  MOCK_METHOD1(
    initialize, void(rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node));
  MOCK_METHOD0(clear, void());
  MOCK_METHOD0(getAllFrameNames, std::vector<std::string>());
  MOCK_METHOD3(transform, bool(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out,
    const std::string & frame));
  MOCK_METHOD3(lastAvailableTransform, bool(
    const std::string & target_frame,
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform));
  MOCK_METHOD4(transformHasProblems, bool(
    const std::string & frame,
    const std::string & fixed_frame,
    const rclcpp::Time & time,
    std::string & error));
  MOCK_METHOD2(frameHasProblems, bool(const std::string & frame, std::string & error));
  MOCK_METHOD0(getInternals, rviz_common::InternalFrameTransformerPtr());
};

#endif  // MOCK_FRAME_TRANSFORMER_HPP_
