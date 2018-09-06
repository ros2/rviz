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
#include <string>
#include <vector>

#include "rviz_common/transformation/frame_transformer.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

class MockFrameTransformer : public rviz_common::transformation::FrameTransformer
{
public:
  MOCK_METHOD2(initialize, void(
      rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
      rclcpp::Clock::SharedPtr clock));
  MOCK_METHOD0(clear, void());
  MOCK_METHOD0(getAllFrameNames, std::vector<std::string>());
  MOCK_METHOD2(transform, rviz_common::transformation::PoseStamped(
      const rviz_common::transformation::PoseStamped & pose_in, const std::string & frame));
  MOCK_METHOD2(transformIsAvailable, bool(
      const std::string & target_frame, const std::string & source_frame));
  MOCK_METHOD4(transformHasProblems, bool(
      const std::string & frame,
      const std::string & fixed_frame,
      const rclcpp::Time & time,
      std::string & error));
  MOCK_METHOD2(frameHasProblems, bool(const std::string & frame, std::string & error));
  MOCK_METHOD0(getConnector,
    rviz_common::transformation::TransformationLibraryConnector::WeakPtr());
};

#endif  // MOCK_FRAME_TRANSFORMER_HPP_
