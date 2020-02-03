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

#ifndef RVIZ_DEFAULT_PLUGINS__MOCK_FRAME_MANAGER_HPP_
#define RVIZ_DEFAULT_PLUGINS__MOCK_FRAME_MANAGER_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

class MockFrameManager : public rviz_common::FrameManagerIface
{
public:
  MOCK_METHOD1(setFixedFrame, void(const std::string &));
  MOCK_METHOD1(setPause, void(bool));
  MOCK_METHOD0(getPause, bool());
  MOCK_METHOD1(setSyncMode, void(SyncMode));
  MOCK_METHOD0(getSyncMode, SyncMode());
  MOCK_METHOD1(syncTime, void(rclcpp::Time));
  MOCK_METHOD0(getTime, rclcpp::Time());

  MOCK_METHOD3(getTransform, bool(const std::string &, Ogre::Vector3 &, Ogre::Quaternion &));
  MOCK_METHOD4(
    getTransform, bool(
      const std::string &, rclcpp::Time, Ogre::Vector3 &, Ogre::Quaternion &));
  MOCK_METHOD5(
    transform, bool(
      const std::string &,
      rclcpp::Time,
      const geometry_msgs::msg::Pose &,
      Ogre::Vector3 &,
      Ogre::Quaternion &));
  MOCK_METHOD0(update, void());
  MOCK_METHOD2(frameHasProblems, bool(const std::string &, std::string &));
  MOCK_METHOD2(transformHasProblems, bool(const std::string &, std::string &));
  MOCK_METHOD3(transformHasProblems, bool(const std::string &, rclcpp::Time, std::string &));
  MOCK_METHOD0(getFixedFrame, const std::string & ());
  MOCK_METHOD0(
    getConnector, rviz_common::transformation::TransformationLibraryConnector::WeakPtr());
  MOCK_METHOD0(getTransformer, std::shared_ptr<rviz_common::transformation::FrameTransformer>());
  MOCK_METHOD0(getAllFrameNames, std::vector<std::string>());
  MOCK_METHOD1(
    setTransformerPlugin,
    void(std::shared_ptr<rviz_common::transformation::FrameTransformer> transformer));

  MOCK_METHOD0(fixedFrameChanged, void());
};

#endif  // RVIZ_DEFAULT_PLUGINS__MOCK_FRAME_MANAGER_HPP_
