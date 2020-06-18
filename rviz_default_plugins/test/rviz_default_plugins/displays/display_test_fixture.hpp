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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__DISPLAY_TEST_FIXTURE_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__DISPLAY_TEST_FIXTURE_HPP_

#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <QApplication>  // NOLINT

#include <OgreRoot.h>

#include "rclcpp/clock.hpp"

#include "../ogre_testing_environment.hpp"

#include "../mock_display_context.hpp"
#include "../mock_frame_manager.hpp"
#include "../mock_selection_manager.hpp"
#include "../mock_handler_manager.hpp"

class DisplayTestFixture : public testing::Test
{
public:
  static void SetUpTestCase();

  DisplayTestFixture();

  ~DisplayTestFixture() override;

  static void TearDownTestCase();

  void mockValidTransform();

  void mockValidTransform(Ogre::Vector3 position, Ogre::Quaternion orientation);

  static std::shared_ptr<rviz_default_plugins::OgreTestingEnvironment> testing_environment_;
  static Ogre::SceneManager * scene_manager_;

  std::shared_ptr<MockDisplayContext> context_;
  std::shared_ptr<MockFrameManager> frame_manager_;
  std::shared_ptr<MockSelectionManager> selection_manager_;
  std::shared_ptr<MockHandlerManager> handler_manager_;
  std::shared_ptr<rclcpp::Clock> clock_;

  std::string fixed_frame = "fixed_frame";
};


#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__DISPLAY_TEST_FIXTURE_HPP_
