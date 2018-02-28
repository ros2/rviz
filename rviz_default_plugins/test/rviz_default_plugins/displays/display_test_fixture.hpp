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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>
#include <string>

#include <QApplication>  // NOLINT

#include <OgreRoot.h>

#include "rclcpp/clock.hpp"

#include "test/rviz_rendering/ogre_testing_environment.hpp"

#include "../mock_display_context.hpp"
#include "../mock_frame_manager.hpp"
#include "../mock_selection_manager.hpp"

class DisplayTestFixture : public testing::Test
{
public:
  static void SetUpTestCase()
  {
    testing_environment_ = std::make_shared<rviz_rendering::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();

    scene_manager_ = Ogre::Root::getSingletonPtr()->createSceneManager();
  }

  void SetUp() override
  {
    context_ = std::make_shared<testing::NiceMock<MockDisplayContext>>();
    frame_manager_ = std::make_shared<testing::NiceMock<MockFrameManager>>();
    selection_manager_ = std::make_shared<testing::NiceMock<MockSelectionManager>>();
    clock_ = std::make_shared<rclcpp::Clock>();

    EXPECT_CALL(*frame_manager_, getFixedFrame()).WillRepeatedly(testing::ReturnRef(fixed_frame));

    EXPECT_CALL(*context_, getClock()).WillRepeatedly(testing::Return(clock_));
    EXPECT_CALL(*context_, getSceneManager()).WillRepeatedly(testing::Return(scene_manager_));
    EXPECT_CALL(*context_, getFrameManager()).WillRepeatedly(testing::Return(frame_manager_.get()));
    EXPECT_CALL(*context_, getSelectionManager()).WillRepeatedly(
      testing::Return(selection_manager_.get()));
  }

  void TearDown() override
  {
    scene_manager_->getRootSceneNode()->removeAndDestroyAllChildren();
  }

  static void TearDownTestCase()
  {
    Ogre::Root::getSingletonPtr()->destroySceneManager(scene_manager_);
  }

  void mockValidTransform()
  {
    Ogre::Vector3 position(0, 1, 0);
    Ogre::Quaternion orientation(0, 0, 1, 0);
    mockValidTransform(position, orientation);
  }

  void mockValidTransform(Ogre::Vector3 position, Ogre::Quaternion orientation)
  {
    EXPECT_CALL(
      *frame_manager_,
      transform(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))  // NOLINT
    .WillRepeatedly(::testing::DoAll(    // NOLINT
        ::testing::SetArgReferee<3>(position),
        ::testing::SetArgReferee<4>(orientation),
        ::testing::Return(true)
      ));

    EXPECT_CALL(
      *frame_manager_,
      getTransform(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .WillRepeatedly(::testing::DoAll(    // NOLINT
        ::testing::SetArgReferee<2>(position),
        ::testing::SetArgReferee<3>(orientation),
        ::testing::Return(true)
      ));
  }

  static std::shared_ptr<rviz_rendering::OgreTestingEnvironment> testing_environment_;
  static Ogre::SceneManager * scene_manager_;

  std::shared_ptr<MockDisplayContext> context_;
  std::shared_ptr<MockFrameManager> frame_manager_;
  std::shared_ptr<MockSelectionManager> selection_manager_;
  std::shared_ptr<rclcpp::Clock> clock_;

  std::string fixed_frame = "fixed_frame";
};

Ogre::SceneManager * DisplayTestFixture::scene_manager_ = nullptr;
std::shared_ptr<rviz_rendering::OgreTestingEnvironment>
DisplayTestFixture::testing_environment_ = nullptr;

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__DISPLAY_TEST_FIXTURE_HPP_
