// Copyright (c) 2026, John C. Furey
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

#include <QApplication>  // NOLINT
#include <OgreCamera.h>  // NOLINT
#include <OgreRoot.h>  // NOLINT
#include <OgreSceneManager.h>  // NOLINT

#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"
#include "mock_display_context.hpp"
#include "ogre_testing_environment.hpp"
#include "rviz_rendering/render_system.hpp"

using rviz_rendering::RenderWindowOgreAdapter;

class RenderPanelTest : public testing::Test
{
protected:
  void SetUp() override
  {
    environment_.setUpOgreTestEnvironment();
    scene_ = rviz_rendering::RenderSystem::get()->getOgreRoot()->createSceneManager();
    ON_CALL(context_, getSceneManager()).WillByDefault(testing::Return(scene_));
  }

  void TearDown() override
  {
    rviz_rendering::RenderSystem::get()->getOgreRoot()->destroySceneManager(scene_);
  }

  rviz_common::OgreTestingEnvironment environment_;
  testing::NiceMock<MockDisplayContext> context_;
  Ogre::SceneManager * scene_;
};

TEST_F(RenderPanelTest, destroying_panels_releases_their_cameras_and_nodes)
{
  const auto cameras = scene_->getCameras().size();
  const auto nodes = scene_->getRootSceneNode()->numChildren();
  for (int cycle = 0; cycle < 5; ++cycle) {
    {
      rviz_common::RenderPanel panel;
      panel.initialize(&context_, true);
      panel.getRenderWindow()->resize(64, 64);
      panel.getRenderWindow()->initialize();
      EXPECT_EQ(scene_->getCameras().size(), cameras + 1);
      EXPECT_EQ(scene_->getRootSceneNode()->numChildren(), nodes + 1);
    }
    EXPECT_EQ(scene_->getCameras().size(), cameras);
    EXPECT_EQ(scene_->getRootSceneNode()->numChildren(), nodes);
  }
}

TEST_F(RenderPanelTest, destroying_panel_preserves_a_borrowed_camera)
{
  auto * borrowed_camera = scene_->createCamera("BorrowedCamera");
  {
    rviz_common::RenderPanel panel;
    panel.initialize(&context_, true);
    auto * window = panel.getRenderWindow();
    window->resize(64, 64);
    window->initialize();
    RenderWindowOgreAdapter::setOgreCamera(window, borrowed_camera);
  }
  ASSERT_TRUE(scene_->hasCamera("BorrowedCamera"));
  EXPECT_EQ(scene_->getCameras().size(), 1u);
  EXPECT_EQ(scene_->getRootSceneNode()->numChildren(), 0u);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
