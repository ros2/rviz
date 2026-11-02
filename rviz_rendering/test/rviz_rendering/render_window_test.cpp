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

#include <memory>
#include <string>

#include <QApplication>  // NOLINT
#include <OgreCamera.h>  // NOLINT
#include <OgreRoot.h>  // NOLINT
#include <OgreSceneManager.h>  // NOLINT

#include "rviz_rendering/render_system.hpp"
#include "rviz_rendering/render_window.hpp"
#include "ogre_testing_environment.hpp"

using rviz_rendering::RenderWindow;
using rviz_rendering::RenderWindowOgreAdapter;

class RenderWindowTest : public testing::Test
{
protected:
  void SetUp() override
  {
    environment_ = std::make_unique<rviz_rendering::OgreTestingEnvironment>();
    environment_->setUpOgreTestEnvironment();
  }

  std::unique_ptr<rviz_rendering::OgreTestingEnvironment> environment_;
};

TEST_F(RenderWindowTest, destroying_window_releases_its_private_scene)
{
  auto * root = rviz_rendering::RenderSystem::get()->getOgreRoot();
  std::string scene_name;
  {
    RenderWindow window;
    window.resize(64, 64);
    window.initialize();
    scene_name = RenderWindowOgreAdapter::getSceneManager(&window)->getName();
    ASSERT_TRUE(root->hasSceneManager(scene_name));
  }
  EXPECT_FALSE(root->hasSceneManager(scene_name));
}

TEST_F(RenderWindowTest, destroying_window_preserves_external_scene)
{
  auto * root = rviz_rendering::RenderSystem::get()->getOgreRoot();
  auto * scene = root->createSceneManager();
  const auto scene_name = scene->getName();
  {
    RenderWindow window;
    window.resize(64, 64);
    RenderWindowOgreAdapter::setSceneManager(&window, scene);
    RenderWindowOgreAdapter::setOgreCamera(&window, scene->createCamera("TestCamera"));
    window.initialize();
  }
  ASSERT_TRUE(root->hasSceneManager(scene_name));
  EXPECT_TRUE(scene->hasCamera("TestCamera"));
  root->destroySceneManager(scene);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
