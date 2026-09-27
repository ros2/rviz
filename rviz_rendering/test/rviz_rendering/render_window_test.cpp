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
#include <OgreRenderTarget.h>  // NOLINT
#include <OgreRenderTargetListener.h>  // NOLINT
#include <OgreRoot.h>  // NOLINT
#include <OgreViewport.h>  // NOLINT

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

class CountingRenderListener : public Ogre::RenderTargetListener
{
public:
  void preRenderTargetUpdate(const Ogre::RenderTargetEvent &) override
  {
    ++updates;
  }

  int updates = 0;
};

TEST_F(RenderWindowTest, destroying_native_surface_releases_render_target)
{
  RenderWindow window;
  window.resize(64, 64);
  window.initialize();
  const auto target_name =
    RenderWindowOgreAdapter::getOgreViewport(&window)->getTarget()->getName();
  auto * root = rviz_rendering::RenderSystem::get()->getOgreRoot();
  ASSERT_NE(root->getRenderTarget(target_name), nullptr);

  window.destroy();

  EXPECT_EQ(root->getRenderTarget(target_name), nullptr);
  EXPECT_EQ(RenderWindowOgreAdapter::getOgreViewport(&window), nullptr);
}

TEST_F(RenderWindowTest, recreating_surface_preserves_scene_and_viewport_settings)
{
  RenderWindow window;
  window.resize(64, 64);
  window.initialize();
  auto * scene = RenderWindowOgreAdapter::getSceneManager(&window);
  auto * camera = RenderWindowOgreAdapter::getOgreCamera(&window);
  CountingRenderListener listener;
  RenderWindowOgreAdapter::addListener(&window, &listener);
  const Ogre::ColourValue color(0.1f, 0.2f, 0.3f);
  RenderWindowOgreAdapter::setBackgroundColor(&window, &color);
  RenderWindowOgreAdapter::setVisibilityMask(&window, 0x1234u);

  for (int cycle = 0; cycle < 3; ++cycle) {
    window.destroy();
    ASSERT_EQ(RenderWindowOgreAdapter::getOgreViewport(&window), nullptr);
    window.render();  // A queued render must be harmless while the surface is absent.
    window.create();
    window.initialize();
    EXPECT_EQ(RenderWindowOgreAdapter::getSceneManager(&window), scene);
    EXPECT_EQ(RenderWindowOgreAdapter::getOgreCamera(&window), camera);
    const auto * viewport = RenderWindowOgreAdapter::getOgreViewport(&window);
    ASSERT_NE(viewport, nullptr);
    EXPECT_EQ(viewport->getBackgroundColour(), color);
    EXPECT_EQ(viewport->getVisibilityMask(), 0x1234u);
    window.render();
    EXPECT_EQ(listener.updates, cycle + 1);
  }
  RenderWindowOgreAdapter::removeListener(&window, &listener);
  window.destroy();
  window.create();
  window.initialize();
  window.render();
  EXPECT_EQ(listener.updates, 3);
}

TEST_F(RenderWindowTest, initializing_twice_preserves_render_target)
{
  RenderWindow window;
  window.resize(64, 64);
  window.initialize();
  const auto target_name =
    RenderWindowOgreAdapter::getOgreViewport(&window)->getTarget()->getName();
  window.initialize();
  EXPECT_EQ(
    RenderWindowOgreAdapter::getOgreViewport(&window)->getTarget()->getName(), target_name);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
