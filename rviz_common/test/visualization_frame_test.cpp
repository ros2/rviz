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

#include <string>
#include <vector>

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <QAction>  // NOLINT
#include <QApplication>  // NOLINT
#include <QMenu>  // NOLINT

#include "rviz_common/panel.hpp"
#include "rviz_common/panel_dock_widget.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/visualization_frame.hpp"
#include "rviz_rendering/render_system.hpp"
#include "rviz_rendering/render_window.hpp"

#include "ogre_testing_environment.hpp"

namespace
{

struct PanelCleanup
{
  bool destroyed = false;
  bool scene_alive = false;
  bool node_released = false;
};

// A custom panel that owns an Ogre object in the frame's main scene.
class SceneResourcePanel : public rviz_common::Panel
{
public:
  SceneResourcePanel(Ogre::SceneManager * scene, PanelCleanup & cleanup)
  : scene_name_(scene->getName()),
    node_(scene->getRootSceneNode()->createChildSceneNode()),
    cleanup_(cleanup)
  {}

  ~SceneResourcePanel() override
  {
    cleanup_.destroyed = true;
    auto * root = Ogre::Root::getSingletonPtr();
    cleanup_.scene_alive = root && root->hasSceneManager(scene_name_);
    if (cleanup_.scene_alive) {
      auto * scene = root->getSceneManager(scene_name_);
      const auto nodes_before = scene->getRootSceneNode()->numChildren();
      scene->destroySceneNode(node_);
      cleanup_.node_released = scene->getRootSceneNode()->numChildren() + 1 == nodes_before;
    }
  }

private:
  std::string scene_name_;
  Ogre::SceneNode * node_;
  PanelCleanup & cleanup_;
};

class FrameWithScenePanels : public rviz_common::VisualizationFrame
{
public:
  FrameWithScenePanels()
  : VisualizationFrame({})
  {
    // Exercise the frame's real teardown without loading a user's configuration.
    render_panel_ = new rviz_common::RenderPanel(this);
    render_panel_->getRenderWindow()->resize(64, 64);
    render_panel_->getRenderWindow()->initialize();
    delete_view_menu_ = new QMenu(this);
  }

  Ogre::SceneManager * scene()
  {
    return rviz_rendering::RenderWindowOgreAdapter::getSceneManager(getRenderWindow());
  }

  void addScenePanel(PanelCleanup & cleanup)
  {
    auto name = QString("Scene panel %1").arg(custom_panels_.size());
    auto * panel = new SceneResourcePanel(scene(), cleanup);
    auto * dock = new rviz_common::PanelDockWidget(name);
    dock->setContentWidget(panel);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
    auto * action = new QAction(name, this);
    delete_view_menu_->addAction(action);
    custom_panels_.push_back({panel, dock, name, "SceneResourcePanel", action});
    connect(dock, &QObject::destroyed, this, &FrameWithScenePanels::onPanelDeleted);
  }
};

class VisualizationFrameLifetimeTest : public testing::TestWithParam<int>
{
protected:
  void SetUp() override
  {
    environment_.setUpOgreTestEnvironment();
  }

  rviz_common::OgreTestingEnvironment environment_;
};

TEST_P(VisualizationFrameLifetimeTest, custom_panels_release_resources_before_the_main_scene)
{
  std::vector<PanelCleanup> cleanup(GetParam());
  std::string scene_name;
  {
    FrameWithScenePanels frame;
    scene_name = frame.scene()->getName();
    for (auto & panel_cleanup : cleanup) {
      frame.addScenePanel(panel_cleanup);
    }
  }

  for (const auto & panel_cleanup : cleanup) {
    EXPECT_TRUE(panel_cleanup.destroyed);
    EXPECT_TRUE(panel_cleanup.scene_alive);
    EXPECT_TRUE(panel_cleanup.node_released);
  }
  EXPECT_FALSE(Ogre::Root::getSingleton().hasSceneManager(scene_name));
}

INSTANTIATE_TEST_SUITE_P(
  PanelCounts, VisualizationFrameLifetimeTest, testing::Values(1, 2));

}  // namespace

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
