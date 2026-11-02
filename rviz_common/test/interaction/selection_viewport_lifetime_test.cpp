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

#include <OgreCamera.h>
#include <OgreRectangle2D.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <QApplication>  // NOLINT

#include "rviz_rendering/render_window.hpp"
#include "selection_test_fixture.hpp"

using rviz_rendering::RenderWindow;
using rviz_rendering::RenderWindowOgreAdapter;

class SelectionViewportLifetimeTest : public SelectionTestFixture
{
protected:
  std::unique_ptr<RenderWindow> makeWindow()
  {
    auto window = std::make_unique<RenderWindow>();
    window->resize(64, 64);
    static int count = 0;
    auto * camera = scene_manager_->createCamera("HighlightCamera" + std::to_string(count++));
    RenderWindowOgreAdapter::setSceneManager(window.get(), scene_manager_);
    RenderWindowOgreAdapter::setOgreCamera(window.get(), camera);
    window->initialize();
    return window;
  }

  Ogre::Rectangle2D * highlightRectangle()
  {
    for (auto * child : scene_manager_->getRootSceneNode()->getChildren()) {
      auto * node = static_cast<Ogre::SceneNode *>(child);
      for (auto * object : node->getAttachedObjects()) {
        if (auto * rectangle = dynamic_cast<Ogre::Rectangle2D *>(object)) {
          return rectangle;
        }
      }
    }
    return nullptr;
  }
};

TEST_F(SelectionViewportLifetimeTest, destroying_surface_cancels_highlight_and_allows_a_new_one)
{
  auto window = makeWindow();
  auto * rectangle = highlightRectangle();
  ASSERT_NE(rectangle, nullptr);
  selection_manager_->highlight(window.get(), 0, 0, 32, 32);
  selection_manager_->update();
  ASSERT_TRUE(rectangle->getVisible());

  window->destroy();
  selection_manager_->update();

  EXPECT_FALSE(rectangle->getVisible());
  window->create();
  window->initialize();
  selection_manager_->highlight(window.get(), 0, 0, 32, 32);
  selection_manager_->update();
  EXPECT_TRUE(rectangle->getVisible());
}

TEST_F(SelectionViewportLifetimeTest, deleting_window_cancels_highlight)
{
  auto window = makeWindow();
  auto * rectangle = highlightRectangle();
  ASSERT_NE(rectangle, nullptr);
  selection_manager_->highlight(window.get(), 0, 0, 32, 32);
  selection_manager_->update();
  ASSERT_TRUE(rectangle->getVisible());

  window.reset();
  selection_manager_->update();

  EXPECT_FALSE(rectangle->getVisible());
}

TEST_F(SelectionViewportLifetimeTest, destroying_a_previous_window_preserves_current_highlight)
{
  auto first = makeWindow();
  auto second = makeWindow();
  auto * rectangle = highlightRectangle();
  ASSERT_NE(rectangle, nullptr);
  selection_manager_->highlight(first.get(), 0, 0, 32, 32);
  selection_manager_->highlight(second.get(), 0, 0, 32, 32);

  first->destroy();
  selection_manager_->update();

  EXPECT_TRUE(rectangle->getVisible());
}

TEST_F(SelectionViewportLifetimeTest, destroying_selection_manager_before_window_is_safe)
{
  auto window = makeWindow();
  selection_manager_->highlight(window.get(), 0, 0, 32, 32);

  selection_manager_.reset();
  window->destroy();
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
