// Copyright 2026 IOES Lab
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <OgreRenderTarget.h>
#include <OgreViewport.h>

#include <QApplication>  // NOLINT
#include <QElapsedTimer>  // NOLINT
#include <QPlatformSurfaceEvent>  // NOLINT
#include <QThread>  // NOLINT

#include "rviz_rendering/render_window.hpp"

namespace
{
bool waitForExposure(rviz_rendering::RenderWindow & window, bool exposed)
{
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < 5000) {
    QApplication::processEvents();
    if (window.isExposed() == exposed) {
      return true;
    }
    QThread::msleep(10);
  }
  return false;
}
}  // namespace

TEST(RenderWindowLifetime, hidden_and_destroying_surfaces_stop_global_rendering)
{
  rviz_rendering::RenderWindow window;
  window.resize(320, 240);
  window.show();
  ASSERT_TRUE(waitForExposure(window, true));
  window.renderNow();
  auto * viewport = rviz_rendering::RenderWindowOgreAdapter::getOgreViewport(&window);
  ASSERT_NE(viewport, nullptr);
  auto * target = viewport->getTarget();
  ASSERT_TRUE(target->isActive());

  window.hide();
  // Hide must deactivate immediately, before an unrelated Root render pass.
  EXPECT_FALSE(target->isActive());
  ASSERT_TRUE(waitForExposure(window, false));
  window.show();
  ASSERT_TRUE(waitForExposure(window, true));
  EXPECT_TRUE(target->isActive());

  QPlatformSurfaceEvent destroying(QPlatformSurfaceEvent::SurfaceAboutToBeDestroyed);
  QApplication::sendEvent(&window, &destroying);
  EXPECT_FALSE(target->isActive());
  window.close();
}

int main(int argc, char ** argv)
{
  QApplication application(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
