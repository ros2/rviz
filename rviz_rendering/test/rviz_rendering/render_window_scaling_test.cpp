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

#include <ostream>
#include <vector>

#include <QApplication>  // NOLINT
#include <QResizeEvent>  // NOLINT
#include <QTest>  // NOLINT
#include <OgreViewport.h>  // NOLINT

#include "rviz_rendering/render_window.hpp"
#include "ogre_testing_environment.hpp"

using rviz_rendering::RenderWindowOgreAdapter;

void PrintTo(const QSize & size, std::ostream * os)
{
  *os << size.width() << "x" << size.height();
}

namespace
{

QSize viewportSize(const Ogre::Viewport * viewport)
{
  return QSize(viewport->getActualWidth(), viewport->getActualHeight());
}

// Records every size the viewport takes, including one that is
// corrected again before the resize returns.
class ViewportSizeRecorder : public Ogre::Viewport::Listener
{
public:
  explicit ViewportSizeRecorder(Ogre::Viewport * viewport)
  : viewport_(viewport)
  {
    viewport_->addListener(this);
  }

  ~ViewportSizeRecorder() override
  {
    viewport_->removeListener(this);
  }

  void viewportDimensionsChanged(Ogre::Viewport * viewport) override
  {
    sizes.push_back(viewportSize(viewport));
  }

  std::vector<QSize> sizes;

private:
  Ogre::Viewport * viewport_;
};

// Holds rendering while waiting for a resize, and records the viewport size
// right after the window has handled that resize.
class ResizeProbeWindow : public rviz_rendering::RenderWindow
{
public:
  bool hold_rendering = false;
  QSize resized_to;
  QSize viewport_size_after_resize;

protected:
  bool event(QEvent * event) override
  {
    if (hold_rendering && event->type() == QEvent::UpdateRequest) {
      return true;
    }
    const bool handled = RenderWindow::event(event);
    const auto * viewport = RenderWindowOgreAdapter::getOgreViewport(this);
    if (event->type() == QEvent::Resize && viewport) {
      resized_to = static_cast<QResizeEvent *>(event)->size();
      viewport_size_after_resize = viewportSize(viewport);
    }
    return handled;
  }

  void exposeEvent(QExposeEvent * event) override
  {
    if (!hold_rendering) {
      RenderWindow::exposeEvent(event);
    }
  }
};

}  // namespace

TEST(RenderWindowScaling, viewport_matches_native_size_after_resizing)
{
  rviz_rendering::OgreTestingEnvironment environment;
  environment.setUpOgreTestEnvironment();
  ResizeProbeWindow window;
#ifdef _WIN32
  // Ogre's external Win32 window does not account for frame decorations.
  // Match the frameless window embedded in RenderPanel.
  window.setFlag(Qt::FramelessWindowHint);
#endif
  // Multiples of 16 remain exact when the tested scales combine with quarter-step
  // desktop scales. Odd sizes can be rounded in multiple coordinate spaces by Qt
  // and the window system; their conversion is covered in pixel_scaling_test.
  window.resize(320, 240);
  window.show();
  ASSERT_TRUE(QTest::qWaitForWindowExposed(&window));
  window.renderNow();

  auto * viewport = RenderWindowOgreAdapter::getOgreViewport(&window);
  ASSERT_NE(viewport, nullptr);
  EXPECT_EQ(viewportSize(viewport), window.size() * window.devicePixelRatio());

  ViewportSizeRecorder recorder(viewport);
  for (const auto & size : {QSize(512, 304), QSize(256, 192), QSize(320, 240)}) {
    const auto native_size = size * window.devicePixelRatio();
    recorder.sizes.clear();

    window.hold_rendering = true;
    window.resize(size);
    ASSERT_TRUE(QTest::qWaitFor([&window, &size]() {return window.resized_to == size;}));
    window.hold_rendering = false;
    EXPECT_EQ(window.size(), size);
    // Nothing has rendered since the resize, so a fix deferred to the next frame fails here.
    EXPECT_EQ(window.viewport_size_after_resize, native_size);

    window.renderNow();
    EXPECT_EQ(viewportSize(viewport), native_size);
    // Includes sizes that Ogre corrected again before the resize returned.
    EXPECT_THAT(recorder.sizes, testing::Each(native_size));
  }
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
