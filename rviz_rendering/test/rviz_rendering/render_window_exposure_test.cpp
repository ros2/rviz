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

#include <OgreViewport.h>

#include <QApplication>  // NOLINT
#include <QExposeEvent>  // NOLINT
#include <QResizeEvent>  // NOLINT
#include <QTest>  // NOLINT

#include "rviz_rendering/render_window.hpp"
#include "ogre_testing_environment.hpp"

using rviz_rendering::RenderWindowOgreAdapter;

class ExposureProbeWindow : public rviz_rendering::RenderWindow, public Ogre::Viewport::Listener
{
public:
  ~ExposureProbeWindow() override
  {
    if (viewport_) {
      viewport_->removeListener(this);
    }
  }

  void renderNow() override
  {
    RenderWindow::renderNow();
    if (!viewport_) {
      viewport_ = RenderWindowOgreAdapter::getOgreViewport(this);
      if (viewport_) {
        viewport_->addListener(this);
        if (hide_after_first_render) {
          hide();
        }
      }
    }
  }

  void viewportDimensionsChanged(Ogre::Viewport *) override
  {
    ++viewport_updates;
  }

  void deliverExpose()
  {
    QExposeEvent event(QRegion(QRect(QPoint(), size())));
    exposeEvent(&event);
  }

  QSize viewportSize() const
  {
    return viewport_ ? QSize(viewport_->getActualWidth(), viewport_->getActualHeight()) : QSize();
  }

  int viewport_updates = 0;
  int hidden_resizes = 0;
  bool hide_after_first_render = false;

protected:
  bool event(QEvent * event) override
  {
    if (event->type() == QEvent::Resize && !isExposed()) {
      ++hidden_resizes;
    }
    return RenderWindow::event(event);
  }

private:
  Ogre::Viewport * viewport_ = nullptr;
};

class RenderWindowExposureTest : public testing::Test
{
protected:
  void SetUp() override
  {
    environment_.setUpOgreTestEnvironment();
    window_ = std::make_unique<ExposureProbeWindow>();
    window_->resize(320, 240);
  }

  bool showAndWaitForInitialResize()
  {
    window_->show();
    return QTest::qWaitForWindowExposed(window_.get()) &&
           QTest::qWaitFor([this]() {return window_->viewport_updates > 0;});
  }

  QSize expectedViewportSize() const
  {
    return window_->size() * window_->devicePixelRatio();
  }

  rviz_rendering::OgreTestingEnvironment environment_;
  std::unique_ptr<ExposureProbeWindow> window_;
};

TEST_F(RenderWindowExposureTest, initial_exposure_updates_the_backing_surface)
{
  ASSERT_TRUE(showAndWaitForInitialResize());
  EXPECT_EQ(window_->size(), QSize(320, 240));
  EXPECT_EQ(window_->viewportSize(), expectedViewportSize());
}

TEST_F(RenderWindowExposureTest, repeated_exposure_does_not_resize_the_surface)
{
  ASSERT_TRUE(showAndWaitForInitialResize());
  QTest::qWait(25);
  const auto updates = window_->viewport_updates;

  for (int exposure = 0; exposure < 3; ++exposure) {
    window_->deliverExpose();
  }
  QTest::qWait(25);

  EXPECT_EQ(window_->viewport_updates, updates);
  EXPECT_EQ(window_->viewportSize(), expectedViewportSize());
}

TEST_F(RenderWindowExposureTest, resizing_while_hidden_updates_the_surface_when_shown)
{
  ASSERT_TRUE(showAndWaitForInitialResize());
  window_->hide();
  ASSERT_TRUE(QTest::qWaitFor([this]() {return !window_->isExposed();}));
  const auto old_size = window_->size();
  const auto hidden_resizes = window_->hidden_resizes;
  window_->resize(512, 304);
  // Deliver the resize while hidden, independently of platform event ordering.
  QResizeEvent resize_event(window_->size(), old_size);
  QApplication::sendEvent(window_.get(), &resize_event);
  QTest::qWait(25);
  ASSERT_GT(window_->hidden_resizes, hidden_resizes);

  window_->show();
  ASSERT_TRUE(QTest::qWaitForWindowExposed(window_.get()));
  EXPECT_TRUE(QTest::qWaitFor([this]() {
      return window_->viewportSize() == expectedViewportSize();
  }));
  EXPECT_EQ(window_->size(), QSize(512, 304));
}

TEST_F(RenderWindowExposureTest, hiding_before_the_deferred_resize_retries_when_shown)
{
  window_->hide_after_first_render = true;
  window_->show();
  ASSERT_TRUE(QTest::qWaitFor([this]() {
      return !window_->viewportSize().isEmpty() && !window_->isExposed();
  }));
  QTest::qWait(25);
  EXPECT_EQ(window_->viewport_updates, 0);

  ASSERT_TRUE(showAndWaitForInitialResize());
  EXPECT_EQ(window_->viewportSize(), expectedViewportSize());
}

enum class BackingScaleChange
{
  Screen,
  DevicePixelRatio
};

class RenderWindowBackingScaleTest : public RenderWindowExposureTest,
  public testing::WithParamInterface<BackingScaleChange>
{
protected:
  void deliverBackingScaleChange()
  {
    if (GetParam() == BackingScaleChange::Screen) {
      window_->screenChanged(window_->screen());
    } else {
#if QT_VERSION >= QT_VERSION_CHECK(6, 6, 0)
      QEvent event(QEvent::DevicePixelRatioChange);
      QApplication::sendEvent(window_.get(), &event);
#endif
    }
  }
};

TEST_P(RenderWindowBackingScaleTest, scale_change_without_resize_refreshes_once)
{
  ASSERT_TRUE(showAndWaitForInitialResize());
  QTest::qWait(25);
  const auto logical_size = window_->size();
  const auto updates = window_->viewport_updates;

  // Simulate the notification without changing logical size or physical displays.
  deliverBackingScaleChange();
  for (int exposure = 0; exposure < 3; ++exposure) {
    window_->deliverExpose();
  }
  ASSERT_TRUE(QTest::qWaitFor([this, updates]() {
      return window_->viewport_updates > updates;
  }));
  QTest::qWait(25);

  EXPECT_EQ(window_->viewport_updates, updates + 1);
  EXPECT_EQ(window_->size(), logical_size);
  EXPECT_EQ(window_->viewportSize(), expectedViewportSize());
}

TEST_P(RenderWindowBackingScaleTest, scale_change_while_hidden_refreshes_when_shown)
{
  ASSERT_TRUE(showAndWaitForInitialResize());
  QTest::qWait(25);
  const auto logical_size = window_->size();
  window_->hide();
  ASSERT_TRUE(QTest::qWaitFor([this]() {return !window_->isExposed();}));
  const auto updates = window_->viewport_updates;

  deliverBackingScaleChange();
  window_->deliverExpose();
  QTest::qWait(25);
  EXPECT_EQ(window_->viewport_updates, updates);

  window_->show();
  ASSERT_TRUE(QTest::qWaitForWindowExposed(window_.get()));
  ASSERT_TRUE(QTest::qWaitFor([this, updates]() {
      return window_->viewport_updates > updates;
  }));
  QTest::qWait(25);

  EXPECT_EQ(window_->viewport_updates, updates + 1);
  EXPECT_EQ(window_->size(), logical_size);
  EXPECT_EQ(window_->viewportSize(), expectedViewportSize());
}

INSTANTIATE_TEST_SUITE_P(
  BackingScaleChanges, RenderWindowBackingScaleTest,
  testing::Values(
    BackingScaleChange::Screen
#if QT_VERSION >= QT_VERSION_CHECK(6, 6, 0)
    , BackingScaleChange::DevicePixelRatio
#endif
  ),
  [](const testing::TestParamInfo<BackingScaleChange> & info) {
    return info.param == BackingScaleChange::Screen ? "ScreenChange" : "DevicePixelRatioChange";
  });

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  app.setQuitOnLastWindowClosed(false);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
