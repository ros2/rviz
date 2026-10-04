// Copyright (c) 2026, Miko Parkkinen
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

#include <vector>

#include <QApplication>  // NOLINT
#include <QElapsedTimer>  // NOLINT
#include <QEventLoop>  // NOLINT
#include <QKeyEvent>  // NOLINT
#include <QLineEdit>  // NOLINT
#include <QMouseEvent>  // NOLINT
#include <QThread>  // NOLINT
#include <QVBoxLayout>  // NOLINT

#include "rviz_common/render_panel.hpp"
#include "rviz_rendering/render_window.hpp"

#include "display_context_fixture.hpp"

namespace
{

void sendKey(QObject * receiver, int key, const QString & text = QString())
{
  QKeyEvent press(QEvent::KeyPress, key, Qt::NoModifier, text);
  QApplication::sendEvent(receiver, &press);
  QKeyEvent release(QEvent::KeyRelease, key, Qt::NoModifier, text);
  QApplication::sendEvent(receiver, &release);
}

void dragMouse(QWindow * window, Qt::MouseButton button)
{
  const QPointF start(40, 40);
  const QPointF finish(60, 50);
  QMouseEvent press(
    QEvent::MouseButtonPress, start, start, button, button, Qt::NoModifier);
  QApplication::sendEvent(window, &press);
  QMouseEvent move(
    QEvent::MouseMove, finish, finish, Qt::NoButton, button, Qt::NoModifier);
  QApplication::sendEvent(window, &move);
  QMouseEvent release(
    QEvent::MouseButtonRelease, finish, finish, button, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(window, &release);
}

template<typename Predicate>
bool waitUntil(Predicate condition)
{
  QElapsedTimer timer;
  timer.start();
  while (!condition() && timer.elapsed() < 5000) {
    QApplication::processEvents(QEventLoop::AllEvents, 10);
    QThread::msleep(1);
  }
  return condition();
}

}  // namespace

class RenderPanelTest : public DisplayContextFixture
{};

TEST_F(RenderPanelTest, render_window_delivers_keys_after_mouse_interactions)
{
  rviz_common::RenderPanel panel;
  panel.initialize(context_.get());
  auto * render_window = panel.getRenderWindow();
  std::vector<int> keys;
  EXPECT_CALL(*context_, handleChar(testing::_, &panel)).WillRepeatedly(
    testing::Invoke([&keys](QKeyEvent * event, rviz_common::RenderPanel *) {
      keys.push_back(event->key());
    }));
  EXPECT_CALL(*context_, handleMouseEvent(testing::_)).Times(6);

  sendKey(&panel, Qt::Key_F);
  EXPECT_THAT(keys, testing::ElementsAre(Qt::Key_F));

  for (const auto button : {Qt::LeftButton, Qt::RightButton}) {
    SCOPED_TRACE(static_cast<int>(button));
    keys.clear();
    dragMouse(render_window, button);
    // Native mouse activation can make the contained QWindow the keyboard receiver.
    // Dispatch through that production boundary, rather than calling a panel handler.
    sendKey(render_window, Qt::Key_F);
    sendKey(render_window, Qt::Key_M);
    EXPECT_THAT(keys, testing::ElementsAre(Qt::Key_F, Qt::Key_M));
  }
}

TEST_F(RenderPanelTest, render_window_preserves_tab_navigation_and_text_input)
{
  QWidget window;
  QVBoxLayout layout(&window);
  rviz_common::RenderPanel panel(&window);
  QLineEdit field(&window);
  layout.addWidget(&panel);
  layout.addWidget(&field);
  panel.initialize(context_.get(), true);
  window.resize(400, 320);
  window.show();
  window.activateWindow();
  auto * render_window = panel.getRenderWindow();
  ASSERT_TRUE(waitUntil([&window]() {return window.isActiveWindow();}));
  render_window->requestActivate();
  ASSERT_TRUE(waitUntil([render_window]() {
      return QGuiApplication::focusWindow() == render_window;
  }));

  EXPECT_CALL(*context_, handleChar(testing::_, testing::_)).Times(0);
  sendKey(QGuiApplication::focusWindow(), Qt::Key_Tab);
  ASSERT_TRUE(waitUntil([&field, render_window]() {
      return field.hasFocus() && QGuiApplication::focusWindow() != render_window;
  }));
  sendKey(QGuiApplication::focusWindow(), Qt::Key_F, QStringLiteral("f"));
  sendKey(QGuiApplication::focusWindow(), Qt::Key_M, QStringLiteral("m"));
  EXPECT_EQ(field.text(), QStringLiteral("fm"));
  testing::Mock::VerifyAndClearExpectations(context_.get());

  render_window->requestActivate();
  ASSERT_TRUE(waitUntil([render_window]() {
      return QGuiApplication::focusWindow() == render_window;
  }));
  EXPECT_CALL(*context_, handleChar(testing::_, &panel)).Times(1);
  sendKey(QGuiApplication::focusWindow(), Qt::Key_M);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
