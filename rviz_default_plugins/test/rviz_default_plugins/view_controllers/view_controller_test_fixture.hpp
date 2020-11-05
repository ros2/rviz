/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__VIEW_CONTROLLER_TEST_FIXTURE_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__VIEW_CONTROLLER_TEST_FIXTURE_HPP_

#include <memory>

#include <OgreViewport.h>

#include "../displays/display_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class ViewControllerTestFixture : public DisplayTestFixture
{
public:
  static void SetUpTestCase()
  {
    DisplayTestFixture::SetUpTestCase();
  }

  ViewControllerTestFixture()
  {
    render_panel_ = std::make_shared<rviz_common::RenderPanel>(nullptr);
  }

  static void TearDownTestCase()
  {
    DisplayTestFixture::TearDownTestCase();
  }

  void TearDown() override
  {
    render_panel_.reset();
  }

  void dragMouseInViewport(
    std::shared_ptr<rviz_common::ViewController> view_controller,
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    auto click = generateMousePressEvent(from_x, from_y, button, modifiers);
    view_controller->handleMouseEvent(click);
    auto move = generateMouseMoveEvent(to_x, to_y, from_x, from_y, button, modifiers);
    view_controller->handleMouseEvent(move);
    auto release = generateMouseReleaseEvent(to_x, to_y, button, modifiers);
    view_controller->handleMouseEvent(release);
  }

  rviz_common::ViewportMouseEvent generateMouseMoveEvent(
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    auto mouseEvent = new QMouseEvent(
      QMouseEvent::MouseMove, QPointF(to_x, to_y), Qt::LeftButton, button, modifiers);
    return {render_panel_.get(), mouseEvent, from_x, from_y};
  }

  rviz_common::ViewportMouseEvent generateMousePressEvent(
    int x, int y, Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    return generateMouseEvent(x, y, QMouseEvent::MouseButtonPress, button, modifiers);
  }

  rviz_common::ViewportMouseEvent generateMouseReleaseEvent(
    int x, int y, Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    return generateMouseEvent(x, y, QMouseEvent::MouseButtonRelease, button, modifiers);
  }

  rviz_common::ViewportMouseEvent generateMouseWheelEvent(int delta)
  {
    auto point = QPointF();
#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    auto global_point = QPointF();
    auto pixel_delta = QPoint();
    auto angle_delta = QPoint(0, delta);
    auto mouseEvent = new QWheelEvent(
      point,
      global_point,
      pixel_delta,
      angle_delta,
      Qt::NoButton,
      Qt::NoModifier,
      Qt::NoScrollPhase,
      false);
#else
    auto mouseEvent = new QWheelEvent(
      point, delta, Qt::NoButton, Qt::NoModifier);
#endif
    return {render_panel_.get(), mouseEvent, 0, 0};
  }

protected:
  // This function call is necessary as on different systems the render window may have different
  // "real" dimensions. This function corrects this behaviour for testing
  void setOSIndependentDimensions(
    Ogre::Viewport * viewport, float actual_width, float actual_height)
  {
    auto width = viewport->getActualHeight();
    auto height = viewport->getActualWidth();
    viewport->setDimensions(0, 0, actual_width / width, actual_height / height);
  }

private:
  rviz_common::ViewportMouseEvent generateMouseEvent(
    int x, int y, QMouseEvent::Type type, Qt::MouseButton button, Qt::KeyboardModifiers modifiers)
  {
    auto mouseEvent = new QMouseEvent(type, QPointF(x, y), button, button, modifiers);
    return {render_panel_.get(), mouseEvent, x, y};
  }

public:
  std::shared_ptr<rviz_common::RenderPanel> render_panel_;
};

class MockViewController : public rviz_common::ViewController
{
public:
  MOCK_METHOD1(lookAt, void(const Ogre::Vector3 &));
  MOCK_METHOD0(reset, void());
};

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__VIEW_CONTROLLER_TEST_FIXTURE_HPP_
