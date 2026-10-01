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
#include <utility>

#include <QApplication>  // NOLINT
#include <QMouseEvent>  // NOLINT
#include <QTest>  // NOLINT
#include <QWheelEvent>  // NOLINT

#include "rviz_common/render_panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/tool.hpp"
#include "rviz_common/tool_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "rviz_rendering/render_window.hpp"
#include "ogre_testing_environment.hpp"

class ViewportMouseEventTest : public testing::Test
{
protected:
  void SetUp() override
  {
    environment_.setUpOgreTestEnvironment();
  }

  rviz_common::OgreTestingEnvironment environment_;
};

TEST_F(ViewportMouseEventTest, mouse_coordinates_include_fractional_scaling)
{
  rviz_common::RenderPanel panel;
  const auto ratio = panel.getRenderWindow()->devicePixelRatio();
  QMouseEvent mouse(
    QEvent::MouseMove, QPointF(101.5, 73.5), QPointF(101.5, 73.5),
    Qt::NoButton, Qt::LeftButton, Qt::ShiftModifier);
  rviz_common::ViewportMouseEvent event(&panel, &mouse, 99, 71);
  EXPECT_EQ(event.x, qRound(101.5 * ratio));
  EXPECT_EQ(event.y, qRound(73.5 * ratio));
  EXPECT_EQ(event.last_x, qRound(99 * ratio));
  EXPECT_EQ(event.last_y, qRound(71 * ratio));
  EXPECT_TRUE(event.left());
  EXPECT_TRUE(event.shift());
}

TEST_F(ViewportMouseEventTest, wheel_coordinates_include_fractional_scaling)
{
  rviz_common::RenderPanel panel;
  const auto ratio = panel.getRenderWindow()->devicePixelRatio();
  QWheelEvent wheel(
    QPointF(101.5, 73.5), QPointF(101.5, 73.5), QPoint(), QPoint(0, 120),
    Qt::NoButton, Qt::ControlModifier, Qt::NoScrollPhase, false);
  rviz_common::ViewportMouseEvent event(&panel, &wheel, 99, 71);
  EXPECT_EQ(event.x, qRound(101.5 * ratio));
  EXPECT_EQ(event.y, qRound(73.5 * ratio));
  EXPECT_EQ(event.last_x, qRound(99 * ratio));
  EXPECT_EQ(event.last_y, qRound(71 * ratio));
  EXPECT_EQ(event.wheel_delta, 120);
  EXPECT_TRUE(event.control());
}

class RecordingTool : public rviz_common::Tool
{
public:
  void activate() override {}
  void deactivate() override {}

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    ++event_count;
    position = QPoint(event.x, event.y);
    previous_position = QPoint(event.last_x, event.last_y);
    return 0;
  }

  QPoint position;
  QPoint previous_position;
  int event_count = 0;
};

TEST_F(ViewportMouseEventTest, manager_delivers_device_coordinates_without_scaling_twice)
{
  rviz_common::RenderPanel panel;
  panel.resize(320, 240);
  panel.winId();
  panel.show();
  ASSERT_TRUE(QTest::qWaitForWindowExposed(panel.windowHandle()));
  panel.getRenderWindow()->renderNow();
  ASSERT_NE(panel.windowHandle(), nullptr);
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("scaling_test");
  rviz_common::VisualizationManager manager(
    &panel, node, nullptr, std::make_shared<rclcpp::Clock>());
  panel.initialize(&manager);
  manager.initialize();
  RecordingTool tool;
  manager.getToolManager()->setCurrentTool(&tool);

  QMouseEvent mouse(
    QEvent::MouseMove, QPointF(101, 73), QPointF(101, 73),
    Qt::NoButton, Qt::NoButton, Qt::NoModifier);
  rviz_common::ViewportMouseEvent event(&panel, &mouse, 99, 71);
  manager.handleMouseEvent(event);
  const auto ratio = panel.getRenderWindow()->devicePixelRatio();
  EXPECT_EQ(tool.position, QPoint(qRound(101 * ratio), qRound(73 * ratio)));
  EXPECT_EQ(tool.previous_position, QPoint(qRound(99 * ratio), qRound(71 * ratio)));
  manager.getToolManager()->setCurrentTool(nullptr);
}

TEST_F(ViewportMouseEventTest, panel_preserves_device_pixel_history_between_mouse_and_wheel_events)
{
  rviz_common::RenderPanel panel;
  panel.resize(320, 240);
  panel.winId();
  panel.show();
  ASSERT_TRUE(QTest::qWaitForWindowExposed(panel.windowHandle()));
  panel.getRenderWindow()->renderNow();
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("scaling_test");
  rviz_common::VisualizationManager manager(
    &panel, node, nullptr, std::make_shared<rclcpp::Clock>());
  panel.initialize(&manager);
  manager.initialize();
  RecordingTool tool;
  manager.getToolManager()->setCurrentTool(&tool);

  const auto ratio = panel.getRenderWindow()->devicePixelRatio();
  const std::pair<QEvent::Type, int> samples[] = {
    {QEvent::MouseButtonPress, 109}, {QEvent::MouseMove, 119},
    {QEvent::Wheel, 127}, {QEvent::Wheel, 137},
    {QEvent::MouseMove, 149}, {QEvent::MouseButtonRelease, 149}};
  QPoint previous_position;
  int event_count = 0;
  for (const auto & [type, y] : samples) {
    SCOPED_TRACE(event_count);
    // A vertical drag at native pixel x=151 has a fractional logical x at every
    // non-unit scale under test. Its horizontal delta must remain zero.
    const QPoint device_position(151, y);
    const QPointF logical_position(151.0 / ratio, y / ratio);
    if (type == QEvent::Wheel) {
      QWheelEvent wheel(
        logical_position, logical_position, QPoint(), QPoint(0, 120),
        Qt::LeftButton, Qt::NoModifier, Qt::NoScrollPhase, false);
      QApplication::sendEvent(panel.getRenderWindow(), &wheel);
    } else {
      const auto button = type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton;
      const auto buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton;
      QMouseEvent mouse(type, logical_position, logical_position, button, buttons, Qt::NoModifier);
      QApplication::sendEvent(panel.getRenderWindow(), &mouse);
    }
    EXPECT_EQ(tool.event_count, ++event_count);
    EXPECT_EQ(tool.position, device_position);
    EXPECT_EQ(tool.previous_position, previous_position);
    previous_position = device_position;
  }

  manager.getToolManager()->setCurrentTool(nullptr);
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
