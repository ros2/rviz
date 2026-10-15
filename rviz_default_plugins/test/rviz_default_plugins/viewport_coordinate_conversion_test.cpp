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

#include <QApplication>  // NOLINT
#include <QCursor>  // NOLINT
#include <QKeyEvent>  // NOLINT
#include <QMouseEvent>  // NOLINT
#include <QTest>  // NOLINT
#include <OgreSceneNode.h>  // NOLINT

#include "rviz_common/render_panel.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_control.hpp"
#include "rviz_rendering/render_window.hpp"

#include "displays/display_test_fixture.hpp"
#include "mock_view_picker.hpp"
#include "view_controllers/view_controller_test_fixture.hpp"

using rviz_rendering::RenderWindowOgreAdapter;
using rviz_default_plugins::displays::InteractiveMarker;
using rviz_default_plugins::displays::InteractiveMarkerControl;

class ViewportCoordinateConversionTest : public DisplayTestFixture
{
protected:
  void SetUp() override
  {
    original_cursor_ = QCursor::pos();
    mockValidTransform(Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
    panel_ = std::make_unique<rviz_common::RenderPanel>();
    panel_->resize(320, 240);
    panel_->winId();
    panel_->show();
    ASSERT_TRUE(QTest::qWaitForWindowExposed(panel_->windowHandle()));
    panel_->getRenderWindow()->renderNow();
    RenderWindowOgreAdapter::setOgreCameraPosition(panel_->getRenderWindow(), Ogre::Vector3::ZERO);
    RenderWindowOgreAdapter::setOgreCameraOrientation(
      panel_->getRenderWindow(), Ogre::Quaternion::IDENTITY);
  }

  void TearDown() override
  {
    // Discard the posted context-menu event without entering QMenu's modal loop.
    panel_.reset();
    QCursor::setPos(original_cursor_);
  }

  visualization_msgs::msg::InteractiveMarker markerMessage()
  {
    visualization_msgs::msg::InteractiveMarker message;
    message.header.frame_id = fixed_frame;
    message.name = "scaling_test";
    message.scale = 1;
    message.pose.orientation.w = 1;
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.name = "menu";
    control.orientation.w = 1;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
    message.controls.push_back(control);
    visualization_msgs::msg::MenuEntry entry;
    entry.id = 1;
    entry.title = "Test menu";
    message.menu_entries.push_back(entry);
    return message;
  }

  Ogre::Vector3 menuCursorPoint()
  {
    // With the camera at the origin looking down -Z, this projects exactly to
    // the center, avoiding floating-point error in a ray/projection round trip.
    return Ogre::Vector3(0, 0, -10);
  }

  std::unique_ptr<rviz_common::RenderPanel> panel_;
  QPoint original_cursor_;
};

TEST_F(ViewportCoordinateConversionTest, focus_shortcut_passes_device_coordinates_to_picker)
{
  auto picker = std::make_shared<MockViewPicker>();
  EXPECT_CALL(*context_, getViewPicker()).WillRepeatedly(testing::Return(picker));
  MockViewController controller;
  controller.initialize(context_.get());
  const QPoint logical_position(101, 73);
  const auto ratio = panel_->getRenderWindow()->devicePixelRatio();
  const Ogre::Vector3 object_position(3, 4, 5);
  picker->registerObject(Visible3DObject(
      qRound(101 * ratio), qRound(73 * ratio), object_position));
  QCursor::setPos(panel_->mapToGlobal(logical_position));
  ASSERT_EQ(panel_->mapFromGlobal(QCursor::pos()), logical_position);
  EXPECT_CALL(controller, lookAt(testing::_)).WillOnce(
    [&object_position](const Ogre::Vector3 & point) {EXPECT_EQ(point, object_position);});

  QKeyEvent key(QEvent::KeyPress, Qt::Key_F, Qt::NoModifier);
  controller.handleKeyEvent(&key, panel_.get());
}

TEST_F(ViewportCoordinateConversionTest, marker_menu_warps_cursor_to_logical_projection)
{
  InteractiveMarker marker(scene_manager_->getRootSceneNode(), context_.get());
  ASSERT_TRUE(marker.processMessage(markerMessage()));
  QMouseEvent mouse(
    QEvent::MouseButtonRelease, QPointF(), QPointF(),
    Qt::RightButton, Qt::NoButton, Qt::NoModifier);
  rviz_common::ViewportMouseEvent event(panel_.get(), &mouse, QPoint());
  QCursor::setPos(panel_->mapToGlobal(QPoint(20, 20)));

  EXPECT_TRUE(marker.handle3DCursorEvent(
      event, menuCursorPoint(), Ogre::Quaternion::IDENTITY, "menu"));
  EXPECT_EQ(panel_->mapFromGlobal(QCursor::pos()), QPoint(160, 120));
  EXPECT_TRUE(panel_->contextMenuVisible());
}

TEST_F(ViewportCoordinateConversionTest, control_menu_warps_cursor_to_logical_projection)
{
  InteractiveMarker marker(scene_manager_->getRootSceneNode(), context_.get());
  const auto message = markerMessage();
  ASSERT_TRUE(marker.processMessage(message));
  auto control = std::make_shared<InteractiveMarkerControl>(
    context_.get(), scene_manager_->getRootSceneNode(), &marker);
  control->processMessage(message.controls.front());
  QMouseEvent mouse(
    QEvent::MouseButtonRelease, QPointF(), QPointF(),
    Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
  rviz_common::ViewportMouseEvent event(panel_.get(), &mouse, QPoint());
  QCursor::setPos(panel_->mapToGlobal(QPoint(20, 20)));

  control->handle3DCursorEvent(event, menuCursorPoint(), Ogre::Quaternion::IDENTITY);
  EXPECT_EQ(panel_->mapFromGlobal(QCursor::pos()), QPoint(160, 120));
  EXPECT_TRUE(panel_->contextMenuVisible());
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
