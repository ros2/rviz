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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>

#include <QApplication>  // NOLINT
#include <QMouseEvent>  // NOLINT

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_control.hpp"
#include "rviz_rendering/render_window.hpp"

#include "../display_test_fixture.hpp"
#include "../../mock_view_picker.hpp"

using rviz_default_plugins::displays::InteractiveMarker;
using rviz_default_plugins::displays::InteractiveMarkerControl;
using rviz_rendering::RenderWindowOgreAdapter;
using ControlMessage = visualization_msgs::msg::InteractiveMarkerControl;

class DragProbeControl : public InteractiveMarkerControl
{
public:
  using InteractiveMarkerControl::InteractiveMarkerControl;

  bool mouseDragging() const {return mouse_dragging_;}
  Ogre::Viewport * dragViewport() const {return drag_viewport_;}
};

class InteractiveMarkerControlLifetimeTest : public DisplayTestFixture,
  public testing::WithParamInterface<bool>
{
protected:
  void SetUp() override
  {
    view_picker_ = std::make_shared<MockViewPicker>();
    ON_CALL(*context_, getViewPicker()).WillByDefault(testing::Return(view_picker_));
    mockValidTransform(Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);

    panel_ = std::make_unique<rviz_common::RenderPanel>();
    panel_->initialize(context_.get(), true);
    panel_->getRenderWindow()->resize(64, 64);
    panel_->getRenderWindow()->initialize();

    auto * reference_node = scene_manager_->getRootSceneNode()->createChildSceneNode();
    marker_ = std::make_unique<InteractiveMarker>(reference_node, context_.get());
    visualization_msgs::msg::InteractiveMarker marker_message;
    marker_message.header.frame_id = fixed_frame;
    marker_message.name = "lifetime_test";
    marker_message.scale = 1;
    marker_message.pose.orientation.w = 1;
    marker_message.controls.emplace_back();
    marker_message.controls.back().name = "idle";
    marker_message.controls.back().orientation.w = 1;
    ASSERT_TRUE(marker_->processMessage(marker_message));

    control_ = std::make_shared<DragProbeControl>(context_.get(), reference_node, marker_.get());
    ControlMessage message;
    message.name = "drag";
    message.orientation.w = 1;
    message.orientation_mode = ControlMessage::VIEW_FACING;
    message.interaction_mode = GetParam() ? ControlMessage::MOVE_3D : ControlMessage::MOVE_PLANE;
    control_->processMessage(message);
  }

  void sendButton(QEvent::Type type)
  {
    QMouseEvent mouse_event(
      type, QPointF(32, 32), QPointF(32, 32), Qt::LeftButton,
      type == QEvent::MouseButtonPress ? Qt::LeftButton : Qt::NoButton, Qt::NoModifier);
    rviz_common::ViewportMouseEvent event(panel_.get(), &mouse_event, 32, 32);
    if (GetParam()) {
      control_->handle3DCursorEvent(event, Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
    } else {
      control_->handleMouseEvent(event);
    }
  }

  void beginDrag()
  {
    sendButton(QEvent::MouseButtonPress);
    ASSERT_EQ(
      control_->dragViewport(),
      RenderWindowOgreAdapter::getOgreViewport(panel_->getRenderWindow()));
    ASSERT_NE(control_->dragViewport(), nullptr);
  }

  void expectCanceledDrag()
  {
    ASSERT_EQ(control_->dragViewport(), nullptr);
    ASSERT_FALSE(control_->mouseDragging());
    control_->update();
    control_->interactiveMarkerPoseChanged(Ogre::Vector3::ZERO, Ogre::Quaternion::IDENTITY);
    marker_->requestPoseUpdate(Ogre::Vector3(2, 0, 0), Ogre::Quaternion::IDENTITY);
    EXPECT_EQ(marker_->getPosition(), Ogre::Vector3(2, 0, 0));
  }

  std::shared_ptr<MockViewPicker> view_picker_;
  std::unique_ptr<rviz_common::RenderPanel> panel_;
  std::unique_ptr<InteractiveMarker> marker_;
  std::shared_ptr<DragProbeControl> control_;
};

TEST_P(InteractiveMarkerControlLifetimeTest, destroying_surface_cancels_drag_and_allows_a_new_one)
{
  beginDrag();
  auto * window = panel_->getRenderWindow();

  window->destroy();

  expectCanceledDrag();
  window->create();
  window->initialize();
  beginDrag();
  sendButton(QEvent::MouseButtonRelease);
  EXPECT_EQ(control_->dragViewport(), nullptr);
}

TEST_P(InteractiveMarkerControlLifetimeTest, deleting_panel_cancels_drag)
{
  beginDrag();

  panel_.reset();

  expectCanceledDrag();
}

TEST_P(InteractiveMarkerControlLifetimeTest, destroying_another_window_preserves_drag)
{
  beginDrag();
  rviz_common::RenderPanel other_panel;
  other_panel.initialize(context_.get(), true);
  other_panel.getRenderWindow()->resize(64, 64);
  other_panel.getRenderWindow()->initialize();

  other_panel.getRenderWindow()->destroy();

  EXPECT_EQ(
    control_->dragViewport(), RenderWindowOgreAdapter::getOgreViewport(panel_->getRenderWindow()));
  marker_->requestPoseUpdate(Ogre::Vector3(2, 0, 0), Ogre::Quaternion::IDENTITY);
  EXPECT_EQ(marker_->getPosition(), Ogre::Vector3::ZERO);
  sendButton(QEvent::MouseButtonRelease);
  EXPECT_EQ(marker_->getPosition(), Ogre::Vector3(2, 0, 0));
}

TEST_P(InteractiveMarkerControlLifetimeTest, deleting_control_before_surface_is_safe)
{
  beginDrag();

  control_.reset();
  panel_->getRenderWindow()->destroy();
}

TEST_P(InteractiveMarkerControlLifetimeTest, input_without_a_surface_is_ignored)
{
  panel_->getRenderWindow()->destroy();

  sendButton(QEvent::MouseButtonPress);

  EXPECT_EQ(control_->dragViewport(), nullptr);
  EXPECT_FALSE(control_->mouseDragging());
  marker_->requestPoseUpdate(Ogre::Vector3(2, 0, 0), Ogre::Quaternion::IDENTITY);
  EXPECT_EQ(marker_->getPosition(), Ogre::Vector3(2, 0, 0));
}

INSTANTIATE_TEST_SUITE_P(
  MouseAndCursor, InteractiveMarkerControlLifetimeTest, testing::Values(false, true));

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
