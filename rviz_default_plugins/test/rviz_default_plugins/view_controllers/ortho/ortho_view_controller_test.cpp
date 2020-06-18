/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <memory>

#include <QApplication>  // NOLINT cpplint cannot handle include order
#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include <OgreCamera.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"

#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"
#include "../view_controller_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"

using namespace ::testing;  // NOLINT

class OrthoViewControllerTestFixture : public ViewControllerTestFixture
{
public:
  OrthoViewControllerTestFixture()
  {
    ortho_view_ = std::make_shared<
      rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController>();
    ortho_view_->initialize(context_.get());
    testing_environment_->createOgreRenderWindow()->addViewport(ortho_view_->getCamera());
  }

  void dragMouse(
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    dragMouseInViewport(ortho_view_, to_x, to_y, from_x, from_y, button, modifiers);
  }

public:
  std::shared_ptr<rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController>
  ortho_view_;
};

TEST_F(OrthoViewControllerTestFixture, moving_the_mouse_with_left_click_rotates_the_view) {
  auto angle_property = ortho_view_->childAt(5);
  EXPECT_THAT(angle_property->getNameStd(), StrEq("Angle"));
  EXPECT_THAT(angle_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));

  dragMouse(20, 10, 10, 10, Qt::LeftButton);

  EXPECT_THAT(angle_property->getValue().toFloat(), FloatNear(0.05f, 0.001f));
}

TEST_F(OrthoViewControllerTestFixture, moving_the_mouse_with_right_click_zooms) {
  auto scale_property = ortho_view_->childAt(4);
  EXPECT_THAT(scale_property->getNameStd(), StrEq("Scale"));
  EXPECT_THAT(scale_property->getValue().toFloat(), FloatNear(10, 0.001f));

  dragMouse(10, 30, 10, 10, Qt::RightButton);

  EXPECT_THAT(scale_property->getValue().toFloat(), FloatNear(8, 0.001f));
}

TEST_F(OrthoViewControllerTestFixture, moving_the_mouse_with_shift_and_left_moves_position) {
  auto x_property = ortho_view_->childAt(6);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = ortho_view_->childAt(7);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0, 0.001f));

  dragMouse(30, 30, 10, 10, Qt::LeftButton, Qt::ShiftModifier);

  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(-2, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(2, 0.001f));
}

TEST_F(OrthoViewControllerTestFixture, moving_the_middle_button_move_position) {
  auto x_property = ortho_view_->childAt(6);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = ortho_view_->childAt(7);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0, 0.001f));

  dragMouse(30, 30, 10, 10, Qt::MiddleButton);

  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(-2, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(2, 0.001f));
}

TEST_F(OrthoViewControllerTestFixture, moving_the_wheel_zooms) {
  auto scale_property = ortho_view_->childAt(4);
  EXPECT_THAT(scale_property->getNameStd(), StrEq("Scale"));
  EXPECT_THAT(scale_property->getValue().toFloat(), FloatNear(10, 0.001f));

  auto event = generateMouseWheelEvent(100);
  ortho_view_->handleMouseEvent(event);

  EXPECT_THAT(scale_property->getValue().toFloat(), FloatNear(11, 0.001f));
}

TEST_F(OrthoViewControllerTestFixture, update_sets_camera_to_position_indicated_by_properties) {
  dragMouse(30, 30, 10, 10, Qt::MiddleButton);

  ortho_view_->update(0, 0);

  auto camera_parent = ortho_view_->getCamera()->getParentSceneNode();
  EXPECT_THAT(camera_parent->getPosition(), Vector3Eq(Ogre::Vector3(-2, 2, 500)));
}

TEST_F(
  OrthoViewControllerTestFixture,
  mimic_sets_camera_above_focal_point_when_given_an_orbit_view_controller)
{
  auto orbit_view = std::make_shared<rviz_default_plugins::view_controllers::OrbitViewController>();
  orbit_view->initialize(context_.get());
  orbit_view->move(10, 12, 1);

  ortho_view_->mimic(orbit_view.get());
  ortho_view_->update(0, 0);

  auto x_property = ortho_view_->childAt(6);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = ortho_view_->childAt(7);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(10, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(12, 0.001f));
  auto camera_parent = ortho_view_->getCamera()->getParentSceneNode();
  EXPECT_THAT(camera_parent->getPosition(), Vector3Eq(Ogre::Vector3(10, 12, 500)));
}

TEST_F(
  OrthoViewControllerTestFixture,
  mimic_does_not_move_camera_when_given_same_class_controller)
{
  auto old_ortho_view =
    std::make_shared<rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController>();
  old_ortho_view->initialize(context_.get());
  old_ortho_view->move(10, 10);

  ortho_view_->mimic(old_ortho_view.get());

  auto x_property = ortho_view_->childAt(6);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = ortho_view_->childAt(7);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(
  OrthoViewControllerTestFixture,
  mimic_sets_camera_at_view_controller_camera_position_when_given_any_view_controller)
{
  auto controller = std::make_shared<MockViewController>();
  controller->initialize(context_.get());

  ortho_view_->mimic(controller.get());

  auto x_property = ortho_view_->childAt(6);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = ortho_view_->childAt(7);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
