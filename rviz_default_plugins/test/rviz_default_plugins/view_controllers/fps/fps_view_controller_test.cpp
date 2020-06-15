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

#include <gmock/gmock.h>

#include <memory>

#include <QApplication>  // NOLINT cpplint cannot handle include order
#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include <OgreCamera.h>
#include <OgreRoot.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"

#include "../../displays/display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"
#include "../view_controller_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class FPSViewControllerTestFixture : public ViewControllerTestFixture
{
public:
  FPSViewControllerTestFixture()
  {
    fps_ = std::make_shared<rviz_default_plugins::view_controllers::FPSViewController>();
    fps_->initialize(context_.get());
    testing_environment_->createOgreRenderWindow()->addViewport(fps_->getCamera());
  }

  void dragMouse(
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    dragMouseInViewport(fps_, to_x, to_y, from_x, from_y, button, modifiers);
  }

  void setCameraToDefaultPosition()
  {
    auto yaw_property = fps_->childAt(4);
    auto pitch_property = fps_->childAt(5);
    ASSERT_THAT(yaw_property->getNameStd(), StrEq("Yaw"));
    ASSERT_THAT(pitch_property->getNameStd(), StrEq("Pitch"));
    yaw_property->setValue(0);  // set to zero to make result easier to check
    pitch_property->setValue(0);  // set to zeor to make result easier to check
    auto position_property = fps_->childAt(6);
    ASSERT_THAT(position_property->getNameStd(), StrEq("Position"));
    position_property->childAt(0)->setValue(0);
    position_property->childAt(1)->setValue(0);
    position_property->childAt(2)->setValue(0);
    fps_->update(0, 0);  // Camera now looks in x-direction, located at the origin

    EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0, 0.001f));
    auto x_position = position_property->childAt(0);
    auto y_position = position_property->childAt(1);
    auto z_position = position_property->childAt(2);
    EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(0, 0.001f));
  }

  std::shared_ptr<rviz_default_plugins::view_controllers::FPSViewController> fps_;
};

TEST_F(FPSViewControllerTestFixture, moving_the_mouse_to_the_left_rotates_left)
{
  setCameraToDefaultPosition();

  dragMouse(0, 10, 10, 10, Qt::LeftButton);

  auto yaw_property = fps_->childAt(4);
  auto pitch_property = fps_->childAt(5);
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0.05f, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(FPSViewControllerTestFixture, moving_the_mouse_up_rotates_up)
{
  setCameraToDefaultPosition();

  dragMouse(10, 20, 10, 10, Qt::LeftButton);

  auto yaw_property = fps_->childAt(4);
  auto pitch_property = fps_->childAt(5);
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.05f, 0.001f));
}

TEST_F(FPSViewControllerTestFixture, moving_the_wheel_moves_camera_in_looking_direction) {
  setCameraToDefaultPosition();
  auto event = generateMouseWheelEvent(100);

  fps_->handleMouseEvent(event);

  auto position_property = fps_->childAt(6);
  auto x_position = position_property->childAt(0);
  auto y_position = position_property->childAt(1);
  auto z_position = position_property->childAt(2);
  EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(1, 0.001f));
  EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(
  FPSViewControllerTestFixture,
  moving_the_mouse_with_right_button_moves_camera_in_looking_direction) {
  setCameraToDefaultPosition();

  dragMouse(10, 0, 10, 10, Qt::RightButton);

  auto position_property = fps_->childAt(6);
  auto x_position = position_property->childAt(0);
  auto y_position = position_property->childAt(1);
  auto z_position = position_property->childAt(2);
  EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(1, 0.001f));
  EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(FPSViewControllerTestFixture, moving_the_mouse_with_shift_moves_camera_in_xy_plane) {
  setCameraToDefaultPosition();

  dragMouse(10, 10, 0, 0, Qt::LeftButton, Qt::ShiftModifier);

  auto position_property = fps_->childAt(6);
  auto x_position = position_property->childAt(0);
  auto y_position = position_property->childAt(1);
  auto z_position = position_property->childAt(2);
  EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(-0.1f, 0.001f));
  EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(-0.1f, 0.001f));
}

TEST_F(FPSViewControllerTestFixture, reset_sets_to_some_point_looking_at_origin) {
  setCameraToDefaultPosition();

  fps_->reset();

  auto yaw_property = fps_->childAt(4);
  auto pitch_property = fps_->childAt(5);
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(3.92699f, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.955317f, 0.001f));
  auto position_property = fps_->childAt(6);
  auto x_position = position_property->childAt(0);
  auto y_position = position_property->childAt(1);
  auto z_position = position_property->childAt(2);
  EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(5, 0.001f));
  EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(5, 0.001f));
  EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(10, 0.001f));
}

TEST_F(FPSViewControllerTestFixture, mimic_does_not_change_view_when_given_any_view_controller) {
  auto orbit_view =
    std::make_shared<rviz_default_plugins::view_controllers::OrbitViewController>();
  orbit_view->setClassId("rviz_default_plugins/XYOrbit");
  orbit_view->initialize(context_.get());
  auto old_yaw_property = orbit_view->childAt(7);
  auto old_pitch_property = orbit_view->childAt(8);
  old_yaw_property->setValue(1);
  old_pitch_property->setValue(2);
  orbit_view->move(10, 12, 0);
  orbit_view->update(0, 0);

  fps_->mimic(orbit_view.get());
  fps_->update(0, 0);

  // Yaw and Pitch cannot be equal since the orbit view's yaw and pitch are relative to its focal
  // point, while the fps yaw and pitch are absolute. However, the orientation of the camera
  // scene node should be equivalent.
  auto fps_camera_orientation = fps_->getCamera()->getParentSceneNode()->getOrientation();
  auto orbit_camera_orientation = orbit_view->getCamera()->getParentSceneNode()->getOrientation();
  EXPECT_THAT(fps_camera_orientation, QuaternionEq(orbit_camera_orientation));
  auto position_property = fps_->childAt(6);
  auto x_position = position_property->childAt(0);
  auto y_position = position_property->childAt(1);
  auto z_position = position_property->childAt(2);
  auto orbit_camera_position = orbit_view->getCamera()->getParentSceneNode()->getPosition();
  EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(orbit_camera_position.x, 0.001f));
  EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(orbit_camera_position.y, 0.001f));
  EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(orbit_camera_position.z, 0.001f));
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
