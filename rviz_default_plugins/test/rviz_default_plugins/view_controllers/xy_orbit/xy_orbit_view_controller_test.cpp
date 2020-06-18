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
#include <OgreSceneManager.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "rviz_default_plugins/view_controllers/xy_orbit/xy_orbit_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/ortho/fixed_orientation_ortho_view_controller.hpp"

#include "../../displays/display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"
#include "../view_controller_test_fixture.hpp"

using namespace ::testing;  // NOLINT

class XYOrbitViewControllerTestFixture : public ViewControllerTestFixture
{
public:
  XYOrbitViewControllerTestFixture()
  {
    xy_orbit_ = std::make_shared<rviz_default_plugins::view_controllers::XYOrbitViewController>();
    xy_orbit_->initialize(context_.get());
    testing_environment_->createOgreRenderWindow()->addViewport(xy_orbit_->getCamera());
  }

  void dragMouse(
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    dragMouseInViewport(xy_orbit_, to_x, to_y, from_x, from_y, button, modifiers);
  }

  void setCameraToDefaultYawPitch()
  {
    auto yaw_property = xy_orbit_->childAt(7);
    auto pitch_property = xy_orbit_->childAt(8);
    yaw_property->setValue(0);  // set to zero to make result easier to check
    pitch_property->setValue(0.5f);  // set to 0.5, because setting it to 0 can cause problems
    xy_orbit_->update(0, 0);
  }

  std::shared_ptr<rviz_default_plugins::view_controllers::XYOrbitViewController> xy_orbit_;
};

TEST_F(XYOrbitViewControllerTestFixture, moving_the_mouse_to_the_left_rotates_left)
{
  setCameraToDefaultYawPitch();

  auto yaw_property = xy_orbit_->childAt(7);
  auto pitch_property = xy_orbit_->childAt(8);

  EXPECT_THAT(yaw_property->getNameStd(), StrEq("Yaw"));
  EXPECT_THAT(pitch_property->getNameStd(), StrEq("Pitch"));
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.5, 0.001f));

  dragMouse(0, 10, 10, 10, Qt::LeftButton);

  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0.05f, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.5f, 0.001f));
}

TEST_F(XYOrbitViewControllerTestFixture, moving_the_mouse_up_rotates_up)
{
  setCameraToDefaultYawPitch();

  auto yaw_property = xy_orbit_->childAt(7);
  auto pitch_property = xy_orbit_->childAt(8);

  EXPECT_THAT(yaw_property->getNameStd(), StrEq("Yaw"));
  EXPECT_THAT(pitch_property->getNameStd(), StrEq("Pitch"));
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.5, 0.001f));

  dragMouse(10, 20, 10, 10, Qt::LeftButton);

  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.55f, 0.001f));
}

TEST_F(XYOrbitViewControllerTestFixture, moving_the_wheel_zooms) {
  auto distance_property = xy_orbit_->childAt(4);
  EXPECT_THAT(distance_property->getNameStd(), StrEq("Distance"));
  EXPECT_THAT(distance_property->getValue().toFloat(), FloatNear(10, 0.001f));

  auto event = generateMouseWheelEvent(100);
  xy_orbit_->handleMouseEvent(event);

  EXPECT_THAT(distance_property->getValue().toFloat(), FloatNear(9, 0.001f));
}

TEST_F(XYOrbitViewControllerTestFixture, moving_the_mouse_with_right_button_zooms) {
  auto distance_property = xy_orbit_->childAt(4);
  EXPECT_THAT(distance_property->getNameStd(), StrEq("Distance"));
  EXPECT_THAT(distance_property->getValue().toFloat(), FloatNear(10, 0.001f));

  dragMouse(10, 20, 10, 10, Qt::RightButton);

  EXPECT_THAT(distance_property->getValue().toFloat(), FloatNear(11, 0.001f));
}

TEST_F(XYOrbitViewControllerTestFixture, moving_the_focal_point_does_not_move_z_direction) {
  setCameraToDefaultYawPitch();
  auto x_property = xy_orbit_->childAt(9)->childAt(0);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = xy_orbit_->childAt(9)->childAt(1);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  auto z_property = xy_orbit_->childAt(9)->childAt(2);
  EXPECT_THAT(z_property->getNameStd(), StrEq("Z"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));

  dragMouse(30, 30, 10, 10, Qt::LeftButton, Qt::ShiftModifier);

  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(-0.842f, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(-0.538f, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(XYOrbitViewControllerTestFixture, moving_the_focal_point_from_above_moves_point) {
  auto yaw_property = xy_orbit_->childAt(7);
  auto pitch_property = xy_orbit_->childAt(8);
  yaw_property->setValue(0);
  pitch_property->setValue(Ogre::Math::HALF_PI);  // set camera above view
  xy_orbit_->update(0, 0);

  auto x_property = xy_orbit_->childAt(9)->childAt(0);
  EXPECT_THAT(x_property->getNameStd(), StrEq("X"));
  auto y_property = xy_orbit_->childAt(9)->childAt(1);
  EXPECT_THAT(y_property->getNameStd(), StrEq("Y"));
  auto z_property = xy_orbit_->childAt(9)->childAt(2);
  EXPECT_THAT(z_property->getNameStd(), StrEq("Z"));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(0.0f, 0.001f));

  dragMouse(30, 30, 10, 10, Qt::LeftButton, Qt::ShiftModifier);

  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(-0.6f, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(-0.8f, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(
  XYOrbitViewControllerTestFixture,
  mimic_does_not_change_view_when_changing_back_and_forth_between_orbit_view_controllers)
{
  auto orbit_view =
    std::make_shared<rviz_default_plugins::view_controllers::OrbitViewController>();
  orbit_view->setClassId("rviz_default_plugins/Orbit");
  orbit_view->initialize(context_.get());
  auto old_yaw_property = orbit_view->childAt(7);
  auto old_pitch_property = orbit_view->childAt(8);
  old_yaw_property->setValue(1);
  old_pitch_property->setValue(1);
  orbit_view->move(10, 12, 3);
  orbit_view->update(0, 0);

  auto new_orbit_view =
    std::make_shared<rviz_default_plugins::view_controllers::OrbitViewController>();
  new_orbit_view->setClassId("rviz_default_plugins/Orbit");
  new_orbit_view->initialize(context_.get());

  xy_orbit_->mimic(orbit_view.get());
  xy_orbit_->update(0, 0);
  new_orbit_view->mimic(xy_orbit_.get());
  new_orbit_view->update(0, 0);

  auto new_yaw_value = new_orbit_view->childAt(7)->getValue().toFloat();
  auto new_pitch_value = new_orbit_view->childAt(8)->getValue().toFloat();
  EXPECT_THAT(new_yaw_value, FloatNear(old_yaw_property->getValue().toFloat(), 0.001f));
  EXPECT_THAT(new_pitch_value, FloatNear(old_pitch_property->getValue().toFloat(), 0.001f));
  // We don't check the focal point here, because it can change as the distance changes when
  // moving away from frame origin
}

TEST_F(
  XYOrbitViewControllerTestFixture,
  mimic_keeps_focal_point_and_view_from_top_down_ortho_view_controller)
{
  auto ortho_view =
    std::make_shared<rviz_default_plugins::view_controllers::FixedOrientationOrthoViewController>();
  ortho_view->setClassId("rviz_default_plugins/TopDownOrtho");
  ortho_view->initialize(context_.get());
  ortho_view->move(10, 12);
  // ortho_view->update(0, 0);
  auto old_x_value = ortho_view->childAt(6)->getValue().toFloat();
  auto old_y_value = ortho_view->childAt(7)->getValue().toFloat();

  xy_orbit_->mimic(ortho_view.get());
  xy_orbit_->update(0, 0);

  auto yaw_property = xy_orbit_->childAt(7);
  auto pitch_property = xy_orbit_->childAt(8);
  auto x_property = xy_orbit_->childAt(9)->childAt(0);
  auto y_property = xy_orbit_->childAt(9)->childAt(1);
  auto z_property = xy_orbit_->childAt(9)->childAt(2);
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(-1.5708f, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(1.5698f, 0.001f));
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(old_x_value, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(old_y_value, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(
  XYOrbitViewControllerTestFixture,
  mimic_does_not_move_camera_when_given_same_class_controller)
{
  auto old_orbit_view =
    std::make_shared<rviz_default_plugins::view_controllers::XYOrbitViewController>();
  old_orbit_view->setClassId("rviz_default_plugins/Orbit");
  old_orbit_view->initialize(context_.get());
  auto old_yaw_property = old_orbit_view->childAt(7);
  auto old_pitch_property = old_orbit_view->childAt(8);
  old_yaw_property->setValue(0);
  old_pitch_property->setValue(0.5f);
  old_orbit_view->update(0, 0);
  auto old_x_value = old_orbit_view->childAt(9)->childAt(0)->getValue().toFloat();
  auto old_y_value = old_orbit_view->childAt(9)->childAt(1)->getValue().toFloat();
  auto old_z_value = old_orbit_view->childAt(9)->childAt(2)->getValue().toFloat();

  xy_orbit_->mimic(old_orbit_view.get());
  xy_orbit_->update(0, 0);

  auto x_property = xy_orbit_->childAt(9)->childAt(0);
  auto y_property = xy_orbit_->childAt(9)->childAt(1);
  auto z_property = xy_orbit_->childAt(9)->childAt(2);
  auto yaw_property = xy_orbit_->childAt(7);
  auto pitch_property = xy_orbit_->childAt(8);
  EXPECT_THAT(x_property->getValue().toFloat(), FloatNear(old_x_value, 0.001f));
  EXPECT_THAT(y_property->getValue().toFloat(), FloatNear(old_y_value, 0.001f));
  EXPECT_THAT(z_property->getValue().toFloat(), FloatNear(old_z_value, 0.001f));
  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0.5f, 0.001f));
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
