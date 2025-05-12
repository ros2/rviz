// Copyright (c) 2018, Bosch Software Innovations GmbH.
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

#include <QApplication>  // NOLINT cpplint cannot handle include order
#include <QKeyEvent>  // NOLINT cpplint cannot handle include order

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreVector.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/bool_property.hpp"

#include "rviz_default_plugins/view_controllers/frame/frame_view_controller.hpp"
#include "rviz_default_plugins/view_controllers/orbit/orbit_view_controller.hpp"

#include "../../displays/display_test_fixture.hpp"
#include "../../scene_graph_introspection.hpp"
#include "../view_controller_test_fixture.hpp"

using namespace ::testing;  // NOLINT

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

class FrameViewControllerTestFixture : public ViewControllerTestFixture
{
public:
  FrameViewControllerTestFixture()
  {
    frame_ = std::make_shared<rviz_default_plugins::view_controllers::FrameViewController>();
    frame_->initialize(context_.get());
    testing_environment_->createOgreRenderWindow()->addViewport(frame_->getCamera());
  }

  void dragMouse(
    int to_x, int to_y, int from_x, int from_y,
    Qt::MouseButton button, Qt::KeyboardModifiers modifiers = Qt::NoModifier)
  {
    dragMouseInViewport(frame_, to_x, to_y, from_x, from_y, button, modifiers);
  }

  int getAxisPropertyOptionInt()
  {
    auto axis_property = frame_->childAt(7);
    EXPECT_THAT(axis_property->getNameStd(), StrEq("Point towards"));
    auto enum_property = dynamic_cast<rviz_common::properties::EnumProperty *>(axis_property);
    return enum_property->getOptionInt();
  }


  void setAxisPropertyOption(int option)
  {
    auto axis_property = frame_->childAt(7);
    EXPECT_THAT(axis_property->getNameStd(), StrEq("Point towards"));
    auto enum_property = dynamic_cast<rviz_common::properties::EnumProperty *>(axis_property);
    ASSERT_THAT(enum_property, NotNull()) << "Property is not an EnumProperty";

    auto axisString = [](int i) {
        return QString("%1%2 axis").arg(QChar(i % 2 ? '+' : '-')).arg(QChar('x' + (i - 1) / 2));
      };
    // Use a direct approach: first store the current string
    QString currentString = enum_property->getString();
    // Set the property using the string value
    enum_property->setString(axisString(option));

    // Verify the option was set correctly
    ASSERT_THAT(enum_property->getOptionInt(), Eq(option))
      << "Failed to set axis property to option: " << option
      << ", resulting optionInt: " << enum_property->getOptionInt();
  }

  bool getLockCameraProperty()
  {
    auto lock_property = frame_->childAt(8);
    EXPECT_THAT(lock_property->getNameStd(), StrEq("Lock Camera"));
    return static_cast<rviz_common::properties::BoolProperty *>(lock_property)->getBool();
  }

  void setLockCameraProperty(bool lock)
  {
    auto lock_property = frame_->childAt(8);
    EXPECT_THAT(lock_property->getNameStd(), StrEq("Lock Camera"));
    static_cast<rviz_common::properties::BoolProperty *>(lock_property)->setBool(lock);
  }

  void checkCameraLooksAlong(const Ogre::Vector3 & expected_direction, float tolerance = 0.001f)
  {
    Ogre::Vector3 camera_direction =
      (frame_->getCamera()->getParentSceneNode()->getOrientation() *
      ROBOT_TO_CAMERA_ROTATION.Inverse()) *
      Ogre::Vector3::UNIT_X;

    EXPECT_THAT(camera_direction.x, FloatNear(expected_direction.x, tolerance));
    EXPECT_THAT(camera_direction.y, FloatNear(expected_direction.y, tolerance));
    EXPECT_THAT(camera_direction.z, FloatNear(expected_direction.z, tolerance));
  }

  void setCameraToDefaultPosition()
  {
    auto yaw_property = frame_->childAt(4);
    auto pitch_property = frame_->childAt(5);
    ASSERT_THAT(yaw_property->getNameStd(), StrEq("Yaw"));
    ASSERT_THAT(pitch_property->getNameStd(), StrEq("Pitch"));
    yaw_property->setValue(0);  // set to zero to make result easier to check
    pitch_property->setValue(0);  // set to zeor to make result easier to check
    auto position_property = frame_->childAt(6);
    ASSERT_THAT(position_property->getNameStd(), StrEq("Position"));
    position_property->childAt(0)->setValue(0);
    position_property->childAt(1)->setValue(0);
    position_property->childAt(2)->setValue(0);
    frame_->update(0, 0);  // Camera now looks in x-direction, located at the origin

    EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0, 0.001f));
    auto x_position = position_property->childAt(0);
    auto y_position = position_property->childAt(1);
    auto z_position = position_property->childAt(2);
    EXPECT_THAT(x_position->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(y_position->getValue().toFloat(), FloatNear(0, 0.001f));
    EXPECT_THAT(z_position->getValue().toFloat(), FloatNear(0, 0.001f));
  }

  std::shared_ptr<rviz_default_plugins::view_controllers::FrameViewController> frame_;
};

TEST_F(FrameViewControllerTestFixture, locking_camera_prevents_mouse_interaction) {
  setCameraToDefaultPosition();
  setLockCameraProperty(true);
  dragMouse(0, 10, 10, 10, Qt::LeftButton);

  auto yaw_property = frame_->childAt(4);
  auto pitch_property = frame_->childAt(5);

  EXPECT_THAT(yaw_property->getValue().toFloat(), FloatNear(0, 0.001f));
  EXPECT_THAT(pitch_property->getValue().toFloat(), FloatNear(0, 0.001f));
}

TEST_F(FrameViewControllerTestFixture, reset_points_camera_along_selected_axis) {
  // Change camera orientation
  auto yaw_property = frame_->childAt(4);
  auto pitch_property = frame_->childAt(5);
  yaw_property->setValue(1.5);
  pitch_property->setValue(0.5);
  frame_->update(0, 0);

  // Reset should align with -z axis (option 6)
  frame_->reset();

  checkCameraLooksAlong(Ogre::Vector3(0, 0, -1));
}

TEST_F(FrameViewControllerTestFixture, default_axis_is_negative_z) {
  // Test that camera initially points along -z axis (option 6)
  EXPECT_THAT(getAxisPropertyOptionInt(), Eq(6));

  checkCameraLooksAlong(Ogre::Vector3(0, 0, -1));
}

TEST_F(FrameViewControllerTestFixture, setting_axis_to_positive_x_points_camera_along_x_axis) {
  // +x axis = option 1
  setAxisPropertyOption(1);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(1, 0, 0));
}

TEST_F(
  FrameViewControllerTestFixture,
  setting_axis_to_negative_x_points_camera_along_negative_x_axis) {
  // -x axis = option 2
  setAxisPropertyOption(2);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(-1, 0, 0));
}

TEST_F(FrameViewControllerTestFixture, setting_axis_to_positive_y_points_camera_along_y_axis) {
  // +y axis = option 3
  setAxisPropertyOption(3);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(0, 1, 0));
}

TEST_F(
  FrameViewControllerTestFixture,
  setting_axis_to_negative_y_points_camera_along_negative_y_axis) {
  // -y axis = option 4
  setAxisPropertyOption(4);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(0, -1, 0));
}

TEST_F(FrameViewControllerTestFixture, setting_axis_to_positive_z_points_camera_along_z_axis) {
  // +z axis = option 5
  setAxisPropertyOption(5);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(0, 0, 1));
}

TEST_F(
  FrameViewControllerTestFixture,
  setting_axis_to_negative_z_points_camera_along_negative_z_axis) {
  // -z axis = option 6
  setAxisPropertyOption(6);
  frame_->update(0, 0);

  checkCameraLooksAlong(Ogre::Vector3(0, 0, -1));
}

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
