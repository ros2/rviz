// Copyright (c) 2012, Willow Garage, Inc.
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


#include "rviz_default_plugins/view_controllers/frame/frame_view_controller.hpp"

#include <cmath>

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreVector.h>
#include <OgreViewport.h>

#include "rviz_rendering/geometry.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/uniform_string_stream.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{
static const QString ANY_AXIS("arbitrary");

// helper function to create axis strings from option ID
inline QString fmtAxis(int i)
{
  return QString("%1%2 axis").arg(QChar(i % 2 ? '+' : '-')).arg(QChar('x' + (i - 1) / 2));
}

static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

static const Ogre::Vector3 DEFAULT_FRAMEVIEW_POSITION = Ogre::Vector3(-5, 0, 0);

FrameViewController::FrameViewController()
{
  axis_property_ = new rviz_common::properties::EnumProperty(
    "Point towards", fmtAxis(1),
    "Point the camera along the given axis of the frame.", this,
    SLOT(changedAxis()));
  axis_property_->addOption(ANY_AXIS, -1);

  // x,y,z axes get integers from 1..6: +x, -x, +y, -y, +z, -z
  for (int i = 1; i <= 6; ++i) {
    axis_property_->addOption(fmtAxis(i), i);
  }
  previous_axis_ = axis_property_->getOptionInt();

  locked_property_ = new rviz_common::properties::BoolProperty(
    "Lock Camera", false,
    "Lock camera in its current pose relative to the frame", this);
}

void FrameViewController::onInitialize()
{
  FPSViewController::onInitialize();
  invert_z_->show();
  changedAxis();
}

int FrameViewController::actualCameraAxisOption(double precision) const
{
  // compare current camera direction with unit axes, select the axis that is most aligned with camera
  Ogre::Vector3 actual =
    (camera_scene_node_->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse()) * Ogre::Vector3::UNIT_X;
  double best = 0;
  int sel = -1;
  for (unsigned int i = 0; i < 3; ++i) {
    Ogre::Vector3 axis(0, 0, 0);
    axis[i] = 1.0;
    auto scalar_product = axis.dotProduct(actual);
    if (std::abs(scalar_product) > best) {
      best = std::abs(scalar_product);
      sel = 1 + 2 * i + (scalar_product > 0 ? 0 : 1);
    }
  }
  return sel;
}

void FrameViewController::setAxisFromCamera()
{
  int actual = actualCameraAxisOption();
  if (axis_property_->getOptionInt() == actual) {  // no change?
    return;
  }

  QSignalBlocker block(axis_property_);
  axis_property_->setString(actual == -1 ? ANY_AXIS : fmtAxis(actual));
  rememberAxis(actual);
}

void FrameViewController::changedAxis()
{
  /**
   * Changed axis property only has effect when reset() is called afterwards.
   * We cannot reset the orientation here, otherwise saved/recalled orientation properties
   * would be overwritten by the axis property being recalled and triggering changedAxis().
   */
  rememberAxis(axis_property_->getOptionInt());
  //resetOrientation();
}

inline void FrameViewController::rememberAxis(int current)
{
  if (current >= 1) {  // remember previous axis selection
    previous_axis_ = current;
  }
}

Ogre::Vector3 FrameViewController::getAxis(int option)
{
  Ogre::Vector3 axis(0, 0, 0);
  if (option >= 1 && option <= 6) {
    axis[(option - 1) / 2] = (option % 2) ? +1 : -1;
    if (option >= 3 && invert_z_->getBool()) {
      // When in inverted z mode, y and z axes need sign flip
      axis[(option - 1) / 2] *= -1;
    }
  }
  return axis;
}
Ogre::Quaternion FrameViewController::getRotationToAxis(int option)
{
  Ogre::Quaternion q;
  if (option == 2) {  // special case for the -X axis
    // Create a rotation of 180 degrees around the Z axis
    q = Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI), Ogre::Vector3::UNIT_Z);
  } else {
    Ogre::Vector3 axis = getAxis(option);
    q = Ogre::Vector3::UNIT_X.getRotationTo(axis);
  }
  return q;
}

void FrameViewController::reset()
{
  int option = previous_axis_;

  Ogre::Quaternion rotationToAxis = getRotationToAxis(option);
  Ogre::Vector3 position_behind_axis = rotationToAxis * DEFAULT_FRAMEVIEW_POSITION;
  camera_scene_node_->setPosition(position_behind_axis);

  camera_scene_node_->setOrientation(rotationToAxis * ROBOT_TO_CAMERA_ROTATION);

  setPropertiesFromCamera(camera_);
}

void FrameViewController::updateTargetSceneNode()
{
  /*
   * Overrides updateTargetSceneNode() of FramePositionTrackingViewController to
   * update both position and orientation tracking ot the target frame.
   *
   * Compared to FramePositionTrackingViewController::updateTargetSceneNode():
   * Track both position AND ORIENTATION of the target frame.
   * Take into account z inversion, when activated.
   */
  if (getNewTransform()) {
    target_scene_node_->setPosition(reference_position_);

    Ogre::Quaternion ref_quat = reference_orientation_;
    if (invert_z_->getBool()) {
      ref_quat = ref_quat * Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI), Ogre::Vector3::UNIT_X);
    }
    target_scene_node_->setOrientation(ref_quat);

    context_->queueRender();
  }
}

void FrameViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (locked_property_->getBool()) {
    setStatus("Unlock camera in settings to enable mouse interaction.");
    return;
  }
  FPSViewController::handleMouseEvent(event);
}

void FrameViewController::onTargetFrameChanged(
  const Ogre::Vector3 & /*old_reference_position*/,
  const Ogre::Quaternion & /*old_reference_orientation*/)
{
  /**
   * This empty method overrides the one from FPSViewController.
   * On target frame changed, no offset is applied to position. Just jump to new frame.
   */
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FrameViewController, rviz_common::ViewController)
