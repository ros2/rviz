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

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001f;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001f;

static const Ogre::Vector3 DEFAULT_FPS_POSITION = Ogre::Vector3(5, 5, 10);

FrameViewController::FrameViewController()
{
  axis_property_ = new rviz_common::properties::EnumProperty("Point towards", fmtAxis(6),
                                    "Point the camera along the given axis of the frame.", this,
                                    SLOT(changedAxis()));
  axis_property_->addOption(ANY_AXIS, -1);

  // x,y,z axes get integers from 1..6: +x, -x, +y, -y, +z, -z
  for (int i = 1; i <= 6; ++i) {
    axis_property_->addOption(fmtAxis(i), i);
  }
  previous_axis_ = axis_property_->getOptionInt();

  locked_property_ = new rviz_common::properties::BoolProperty("Lock Camera", false,
    "Lock camera in its current pose relative to the frame", this);
}

void FrameViewController::onInitialize()
{
  FPSViewController::onInitialize();
  changedAxis();
}

int FrameViewController::actualCameraAxisOption(double precision) const
{
  // compare current camera direction with unit axes
  Ogre::Vector3 actual =
    (camera_scene_node_->getOrientation() * ROBOT_TO_CAMERA_ROTATION.Inverse()) *
    Ogre::Vector3::UNIT_X;
  for (unsigned int i = 0; i < 3; ++i) {
    Ogre::Vector3 axis(0, 0, 0);
    axis[i] = 1.0;
    auto scalar_product = axis.dotProduct(actual);
    if (std::abs(scalar_product) > 1.0 - precision) {
      return 1 + 2 * i + (scalar_product > 0 ? 0 : 1);
    }
  }
  return -1;
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
  rememberAxis(axis_property_->getOptionInt());
  reset();
}

inline void FrameViewController::rememberAxis(int current)
{
  if (current >= 1) {  // remember previous axis selection
    previous_axis_ = current;
  }
}

void FrameViewController::reset()
{
  camera_scene_node_->setPosition(Ogre::Vector3::ZERO);
  Ogre::Vector3 axis(0, 0, 0);
  int option = previous_axis_;
  if (option >= 1 && option <= 6) {
    axis[(option - 1) / 2] = (option % 2) ? +1 : -1;
    Ogre::Quaternion q;
    if (option == 2) {  // special case for the -X axis
      // Create a rotation of 180 degrees around the Z axis
      q = Ogre::Quaternion(Ogre::Radian(Ogre::Math::PI), Ogre::Vector3::UNIT_Z);
    } else {
      q = Ogre::Vector3::UNIT_X.getRotationTo(axis);
    }
    camera_scene_node_->setOrientation(q * ROBOT_TO_CAMERA_ROTATION);
  }
  setPropertiesFromCamera(camera_);
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
  // don't adapt the camera pose to the old reference position, but just jump to new frame
}

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)
PLUGINLIB_EXPORT_CLASS(
  rviz_default_plugins::view_controllers::FrameViewController, rviz_common::ViewController)
