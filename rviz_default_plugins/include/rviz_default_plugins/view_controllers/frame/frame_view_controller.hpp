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


#ifndef RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FRAME__FRAME_VIEW_CONTROLLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FRAME__FRAME_VIEW_CONTROLLER_HPP_

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

#include <OgreVector.h>
#include <OgreQuaternion.h>

#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include "rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp"

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"

namespace rviz_default_plugins
{
namespace view_controllers
{
/** @brief A frame-aligned camera. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC FrameViewController : public FPSViewController
{
  Q_OBJECT

public:
  FrameViewController();

  ~FrameViewController() override = default;

  void onInitialize() override;

  void handleMouseEvent(rviz_common::ViewportMouseEvent & evt) override;

  void reset() override;

  void yaw(float angle);

  void pitch(float angle);

protected:
  void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation) override;

  rviz_common::properties::EnumProperty * axis_property_;
  rviz_common::properties::BoolProperty * locked_property_;

  int actualCameraAxisOption(double precision = 0.001) const;

  void setAxisFromCamera();

protected Q_SLOTS:
  void changedAxis();

private:
  void rememberAxis(int current);

  int previous_axis_;
};

}  // namespace view_controllers
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__VIEW_CONTROLLERS__FRAME__FRAME_VIEW_CONTROLLER_HPP_
