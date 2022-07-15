/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_COMMON__FRAME_POSITION_TRACKING_VIEW_CONTROLLER_HPP_
#define RVIZ_COMMON__FRAME_POSITION_TRACKING_VIEW_CONTROLLER_HPP_

#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "rviz_common/view_controller.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{

namespace properties
{
class TfFrameProperty;
}  // namespace properties

/// Base class of ViewControllers which have a "Target Frame".
/**
 * The "Target Frame" is a TF frame whose position they track.
 */
class RVIZ_COMMON_PUBLIC FramePositionTrackingViewController : public ViewController
{
  Q_OBJECT

public:
  FramePositionTrackingViewController();

  ~FramePositionTrackingViewController() override;

  /// Do subclass-specific initialization.
  /**
   * Called by ViewController::initialize after context_, target_scene_node_,
   * and camera_ are set.
   * This version calls updateTargetSceneNode().
   */
  void onInitialize() override;

  /// Called by activate().
  /**
   * Override to implement view-specific activation.
   * This version calls updateTargetSceneNode().
   */
  void onActivate() override;

  void update(float dt, float ros_dt) override;

  /// Configure the settings of this view controller to give a similar view as the source_view.
  /**
   * source_view must return a valid Ogre::Camera* from getCamera().
   *
   * This implementation sets the target frame property.
   */
  void mimic(ViewController * source_view) override;

protected Q_SLOTS:
  /// Called when Target Frame property changes while view is active.
  /**
   * Purpose is to change values in the view controller (like
   * a position offset) such that the actual viewpoint does not
   * change.
   * Calls updateTargetSceneNode() and onTargetFrameChanged().
   */
  virtual void updateTargetFrame();

protected:
  /// Override to implement the change in properties which nullifies the change in target frame.
  /**
   * \see updateTargetFrame()
   */
  virtual void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation);

  bool getNewTransform();

  /// Update the position of the target_scene_node_ based on the Target Frame property.
  virtual void updateTargetSceneNode();

  rviz_common::properties::TfFrameProperty * target_frame_property_;
  Ogre::SceneNode * target_scene_node_;
  Ogre::Quaternion reference_orientation_;
  Ogre::Vector3 reference_position_;

  /// A child scene node to position and rotate the camera
  Ogre::SceneNode * camera_scene_node_;
};

}  // end namespace rviz_common

#endif  // RVIZ_COMMON__FRAME_POSITION_TRACKING_VIEW_CONTROLLER_HPP_
