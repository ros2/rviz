/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_
#define RVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

#include <OgreVector.h>
#include <OgreQuaternion.h>

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

// TODO(wjwwood): reenable this when message_filters is ported.
// #include "tf2_ros/message_filter.h"

#include "rviz_common/visibility_control.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

namespace tf2_ros
{

class Buffer;
class TransformListener;

}  // namespace tf2_ros

namespace rviz_common
{

class Display;

/// Helper class for transforming data into Ogre's world frame (the fixed frame).
/**
 * During one frame update (nominally 33ms), the tf tree stays consistent and
 * queries are cached for speedup.
 */
class RVIZ_COMMON_PUBLIC FrameManagerIface : public QObject
{
  Q_OBJECT

public:
  enum SyncMode
  {
    SyncOff = 0,
    SyncExact,
    SyncApprox
  };

  /// Set the frame to consider "fixed", into which incoming data is transformed.
  /**
   * The fixed frame serves as the reference for all getTransform() and
   * transform() functions in FrameManager.
   */
  virtual
  void
  setFixedFrame(const std::string & frame) = 0;

  /// Enable/disable pause mode.
  virtual
  void
  setPause(bool pause) = 0;

  /// Get pause state.
  virtual
  bool
  getPause() = 0;

  /// Set synchronization mode (off/exact/approximate).
  virtual
  void
  setSyncMode(SyncMode mode) = 0;

  /// Get the synchronization mode.
  virtual
  SyncMode
  getSyncMode() = 0;

  /// Synchronize with given time.
  virtual
  void
  syncTime(rclcpp::Time time) = 0;

  /// Get current time, depending on the sync mode.
  virtual
  rclcpp::Time
  getTime() = 0;

  /// Return the pose for a header, relative to the fixed frame, in Ogre classes, at a given time.
  /**
   * \param[in] header The source of the frame name and time.
   * \param[out] position The position of the header frame relative to the
   *   fixed frame.
   * \param[out] orientation The orientation of the header frame relative to
   *   the fixed frame.
   * \return true on success, false on failure.
   */
  template<typename Header>
  bool
  getTransform(
    const Header & header,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation)
  {
    rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
    return getTransform(header.frame_id, time_stamp, position, orientation);
  }

  /// Return the pose for a frame relative to the fixed frame, in Ogre classes, at the most recent
  /// time there is data for.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the
   *   fixed frame.
   * \return true on success, false on failure.
   */
  virtual
  bool
  getTransform(
    const std::string & frame,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) = 0;

  /// Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[in] time The time at which to get the pose.
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the
   *   fixed frame.
   * \return true on success, false on failure.
   */
  virtual
  bool
  getTransform(
    const std::string & frame,
    rclcpp::Time time,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) = 0;

  /// Transform a pose from a frame into the fixed frame.
  /**
   * \param[in] header The source of the input frame and time.
   * \param[in] pose The input pose, relative to the header frame.
   * \param[out] position Position part of pose relative to the fixed frame.
   * \param[out] orientation: Orientation part of pose relative to the
   *   fixed frame.
   * \return true on success, false on failure.
   */
  template<typename Header>
  bool
  transform(
    const Header & header,
    const geometry_msgs::msg::Pose & pose,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation)
  {
    rclcpp::Time time_stamp(header.stamp, RCL_ROS_TIME);
    return transform(header.frame_id, time_stamp, pose, position, orientation);   // NOLINT
    // linter wants #include <algorithm> for transform
  }

  /// Transform a pose from a frame into the fixed frame.
  /**
   * \param[in] frame The input frame.
   * \param[in] time The time at which to get the pose.
   * \param[in] pose The input pose, relative to the input frame.
   * \param[out] position Position part of pose relative to the fixed frame.
   * \param[out] orientation: Orientation part of pose relative to the fixed
   *   frame.
   * \return true on success, false on failure.
   */
  virtual
  bool
  transform(
    const std::string & frame,
    rclcpp::Time time,
    const geometry_msgs::msg::Pose & pose,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) = 0;

  /// Clear the internal cache.
  virtual
  void
  update() = 0;

  /// Check to see if a frame exists in the tf::TransformListener.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the frame does not exist, an error message is
   *   stored here.
   * \return true if the frame does not exist, false if it does exist.
   */
  virtual
  bool
  frameHasProblems(const std::string & frame, std::string & error) = 0;

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is. */
  virtual
  bool
  transformHasProblems(const std::string & frame, std::string & error) = 0;

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[in] time The time at which the transform is desired.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is. */
  virtual
  bool
  transformHasProblems(const std::string & frame, rclcpp::Time time, std::string & error) = 0;

  /// Return the current fixed frame name.
  virtual
  const std::string &
  getFixedFrame() = 0;

  /// Return a weak pointer to the internal transformation object.
  virtual
  transformation::TransformationLibraryConnector::WeakPtr
  getConnector() = 0;

  /// Return a shared pointer to the transformer object.
  virtual
  std::shared_ptr<transformation::FrameTransformer>
  getTransformer() = 0;

  virtual
  std::vector<std::string>
  getAllFrameNames() = 0;

public Q_SLOTS:
  virtual
  void
  setTransformerPlugin(std::shared_ptr<transformation::FrameTransformer> transformer) = 0;

Q_SIGNALS:
  void
  fixedFrameChanged();
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_
