/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__FRAME_MANAGER_HPP_
#define RVIZ_COMMON__FRAME_MANAGER_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"

// TODO(wjwwood): reenable this when message_filters is ported.
// #include "tf2_ros/message_filter.h"

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
class FrameManager : public QObject
{
  Q_OBJECT

public:
  enum SyncMode
  {
    SyncOff = 0,
    SyncExact,
    SyncApprox
  };

  /// Constructor.
  /**
   * \param tf a pointer to tf::TransformListener (should not be used anywhere
   *   else because of thread safety).
   */
  explicit
  FrameManager(
    std::shared_ptr<tf2_ros::TransformListener> tf,
    std::shared_ptr<tf2_ros::Buffer> buffer
  );

  /// Destructor.
  /**
   * FrameManager should not need to be destroyed by hand, just destroy the
   * std::shared_ptr returned by instance(), and it will be deleted when the
   * last reference is removed.
   */
  ~FrameManager();

  /// Set the frame to consider "fixed", into which incoming data is transformed.
  /**
   * The fixed frame serves as the reference for all getTransform() and
   * transform() functions in FrameManager.
   */
  void
  setFixedFrame(const std::string & frame);

  /// Enable/disable pause mode.
  void
  setPause(bool pause);

  /// Get pause state.
  bool
  getPause();

  /// Set synchronization mode (off/exact/approximate).
  void
  setSyncMode(SyncMode mode);

  /// Get the synchronization mode.
  SyncMode
  getSyncMode();

  /// Synchronize with given time.
  void
  syncTime(rclcpp::Time time);

  /// Get current time, depending on the sync mode.
  rclcpp::Time
  getTime();

  /// Return the pose for a header, relative to the fixed frame, in Ogre classes.
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
  getTransform(const Header & header, Ogre::Vector3 & position, Ogre::Quaternion & orientation)
  {
    return getTransform(header.frame_id, header.stamp, position, orientation);
  }

  /// Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[in] time The time at which to get the pose.
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the
   *   fixed frame.
   * \return true on success, false on failure.
   */
  bool
  getTransform(
    const std::string & frame,
    rclcpp::Time time,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation);

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
    return transform(header.frame_id, header.stamp, pose, position, orientation);   // NOLINT
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
  bool
  transform(
    const std::string & frame,
    rclcpp::Time time,
    const geometry_msgs::msg::Pose & pose,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation);

  /// Clear the internal cache.
  void
  update();

  /// Check to see if a frame exists in the tf::TransformListener.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[in] time Dummy parameter, not actually used.
   * \param[out] error If the frame does not exist, an error message is
   *   stored here.
   * \return true if the frame does not exist, false if it does exist.
   */
  bool
  frameHasProblems(const std::string & frame, rclcpp::Time time, std::string & error);

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[in] time The time at which the transform is desired.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is. */
  bool
  transformHasProblems(const std::string & frame, rclcpp::Time time, std::string & error);

#if 0
  /// Connect a tf::MessageFilter's callbacks to success and failure handler functions.
  /**
   * FrameManager has internal functions for handling success and
   * failure of tf::MessageFilters which call Display::setStatus()
   * based on success or failure of the filter, including appropriate
   * error messages.
   *
   * \param filter The tf::MessageFilter to connect to.
   * \param display The Display using the filter.
   */
  template<class M>
  void
  registerFilterForTransformStatusCheck(tf2_ros::MessageFilter<M> * filter, Display * display)
  {
    filter->registerCallback(boost::bind(&FrameManager::messageCallback<M>, this, _1, display));
    filter->registerFailureCallback(boost::bind(&FrameManager::failureCallback<M>, this, _1, _2,
      display));
  }
#endif

  /// Return the current fixed frame name.
  const std::string &
  getFixedFrame();

  /// Return the tf::TransformListener used to receive transform data.
  tf2_ros::TransformListener *
  getTFClient();

  /// Return a shared pointer to the tf2_ros::TransformListener used to receive transform data.
  const std::shared_ptr<tf2_ros::TransformListener> &
  getTFClientPtr();

  /// Return a shared pointer to the tf2_ros::Buffer object.
  const std::shared_ptr<tf2_ros::Buffer> &
  getTFBufferPtr();

// TODO(wjwwood): figure out how to replace FilgerFailureReason here
#if 0
  /// Create a description of a transform problem.
  /**
   * \param frame_id The name of the frame with issues.
   * \param stamp The time for which the problem was detected.
   * \param caller_id Dummy parameter, not used.
   * \param reason The reason given by the tf::MessageFilter in its failure callback.
   * \return An error message describing the problem.
   *
   * Once a problem has been detected with a given frame or transform,
   * call this to get an error message describing the problem. */
  std::string
  discoverFailureReason(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::string & caller_id,
    tf::FilterFailureReason reason);
#endif

Q_SIGNALS:
  /// Emitted whenever the fixed frame changes.
  void
  fixedFrameChanged();

private:
  bool
  adjustTime(const std::string & frame, rclcpp::Time & time);

#if 0
  template<class M>
  void
  messageCallback(const ros::MessageEvent<M const> & msg_evt, Display * display)
  {
    boost::shared_ptr<M const> const & msg = msg_evt.getConstMessage();
    std::string authority = msg_evt.getPublisherName();

    messageArrived(msg->header.frame_id, msg->header.stamp, authority, display);
  }
#endif

#if 0
  template<class M>
  void failureCallback(
    const ros::MessageEvent<M const> & msg_evt,
    tf::FilterFailureReason reason,
    Display * display)
  {
    boost::shared_ptr<M const> const & msg = msg_evt.getConstMessage();
    std::string authority = msg_evt.getPublisherName();

    messageFailed(msg->header.frame_id, msg->header.stamp, authority, reason, display);
  }
#endif

  void
  messageArrived(
    const std::string & frame_id,
    const rclcpp::Time & stamp,
    const std::string & caller_id,
    Display * display);

#if 0
  void
  messageFailed(
    const std::string & frame_id,
    const ros::Time & stamp,
    const std::string & caller_id,
    tf::FilterFailureReason reason,
    Display * display);
#endif

  struct CacheKey
  {
    CacheKey(const std::string & f, rclcpp::Time t)
    : frame(f),
      time(t)
    {}

    bool operator<(const CacheKey & rhs) const
    {
      if (frame != rhs.frame) {
        return frame < rhs.frame;
      }

      return time < rhs.time;
    }

    std::string frame;
    rclcpp::Time time;
  };

  struct CacheEntry
  {
    CacheEntry(const Ogre::Vector3 & p, const Ogre::Quaternion & o)
    : position(p),
      orientation(o)
    {}

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };
  typedef std::map<CacheKey, CacheEntry> M_Cache;

  std::mutex cache_mutex_;
  M_Cache cache_;

  std::shared_ptr<tf2_ros::TransformListener> tf_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::string fixed_frame_;

  bool pause_;

  SyncMode sync_mode_;

  // the current synchronized time, used to overwrite ros:Time(0)
  rclcpp::Time sync_time_;

  // used for approx. syncing (in nanoseconds)
  int64_t sync_delta_;
  int64_t current_delta_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__FRAME_MANAGER_HPP_
