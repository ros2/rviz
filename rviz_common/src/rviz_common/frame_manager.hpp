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
#include <vector>

#include <OgreVector.h>
#include <OgreQuaternion.h>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "rviz_common/visibility_control.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

namespace rviz_common
{

class Display;

/// Helper class for transforming data into Ogre's world frame (the fixed frame).
/**
 * During one frame update (nominally 33ms), the tf tree stays consistent and
 * queries are cached for speedup.
 */
class FrameManager : public FrameManagerIface
{
  Q_OBJECT

public:
  /// Constructor.
  /**
   * \param tf a pointer to tf::TransformListener (should not be used anywhere
   *   else because of thread safety).
   */
  FrameManager(
    rclcpp::Clock::SharedPtr clock, std::shared_ptr<transformation::FrameTransformer> transformer);

  /// Destructor.
  /**
   * FrameManager should not need to be destroyed by hand, just destroy the
   * std::shared_ptr returned by instance(), and it will be deleted when the
   * last reference is removed.
   */
  ~FrameManager() override = default;

  /// Set the frame to consider "fixed", into which incoming data is transformed.
  /**
   * The fixed frame serves as the reference for all getTransform() and
   * transform() functions in FrameManager.
   */
  void setFixedFrame(const std::string & frame) override;

  /// Enable/disable pause mode.
  void setPause(bool pause) override;

  /// Get pause state.
  bool getPause() override;

  /// Set synchronization mode (off/exact/approximate).
  void setSyncMode(SyncMode mode) override;

  /// Get the synchronization mode.
  SyncMode getSyncMode() override;

  /// Synchronize with given time.
  void syncTime(rclcpp::Time time) override;

  /// Get current time, depending on the sync mode.
  rclcpp::Time getTime() override;

  /// Return the pose for a frame relative to the fixed frame, in Ogre classes, at a given time.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the
   *   fixed frame.
   * \return true on success, false on failure.
   */
  bool getTransform(
    const std::string & frame, Ogre::Vector3 & position, Ogre::Quaternion & orientation) override
  {
    return getTransform(frame, rclcpp::Time(0, 0, clock_->get_clock_type()), position, orientation);
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
  bool getTransform(
    const std::string & frame,
    rclcpp::Time time,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) override;

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
  bool transform(
    const std::string & frame,
    rclcpp::Time time,
    const geometry_msgs::msg::Pose & pose,
    Ogre::Vector3 & position,
    Ogre::Quaternion & orientation) override;

  /// Clear the internal cache.
  void update() override;

  /// Check to see if a frame exists in the tf::TransformListener.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the frame does not exist, an error message is
   *   stored here.
   * \return true if the frame does not exist, false if it does exist.
   */
  bool frameHasProblems(const std::string & frame, std::string & error) override;

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is. */
  bool transformHasProblems(const std::string & frame, std::string & error) override
  {
    return transformHasProblems(frame, rclcpp::Time(0, 0, clock_->get_clock_type()), error);
  }

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[in] time The time at which the transform is desired.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is. */
  bool transformHasProblems(
    const std::string & frame, rclcpp::Time time, std::string & error) override;

  /// Return the current fixed frame name.
  const std::string & getFixedFrame() override;

  /// Return a weak pointer to the internal transformation object.
  transformation::TransformationLibraryConnector::WeakPtr getConnector() override;

  /// Return a shared pointer to the transformer object.
  std::shared_ptr<transformation::FrameTransformer> getTransformer() override;

  std::vector<std::string> getAllFrameNames() override;

  virtual void clear();

  virtual bool anyTransformationDataAvailable();

public Q_SLOTS:
  void setTransformerPlugin(
    std::shared_ptr<rviz_common::transformation::FrameTransformer> transformer) override;

private:
  bool adjustTime(const std::string & frame, rclcpp::Time & time);

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

  std::shared_ptr<transformation::FrameTransformer> transformer_;
  std::string fixed_frame_;

  bool pause_;

  SyncMode sync_mode_;

  // the current synchronized time, used to overwrite ros:Time(0)
  rclcpp::Time sync_time_;

  rclcpp::Clock::SharedPtr clock_;

  // used for approx. syncing (in nanoseconds)
  int64_t sync_delta_;
  int64_t current_delta_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__FRAME_MANAGER_HPP_
