/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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
 *     * Neither the name of the copyright holder nor the names of its contributors
 *       may be used to endorse or promote products derived from
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

#ifndef RVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_
#define RVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "rviz_common/visibility_control.hpp"

#include "rviz_common/transformation/structs.hpp"

namespace rviz_common
{
namespace transformation
{

class FrameTransformerException : public std::runtime_error
{
public:
  RVIZ_COMMON_PUBLIC explicit FrameTransformerException(const char * error_message)
  : std::runtime_error(error_message) {}

  RVIZ_COMMON_PUBLIC ~FrameTransformerException() noexcept override = default;
};

/** \brief class which the internal (plugin specific) implementation member of
 *  FrameTransformer should inherit from
 */
class RVIZ_COMMON_PUBLIC InternalFrameTransformer
{
public:
  virtual ~InternalFrameTransformer() = default;
};

using InternalFrameTransformerPtr = std::weak_ptr<InternalFrameTransformer>;

class RVIZ_COMMON_PUBLIC FrameTransformer
{
public:
  /**
   * \brief The pluginlib needs a no-parameters constructor. The initialization of a
   * FrameTransformer object is therefore delegated to this method */
  virtual void initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node) = 0;

  /** \brief Method thought to reset the internal implementation object */
  virtual void clear() = 0;

  /** @returns A std::string vector containing all the available frame ids */
  virtual std::vector<std::string> getAllFrameNames() = 0;

  /** \brief Transform a PoseStamped into a given target frame
   * @param pose_in The pose to be transformed
   * @param target_frame The frame into which to transform
   * @returns The transformed PoseStamped */
  virtual transformation::PoseStamped transform(
    // NOLINT (this is not std::transform)
    const transformation::PoseStamped & pose_in, const std::string & target_frame) = 0;

  /** \brief Checks if a transformation between two frames is available
   * @param target_frame The target frame of the transformation
   * @param source_frame The source frame of the transformation
   * @returns Whether or not a transformation between the two frames is available */
  virtual bool transformIsAvailable(
    const std::string & target_frame, const std::string & source_frame) = 0;

  /** \brief Checks that a given transformation can be performed
   * @param target_frame The target frame of the transformation
   * @param source_frame The source frame of the transformation
   * @param time The time of the transformation
   * @param error An out string in which an error (of generated) message is saved
   * @returns Whether or not the given transformation has some problem and cannot be perfored */
  virtual bool transformHasProblems(
    const std::string & source_frame,
    const std::string & target_frame,
    const rclcpp::Time & time,
    std::string & error) = 0;

  /** \brief Checks that a given frame exsists and can be used
   * @param frame The frame to check
   * @param error An out string in which an error message (if generated) is saved
   * @returns Whether or not the given frame has some problem */
  virtual bool frameHasProblems(const std::string & frame, std::string & error) = 0;

  /** \brief A getter for the internal implementation object */
  virtual InternalFrameTransformerPtr getInternals() = 0;

//  /** \brief Waits until a transformation between two given frames is available, then performs an
//   * action
//   * @param target_frame The target frame of the transformation
//   * @param source_frame The source frame of the transformation
//   * @param time The time of the tranformation
//   * @param timeout A timeout duration after which the waiting will be stopped
//   * @param callback The function to be called if the tranfrom becomes available before the
//   * timeout */
//  virtual void waitForValidTransform(
//    std::string target_frame,
//    std::string source_frame,
//    rclcpp::Time time,
//    rclcpp::Duration timeout,
//    std::function<void(void)> callback) = 0;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_
