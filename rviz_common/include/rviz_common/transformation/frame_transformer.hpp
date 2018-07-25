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

class RVIZ_COMMON_PUBLIC InternalFrameTransformer
{
public:
  virtual ~InternalFrameTransformer() = default;
};

class FrameTransformerException : public std::runtime_error
{
public:
  RVIZ_COMMON_PUBLIC explicit FrameTransformerException(const char * error_message)
  : std::runtime_error(error_message) {}

  RVIZ_COMMON_PUBLIC ~FrameTransformerException() noexcept override = default;
};

using InternalFrameTransformerPtr = std::weak_ptr<InternalFrameTransformer>;

class RVIZ_COMMON_PUBLIC FrameTransformer
{
public:
  virtual void initialize(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node) = 0;

  virtual void clear() = 0;

  virtual std::vector<std::string> getAllFrameNames() = 0;

  virtual transformation::PoseStamped transform(
    // NOLINT (this is not std::transform)
    const transformation::PoseStamped & pose_in, const std::string & target_frame) = 0;

  virtual bool transformIsAvailable(
    const std::string & target_frame, const std::string & source_frame) = 0;

  virtual bool transformHasProblems(
    const std::string & source_frame,
    const std::string & target_frame,
    const transformation::Time & time,
    std::string & error) = 0;

  virtual bool frameHasProblems(const std::string & frame, std::string & error) = 0;

//  virtual void waitForValidTransform(
//    std::string target_frame,
//    std::string source_frame,
//    transfrormation::Time time,
//    std::function<void(void)> callback) = 0;

  /// Expose internal implementation
  virtual InternalFrameTransformerPtr getInternals() = 0;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_
