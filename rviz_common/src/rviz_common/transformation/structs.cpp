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

#include "rviz_common/transformation/structs.hpp"

#include <string>

namespace rviz_common
{
namespace transformation
{

Time::Time()
: seconds(0), nanoseconds(0) {}

Time::Time(int32_t sec, uint32_t nanosec)
: seconds(sec), nanoseconds(nanosec)
{}

Point::Point()
: x(0), y(0), z(0) {}

Point::Point(double x, double y, double z)
: x(x), y(y), z(z)
{}

Quaternion::Quaternion()
: w(1), x(0), y(0), z(0) {}

Quaternion::Quaternion(double w, double x, double y, double z)
: w(w), x(x), y(y), z(z)
{}

PoseStamped::PoseStamped()
: time_stamp(), frame_id(""), position(), orientation() {}

PoseStamped::PoseStamped(
  Time time, std::string frame, Point position_vector, Quaternion orientation_quat)
: time_stamp(time), frame_id(frame), position(position_vector), orientation(orientation_quat)
{}

TransformStamped::TransformStamped()
: time_stamp(), parent_frame_id(""), child_frame_id(""), translation(), rotation()
{}

TransformStamped::TransformStamped(
  Time time,
  std::string parent_frame,
  std::string child_frame,
  Point translation_vector,
  Quaternion rotation_quat)
: time_stamp(time),
  parent_frame_id(parent_frame),
  child_frame_id(child_frame),
  translation(translation_vector),
  rotation(rotation_quat)
{}

}  // namespace transformation
}  // namespace rviz_common
