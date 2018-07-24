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
: seconds_(0), nanoseconds_(0) {}

Time::Time(int32_t seconds, uint32_t nanoseconds)
{
  seconds_ = seconds;
  nanoseconds_ = nanoseconds;
}

Point::Point()
: x_(0), y_(0), z_(0) {}

Point::Point(double x, double y, double z)
{
  x_ = x;
  y_ = y;
  z_ = z;
}

Quaternion::Quaternion()
: w_(1), x_(0), y_(0), z_(0) {}

Quaternion::Quaternion(double w, double x, double y, double z)
{
  w_ = w;
  x_ = x;
  y_ = y;
  z_ = z;
}

PoseStamped::PoseStamped()
: time_stamp_(), frame_id_(""), position_(), orientation_() {}

PoseStamped::PoseStamped(
  Time time_stamp, std::string frame_id, Point position, Quaternion orientation)
: time_stamp_(time_stamp), frame_id_(frame_id), position_(position), orientation_(orientation)
{}

TransformStamped::TransformStamped()
: time_stamp_(), parent_frame_id_(""), child_frame_id_(""), translation_(), rotation_()
{}

TransformStamped::TransformStamped(
  Time time_stamp,
  std::string parent_frame,
  std::string child_frame,
  Point translation,
  Quaternion rotation)
: time_stamp_(time_stamp),
  parent_frame_id_(parent_frame),
  child_frame_id_(child_frame),
  translation_(translation),
  rotation_(rotation)
{}

}  // namespace transformation
}  // namespace rviz_common
