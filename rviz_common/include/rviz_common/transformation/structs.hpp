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

#ifndef RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_
#define RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_

#include <string>

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace transformation
{

struct RVIZ_COMMON_PUBLIC Time
{
  Time();
  Time(int32_t seconds, uint32_t nanoseconds);

  int32_t seconds_;
  uint32_t nanoseconds_;
};

struct RVIZ_COMMON_PUBLIC Point
{
  Point();
  Point(double x_co, double y_co, double z_co);

  double x;
  double y;
  double z;
};

struct RVIZ_COMMON_PUBLIC Quaternion
{
  Quaternion();
  Quaternion(double w_co, double x_co, double y_co, double z_co);

  double w;
  double x;
  double y;
  double z;
};

struct RVIZ_COMMON_PUBLIC PoseStamped
{
public:
  PoseStamped();
  PoseStamped(
    Time time_stamp, std::string frame_id, Point position, Quaternion orientation);

  Time time_stamp_;
  std::string frame_id_;
  Point position_;
  Quaternion orientation_;
};

struct RVIZ_COMMON_PUBLIC TransformStamped
{
public:
  TransformStamped();
  TransformStamped(
    Time time_stamp,
    std::string parent_frame,
    std::string child_frame,
    Point translation,
    Quaternion rotation);

  Time time_stamp_;
  std::string parent_frame_id_;
  std::string child_frame_id_;
  Point translation_;
  Quaternion rotation_;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_
