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
  Time(int32_t sec, uint32_t nanosec);

  int32_t seconds;
  uint32_t nanoseconds;
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

struct PoseStamped
{
public:
  RVIZ_COMMON_PUBLIC
  PoseStamped();

  RVIZ_COMMON_PUBLIC
  PoseStamped(Time time, std::string frame, Point position_vector, Quaternion orientation_quat);

  Time time_stamp;
  std::string frame_id;
  Point position;
  Quaternion orientation;
};

struct TransformStamped
{
public:
  RVIZ_COMMON_PUBLIC
  TransformStamped();

  RVIZ_COMMON_PUBLIC
  TransformStamped(
    Time time,
    std::string parent_frame,
    std::string child_frame,
    Point translation_vector,
    Quaternion rotation_quat);

  Time time_stamp;
  std::string parent_frame_id;
  std::string child_frame_id;
  Point translation;
  Quaternion rotation;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__STRUCTS_HPP_
