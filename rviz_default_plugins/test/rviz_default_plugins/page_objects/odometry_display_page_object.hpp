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

#ifndef RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__ODOMETRY_DISPLAY_PAGE_OBJECT_HPP_
#define RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__ODOMETRY_DISPLAY_PAGE_OBJECT_HPP_

#include "rviz_visual_testing_framework/page_objects/base_page_object.hpp"

class OdometryDisplayPageObject : public BasePageObject
{
public:
  OdometryDisplayPageObject();

  void setTopic(QString topic);

  void setPositionTolerance(float position_tolerance);

  void setAngleTolerance(float angle_tolerance);

  void setKeepShapes(int keep);

  void setShape(QString shape);

  void setArrowColor(int r, int g, int b);

  void setArrowAlpha(float alpha);

  void setArrowShaftLength(float shaft_length);

  void setArrowShaftRadius(float shaft_radius);

  void setArrowHeadLength(float head_length);

  void setArrowHeadRadius(float head_radius);

  void setAxesLength(float length);

  void setAxesRadius(float radius);

  void setCovariance(bool visible);

  void setCovariancePosition(bool visible);

  void setCovariancePositionColor(int r, int g, int b);

  void setCovariancePositionAlpha(float alpha);

  void setCovariancePositionScale(float scale);

  void setCovarianceOrientation(bool visible);

  void setCovarianceOrientationFrame(QString frame);

  void setCovarianceOrientationColorStyle(QString style);

  void setCovarianceOrientationColor(int r, int g, int b);

  void setCovarianceOrientationAlpha(float alpha);

  void setCovarianceOrientationOffset(float offset);

  void setCovarianceOrientationScale(float scale);
};

#endif  // RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__ODOMETRY_DISPLAY_PAGE_OBJECT_HPP_
