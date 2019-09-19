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

#include "odometry_display_page_object.hpp"

OdometryDisplayPageObject::OdometryDisplayPageObject()
: BasePageObject(0, "Odometry")
{}

void OdometryDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void OdometryDisplayPageObject::setPositionTolerance(float position_tolerance)
{
  setFloat("Position Tolerance", position_tolerance);
}

void OdometryDisplayPageObject::setAngleTolerance(float angle_tolerance)
{
  setFloat("Angle Tolerance", angle_tolerance);
}

void OdometryDisplayPageObject::setKeepShapes(int keep)
{
  setInt("Keep", keep);
}

void OdometryDisplayPageObject::setShape(QString shape)
{
  setComboBox("Shape", shape);
}

void OdometryDisplayPageObject::setArrowColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b, {"Shape"});
}

void OdometryDisplayPageObject::setArrowAlpha(float alpha)
{
  setFloat("Alpha", alpha, {"Shape"});
}

void OdometryDisplayPageObject::setArrowShaftLength(float shaft_length)
{
  setFloat("Shaft Length", shaft_length, {"Shape"});
}

void OdometryDisplayPageObject::setArrowShaftRadius(float shaft_radius)
{
  setFloat("Shaft Radius", shaft_radius, {"Shape"});
}

void OdometryDisplayPageObject::setArrowHeadLength(float head_length)
{
  setFloat("Head Length", head_length, {"Shape"});
}

void OdometryDisplayPageObject::setArrowHeadRadius(float head_radius)
{
  setFloat("Head Radius", head_radius, {"Shape"});
}

void OdometryDisplayPageObject::setAxesLength(float length)
{
  setFloat("Axes Length", length, {"Shape"});
}

void OdometryDisplayPageObject::setAxesRadius(float radius)
{
  setFloat("Axes Radius", radius, {"Shape"});
}

void OdometryDisplayPageObject::setCovariance(bool visible)
{
  setBool("Covariance", visible);
}

void OdometryDisplayPageObject::setCovariancePosition(bool visible)
{
  setBool("Position", visible, {"Covariance"});
}

void OdometryDisplayPageObject::setCovariancePositionColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b, {"Covariance", "Position"});
}

void OdometryDisplayPageObject::setCovariancePositionAlpha(float alpha)
{
  setFloat("Alpha", alpha, {"Covariance", "Position"});
}

void OdometryDisplayPageObject::setCovariancePositionScale(float scale)
{
  setFloat("Scale", scale, {"Covariance", "Position"});
}

void OdometryDisplayPageObject::setCovarianceOrientation(bool visible)
{
  setBool("Orientation", visible, {"Covariance"});
}

void OdometryDisplayPageObject::setCovarianceOrientationFrame(QString frame)
{
  setComboBox("Frame", frame, {"Covariance", "Orientation"});
}

void OdometryDisplayPageObject::setCovarianceOrientationColorStyle(QString style)
{
  setComboBox("Color Style", style, {"Covariance", "Orientation"});
}

void OdometryDisplayPageObject::setCovarianceOrientationColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b, {"Covariance", "Orientation"});
}

void OdometryDisplayPageObject::setCovarianceOrientationAlpha(float alpha)
{
  setFloat("Alpha", alpha, {"Covariance", "Orientation"});
}

void OdometryDisplayPageObject::setCovarianceOrientationOffset(float offset)
{
  setFloat("Offset", offset, {"Covariance", "Orientation"});
}

void OdometryDisplayPageObject::setCovarianceOrientationScale(float scale)
{
  setFloat("Scale", scale, {"Covariance", "Orientation"});
}
