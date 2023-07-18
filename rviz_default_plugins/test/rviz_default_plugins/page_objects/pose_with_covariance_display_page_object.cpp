/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2019, Martin Idel
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

#include <QString>

#include "pose_with_covariance_display_page_object.hpp"

PoseWithCovarianceDisplayPageObject::PoseWithCovarianceDisplayPageObject()
: BasePageObject(0, "PoseWithCovariance")
{}

void PoseWithCovarianceDisplayPageObject::setTopic(const QString & topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}


void PoseWithCovarianceDisplayPageObject::setShape(const QString & shape)
{
  setComboBox("Shape", shape);
}

void PoseWithCovarianceDisplayPageObject::setArrowColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b);
}

void PoseWithCovarianceDisplayPageObject::setArrowAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void PoseWithCovarianceDisplayPageObject::setArrowShaftLength(float shaft_length)
{
  setFloat("Shaft Length", shaft_length);
}

void PoseWithCovarianceDisplayPageObject::setArrowShaftRadius(float shaft_radius)
{
  setFloat("Shaft Radius", shaft_radius);
}

void PoseWithCovarianceDisplayPageObject::setArrowHeadLength(float head_length)
{
  setFloat("Head Length", head_length);
}

void PoseWithCovarianceDisplayPageObject::setArrowHeadRadius(float head_radius)
{
  setFloat("Head Radius", head_radius);
}

void PoseWithCovarianceDisplayPageObject::setAxesLength(float length)
{
  setFloat("Axes Length", length);
}

void PoseWithCovarianceDisplayPageObject::setAxesRadius(float radius)
{
  setFloat("Axes Radius", radius);
}

void PoseWithCovarianceDisplayPageObject::setCovariance(bool visible)
{
  setBool("Covariance", visible);
}

void PoseWithCovarianceDisplayPageObject::setCovariancePosition(bool visible)
{
  setBool("Position", visible, {"Covariance"});
}

void PoseWithCovarianceDisplayPageObject::setCovariancePositionColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b, {"Covariance", "Position"});
}

void PoseWithCovarianceDisplayPageObject::setCovariancePositionAlpha(float alpha)
{
  setFloat("Alpha", alpha, {"Covariance", "Position"});
}

void PoseWithCovarianceDisplayPageObject::setCovariancePositionScale(float scale)
{
  setFloat("Scale", scale, {"Covariance", "Position"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientation(bool visible)
{
  setBool("Orientation", visible, {"Covariance"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationFrame(const QString & frame)
{
  setComboBox("Frame", frame, {"Covariance", "Orientation"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationColorStyle(const QString & style)
{
  setComboBox("Color Style", style, {"Covariance", "Orientation"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b, {"Covariance", "Orientation"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationAlpha(float alpha)
{
  setFloat("Alpha", alpha, {"Covariance", "Orientation"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationOffset(float offset)
{
  setFloat("Offset", offset, {"Covariance", "Orientation"});
}

void PoseWithCovarianceDisplayPageObject::setCovarianceOrientationScale(float scale)
{
  setFloat("Scale", scale, {"Covariance", "Orientation"});
}
