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

#include "path_display_page_object.hpp"

#include <memory>
#include <vector>

#include <QTest>  // NOLINT

#include "rviz_visual_testing_framework/test_helpers.hpp"

PathDisplayPageObject::PathDisplayPageObject()
: BasePageObject(0, "Path")
{}

void PathDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void PathDisplayPageObject::setLineStyleForPath(QString line_style)
{
  setComboBox("Line Style", line_style);
}

void PathDisplayPageObject::setLineWidthForPath(float width)
{
  setFloat("Line Width", width);
}

void PathDisplayPageObject::setPathColor(int red, int green, int blue)
{
  setColorCode("Color", red, green, blue);
}

void PathDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void PathDisplayPageObject::setLengthOfBuffer(int buffer_length)
{
  setInt("Buffer Length", buffer_length);
}

void PathDisplayPageObject::setOffsetFromOrigin(float x, float y, float z)
{
  setVector("Offset", x, y, z);
}

void PathDisplayPageObject::setStyleOfPose(QString pose_style)
{
  setComboBox("Pose Style", pose_style);
}

void PathDisplayPageObject::setPoseAxesLength(float length)
{
  setFloat("Length", length);
}

void PathDisplayPageObject::setPoseAxesRadius(float radius)
{
  setFloat("Radius", radius);
}

void PathDisplayPageObject::setPoseArrowColorProperty(int red, int green, int blue)
{
  setColorCode("Pose Color", red, green, blue);
}

void PathDisplayPageObject::setPoseArrowShaftLength(float length)
{
  setFloat("Shaft Length", length);
}

void PathDisplayPageObject::setPoseArrowHeadLength(float length)
{
  setFloat("Head Length", length);
}

void PathDisplayPageObject::setPoseArrowShaftDiameter(float diameter)
{
  setFloat("Shaft Diameter", diameter);
}

void PathDisplayPageObject::setPoseArrowHeadDiameter(float diameter)
{
  setFloat("Head Diameter", diameter);
}
