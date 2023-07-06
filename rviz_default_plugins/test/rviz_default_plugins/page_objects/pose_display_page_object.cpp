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

#include "pose_display_page_object.hpp"

#include <QString>

#include <memory>
#include <vector>

PoseDisplayPageObject::PoseDisplayPageObject()
: BasePageObject(0, "Pose")
{}

void PoseDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void PoseDisplayPageObject::setShape(QString shape)
{
  setComboBox("Shape", shape);
}

void PoseDisplayPageObject::setColor(int red, int green, int blue)
{
  setColorCode("Color", red, green, blue);
}

void PoseDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}

void PoseDisplayPageObject::setShaftLength(float shaft_length)
{
  setFloat("Shaft Length", shaft_length);
}

void PoseDisplayPageObject::setShaftRadius(float shaft_radius)
{
  setFloat("Shaft Radius", shaft_radius);
}

void PoseDisplayPageObject::setHeadLength(float head_length)
{
  setFloat("Head Length", head_length);
}

void PoseDisplayPageObject::setHeadRadius(float head_radius)
{
  setFloat("Head Radius", head_radius);
}

void PoseDisplayPageObject::setAxesLength(float axes_length)
{
  setFloat("Axes Length", axes_length);
}

void PoseDisplayPageObject::setAxesRadius(float axes_radius)
{
  setFloat("Axes Radius", axes_radius);
}
