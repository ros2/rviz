/*
 * Copyright (c) 2024, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
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

#include "camera_info_display_page_object.hpp"

CameraInfoDisplayPageObject::CameraInfoDisplayPageObject()
: BasePageObject(0, "CameraInfo")
{
}

void CameraInfoDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void CameraInfoDisplayPageObject::setFarClip(float far_clip)
{
  setFloat("Far clip", far_clip);
}

void CameraInfoDisplayPageObject::setShowEdged(bool show_edges)
{
  setBool("Show edges", show_edges);
}

void CameraInfoDisplayPageObject::setShowPolygon(bool show_polygon)
{
  setBool("Show polygons", show_polygon);
}

void CameraInfoDisplayPageObject::setNotShowSidePolygon(bool now_show_side_polygon)
{
  setBool("Not show side polygons", now_show_side_polygon);
}

void CameraInfoDisplayPageObject::setColor(int r, int g, int b)
{
  setColorCode("Color", r, g, b);
}

void CameraInfoDisplayPageObject::setEdgeColor(int r, int g, int b)
{
  setColorCode("Edge color", r, g, b);
}

void CameraInfoDisplayPageObject::setAlpha(float alpha)
{
  setFloat("Alpha", alpha);
}
