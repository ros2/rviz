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

#include "camera_display_page_object.hpp"

#include <memory>
#include <vector>

#include <QTest>  // NOLINT

#include "rviz_visual_testing_framework/test_helpers.hpp"

CameraDisplayPageObject::CameraDisplayPageObject()
: PageObjectWithWindow(0, "Camera")
{
  static int camera_displays_number = 0;
  display_with_window_index_ = camera_displays_number++;
}

void CameraDisplayPageObject::setDisplayVisibilityInRenderWindow(
  QString display_name, bool display_visibility)
{
  setBool(display_name, display_visibility, {"Visibility"});
}

void CameraDisplayPageObject::setTopic(QString topic)
{
  setComboBox("Topic", topic);
  waitForFirstMessage();
}

void CameraDisplayPageObject::setImageRendering(QString image_rendering)
{
  setComboBox("Image Rendering", image_rendering);
}

void CameraDisplayPageObject::setOverlayAlpha(float overlay_alpha)
{
  setFloat("Overlay Alpha", overlay_alpha);
}

void CameraDisplayPageObject::setZoomFactor(float zoom_factor)
{
  setFloat("Zoom Factor", zoom_factor);
}

void CameraDisplayPageObject::setVisibility(bool visibility)
{
  setBool("Visibility", visibility);
}

void CameraDisplayPageObject::setRenderWindow()
{
  render_window_ =
    helpers::findWindow("CameraDisplayRenderWindow" + QString::number(display_with_window_index_));
}
