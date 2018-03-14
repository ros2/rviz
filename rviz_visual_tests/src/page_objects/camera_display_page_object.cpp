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

#include <string>

#include <QTest>  // NOLINT

#include "src/internal/test_helpers.hpp"
#include "src/visual_test_fixture.hpp"

CameraDisplayPageObject::CameraDisplayPageObject(int display_id)
: BasePageObject(display_id, 0, 0)
{
  static int camera_displays_number = 0;
  camera_display_index_ = camera_displays_number++;
}

void CameraDisplayPageObject::setVisibilitySubProperty(
  QString name, int relative_row_index, bool sub_property_visibility)
{
  int property_row_index = 1;

  setBool(name, sub_property_visibility, property_row_index, relative_row_index);
}

void CameraDisplayPageObject::setTopic(QString topic)
{
  int property_row_index = 2;

  setComboBox("Topic", topic, property_row_index);
}

void CameraDisplayPageObject::setUnreliable(bool unreliable)
{
  int property_row_index = 3;

  setBool("Unreliable", unreliable, property_row_index);
}

void CameraDisplayPageObject::setQueueSize(QString queue_size)
{
  int property_row_index = 4;

  setString("Queue Size", queue_size, property_row_index);
}

void CameraDisplayPageObject::setImageRendering(QString image_rendering)
{
  int property_row_index = 5;

  setComboBox("Image Rendering", image_rendering, property_row_index);
}

void CameraDisplayPageObject::setOverlayAlpha(QString overlay_alpha)
{
  int property_row_index = 6;

  setString("Overlay Alpha", overlay_alpha, property_row_index);
}

void CameraDisplayPageObject::setZoomFacor(QString zoom_factor)
{
  int property_row_index = 7;

  setString("Zoom Factor", zoom_factor, property_row_index);
}

void CameraDisplayPageObject::setVisibility(bool visibility)
{
  int property_row_index = 1;

  setBool("Visibility", visibility, property_row_index);
}

void CameraDisplayPageObject::captureDisplayRenderWindow(std::string name)
{
  QTimer::singleShot(VisualTestFixture::total_delay_, this, [this, name] {
      auto render_window =
      helpers::findWindow("CameraDisplayRenderWindow" + QString::number(camera_display_index_));
      if (render_window) {
        render_window->captureScreenShot(name);
      }
    });
}
