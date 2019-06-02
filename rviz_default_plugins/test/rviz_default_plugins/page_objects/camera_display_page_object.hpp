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

#ifndef RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_
#define RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_

#include "rviz_visual_testing_framework/page_objects/page_object_with_window.hpp"

class CameraDisplayPageObject : public PageObjectWithWindow
{
public:
  CameraDisplayPageObject();

  /* @param display_name Name of the display whose visibility must be changed
   * @param relative_row_index Sub-property index relative to the parent property (i.e. Visibility)
   * @param display_visibility the value to set for the sub-property
   */
  void setDisplayVisibilityInRenderWindow(QString display_name, bool display_visibility);
  void setTopic(QString topic);
  void setImageRendering(QString image_rendering);
  void setOverlayAlpha(float overlay_alpha);
  void setZoomFactor(float zoom_factor);
  void setVisibility(bool visibility);

  void setRenderWindow() override;
};

#endif  // RVIZ_DEFAULT_PLUGINS__PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_
