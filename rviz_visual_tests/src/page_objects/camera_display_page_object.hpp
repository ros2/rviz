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

#ifndef PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_
#define PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_

#include <string>

#include "src/page_objects/base_page_object.hpp"

class CameraDisplayPageObject : public BasePageObject
{
public:
  explicit CameraDisplayPageObject(int display_id);

  /* @param name Name of the sub-property to change (i.e. one of the other displays currently used)
   * @param relative_row_index Sub-property index relative to the parent property (i.e. Visibility)
   * @param sub_property_visibility the value to set for the sub-property
   */
  void setVisibilitySubProperty(QString name, int relative_row_index, bool sub_property_visibility);
  void setTopic(QString topic);
  void setUnreliable(bool unreliable);
  void setQueueSize(QString queue_size);
  void setImageRendering(QString image_rendering);
  void setOverlayAlpha(QString overlay_alpha);
  void setZoomFacor(QString zoom_factor);
  void setVisibility(bool visibility);

  void captureDisplayRenderWindow(std::string name) override;

private:
  int camera_display_index_;
};

#endif  // PAGE_OBJECTS__CAMERA_DISPLAY_PAGE_OBJECT_HPP_
