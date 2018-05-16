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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__PAGE_OBJECT_WITH_WINDOW_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__PAGE_OBJECT_WITH_WINDOW_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rviz_visual_testing_framework/page_objects/base_page_object.hpp"

#include "rviz_rendering/render_window.hpp"

class PageObjectWithWindow : public BasePageObject
{
public:
  PageObjectWithWindow(int display_category, QString display_name);

  /// Used internally to make a screenshot.
  void captureDisplayRenderWindow(std::string image_name);

protected:
  /**
   * Abstract function: Implement to capture the correct render window and save it to the member
   * variable render_window_. This will then be used to make a screenshot.
   *
   * Use test_helpers::findWindow(window_name) to find the corresponding render window (note that
   * the window needs to have its Qt object_name set to the name of the window).
   */
  virtual void setRenderWindow() = 0;

  rviz_rendering::RenderWindow * render_window_;
  int display_with_window_index_;
};

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__PAGE_OBJECT_WITH_WINDOW_HPP_
