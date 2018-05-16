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


#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__DISPLAY_HANDLER_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__DISPLAY_HANDLER_HPP_

#include <memory>
#include <vector>

#include "rviz_visual_testing_framework/page_objects/base_page_object.hpp"
#include "rviz_visual_testing_framework/internal/executor.hpp"

class DisplayHandler
{
public:
  DisplayHandler(
    std::shared_ptr<Executor> executor, std::shared_ptr<std::vector<int>> all_displays_ids);
  template<typename T>
  std::shared_ptr<T> addDisplay()
  {
    auto page_object = std::make_shared<T>();
    page_object->initialize(absolute_displays_number_, executor_, all_display_ids_vector_);

    openAddDisplayDialog();
    selectDisplayAndConfirm(page_object);

    addDisplayToIdsVector();
    absolute_displays_number_++;

    return page_object;
  }

  void removeAllDisplays();
  void removeDisplay(std::shared_ptr<BasePageObject> display);

private:
  static QPushButton * getDisplayActionButton(QString button_name);
  static QPushButton * getAddDisplayButton();
  static QPushButton * getRemoveDisplayButton();
  static void removeDisplayWithoutDelay(int display_id);
  void openAddDisplayDialog();
  void selectDisplayAndConfirm(std::shared_ptr<BasePageObject> page_object);
  void addDisplayToIdsVector();

  static int absolute_displays_number_;
  std::shared_ptr<Executor> executor_;
  static std::shared_ptr<std::vector<int>> all_display_ids_vector_;
};

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__INTERNAL__DISPLAY_HANDLER_HPP_
