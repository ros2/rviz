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

#include "rviz_visual_testing_framework/test_helpers.hpp"

#include <vector>

#include <QApplication>  // NOLINT

namespace helpers
{

int findIndex(int display_id, const std::vector<int> & displays_ids_vector)
{
  auto iterator_to_id = std::find(
    displays_ids_vector.begin(), displays_ids_vector.end(), display_id);

  return iterator_to_id != displays_ids_vector.end() ?
         static_cast<int>(std::distance(displays_ids_vector.begin(), iterator_to_id)) :
         -1;
}

QTreeView * getDisplaysTreeView()
{
  return QApplication::activeWindow()
         ->findChild<QWidget *>("Displays")
         ->findChild<QWidget *>("Displays/DisplayPanel")
         ->findChild<QWidget *>("DisplayPanel/TreeWithHelp")
         ->findChild<QTreeView *>("TreeWithHelp/PropertyTree");
}

rviz_rendering::RenderWindow * findWindow(const QString & window_name)
{
  auto all_windows = QApplication::allWindows();
  for (auto & window : all_windows) {
    if (window->objectName() == window_name) {
      rviz_rendering::RenderWindow * render_window =
        qobject_cast<rviz_rendering::RenderWindow *>(window);
      return render_window;
    }
  }
  return nullptr;
}

}  // namespace helpers
