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

#include "display_handler.hpp"

#include <memory>
#include <vector>

#include <QTest>  // NOLINT
#include <QTimer>  // NOLINT

#include "test_helpers.hpp"
#include "src/visual_test_fixture.hpp"

DisplayHandler::DisplayHandler()
{
  absolute_displays_number_ = 0;
}

QPushButton * DisplayHandler::getAddDisplayButton()
{
  return getDisplayActionButton("Add");
}

QPushButton * DisplayHandler::getRemoveDisplayButton()
{
  return getDisplayActionButton("Remove");
}

QPushButton * DisplayHandler::getDisplayActionButton(QString button_name)
{
  auto visualization_frame = QApplication::activeWindow();

  auto displays = visualization_frame->findChild<QWidget *>("Displays");
  auto display_panel = displays->findChild<QWidget *>("Displays/DisplayPanel");

  if (button_name == "Add") {
    return display_panel->findChild<QPushButton *>(
      "DisplayPanel/AddDisplayButton");
  } else if (button_name == "Remove") {
    return display_panel->findChild<QPushButton *>(
      "DisplayPanel/RemoveDisplayButton");
  } else if (button_name == "Duplicate") {
    return display_panel->findChild<QPushButton *>(
      "DisplayPanel/DuplicateDisplayButton");
  } else if (button_name == "Rename") {
    return display_panel->findChild<QPushButton *>(
      "DisplayPanel/RenameDisplayButton");
  } else {
    std::cout << "\n[  ERROR  ] the button doesn't exists, make sure the name is correct!\n";
    return nullptr;
  }
}

void DisplayHandler::removeDisplayWithoutDelay(int display_id)
{
  int display_index = helpers::findIndex(display_id, VisualTestFixture::all_display_ids_vector_);

  if (display_index >= 0) {
    auto remove_display_button = getRemoveDisplayButton();

    auto relative_display_index =
      helpers::getDisplaysTreeView()->model()->index(3 + display_index, 0);
    auto viewport = helpers::getDisplaysTreeView()->viewport();
    auto rect = helpers::getDisplaysTreeView()->visualRect(relative_display_index);
    QTest::mouseClick(
      viewport, Qt::MouseButton::LeftButton, Qt::KeyboardModifiers(),
      rect.center());

    QTest::mouseClick(remove_display_button, Qt::MouseButton::LeftButton);

    VisualTestFixture::all_display_ids_vector_.erase(
      VisualTestFixture::all_display_ids_vector_.begin() + display_index);
  }
}

void DisplayHandler::removeAllDisplays()
{
  std::vector<int> current_displays = VisualTestFixture::all_display_ids_vector_;
  if (current_displays.size() > 0) {
    auto remove_display_button = getRemoveDisplayButton();
    removeDisplayWithoutDelay(current_displays[current_displays.size() - 1]);
    for (size_t i = 0; i < current_displays.size() - 1; ++i) {
      QTest::mouseClick(remove_display_button, Qt::MouseButton::LeftButton);
      VisualTestFixture::all_display_ids_vector_.erase(
        VisualTestFixture::all_display_ids_vector_.end() - 1);
    }
  }
}

void DisplayHandler::removeDisplay(std::shared_ptr<BasePageObject> display)
{
  QTimer::singleShot(VisualTestFixture::total_delay_, this, [this, display]() {
      removeDisplayWithoutDelay(display->getDisplayId());
    });

  helpers::increaseTotalDelay();
}

void DisplayHandler::openAddDisplayDialog()
{
  QTimer::singleShot(VisualTestFixture::total_delay_, this, [this]() {
      auto add_display_button = getAddDisplayButton();

      QTest::mouseClick(add_display_button, Qt::MouseButton::LeftButton);
    });

  helpers::increaseTotalDelay();
}

void DisplayHandler::selectDisplayAndConfirm(std::shared_ptr<BasePageObject> page_object)
{
  QTimer::singleShot(VisualTestFixture::total_delay_, this, [this, page_object]() {
      auto add_display_dialog_window = QApplication::activeWindow();

      auto create_visualization_type_box = add_display_dialog_window->findChild<QWidget *>(
        "AddDisplayDialog/Visualization_Typebox");
      QTabWidget * select_display_tab_widget =
      create_visualization_type_box->findChild<QTabWidget *>("Visualization_Typebox/TabWidget");

      QTreeWidget * add_by_name_tree = qobject_cast<QTreeWidget *>(
        select_display_tab_widget->currentWidget());

      // N.B. when updating to a new master, the position of the various displays in the QWidgetTree
      // may be changed. After updating check it.
      QTreeWidgetItem * display =
      add_by_name_tree
      ->topLevelItem(page_object->getDisplayCategory())
      ->child(page_object->getDisplayNameIndex());

      QRect display_element_bounding_box = add_by_name_tree->visualItemRect(display);

      QTest::mouseClick(
        add_by_name_tree->viewport(),
        Qt::MouseButton::LeftButton,
        Qt::KeyboardModifiers(),
        display_element_bounding_box.center());

      auto add_display_dialog_buttons = add_display_dialog_window->findChild<QDialogButtonBox *>(
        "AddDisplayDialog/ButtonBox");

      QWidget * ok_button = add_display_dialog_buttons->button(QDialogButtonBox::Ok);
      QTest::mouseClick(ok_button, Qt::MouseButton::LeftButton);
    });

  helpers::increaseTotalDelay();
}

void DisplayHandler::addDisplayToIdsVector()
{
  VisualTestFixture::all_display_ids_vector_.push_back(absolute_displays_number_);
}

int DisplayHandler::absolute_displays_number_ = 0;
