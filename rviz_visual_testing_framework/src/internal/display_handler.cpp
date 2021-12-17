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

#include "rviz_visual_testing_framework/internal/display_handler.hpp"

#include <memory>
#include <vector>

#include <QTest>  // NOLINT

#include "rviz_visual_testing_framework/test_helpers.hpp"

DisplayHandler::DisplayHandler(
  std::shared_ptr<Executor> executor, std::shared_ptr<std::vector<int>> all_displays_ids)
: executor_(executor)
{
  absolute_displays_number_ = 0;
  all_display_ids_vector_ = all_displays_ids;
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
    std::cout << "[  ERROR  ] the button doesn't exists, make sure the name is correct!\n";
    return nullptr;
  }
}

void DisplayHandler::removeDisplayWithoutDelay(int display_id)
{
  int display_index = helpers::findIndex(display_id, *all_display_ids_vector_);

  if (display_index >= 0) {
    auto remove_display_button = getRemoveDisplayButton();

    auto relative_display_index =
      helpers::getDisplaysTreeView()->model()->index(2 + display_index, 0);
    auto viewport = helpers::getDisplaysTreeView()->viewport();
    auto rect = helpers::getDisplaysTreeView()->visualRect(relative_display_index);
    QTest::mouseClick(
      viewport, Qt::MouseButton::LeftButton, Qt::KeyboardModifiers(),
      rect.center());

    QTest::mouseClick(remove_display_button, Qt::MouseButton::LeftButton);

    all_display_ids_vector_->erase(all_display_ids_vector_->begin() + display_index);
  }
}

void DisplayHandler::removeAllDisplays()
{
  std::vector<int> current_displays = *all_display_ids_vector_;
  if (current_displays.size() > 0) {
    auto remove_display_button = getRemoveDisplayButton();
    removeDisplayWithoutDelay(current_displays[current_displays.size() - 1]);
    for (size_t i = 0; i < current_displays.size() - 1; ++i) {
      QTest::mouseClick(remove_display_button, Qt::MouseButton::LeftButton);
      all_display_ids_vector_->erase(all_display_ids_vector_->end() - 1);
    }
  }
}

void DisplayHandler::removeDisplay(std::shared_ptr<BasePageObject> display)
{
  executor_->queueAction(
    [this, display]() {
      this->removeDisplayWithoutDelay(display->getDisplayId());
    }
  );
}

void DisplayHandler::openAddDisplayDialog()
{
  executor_->queueAction(
    [this]() {
      auto add_display_button = this->getAddDisplayButton();

      QTest::mouseClick(add_display_button, Qt::MouseButton::LeftButton);
    }
  );
}

void DisplayHandler::selectDisplayAndConfirm(std::shared_ptr<BasePageObject> page_object)
{
  executor_->queueAction(
    [page_object]() {
      auto add_display_dialog_window = QApplication::activeWindow();

      auto create_visualization_type_box = add_display_dialog_window->findChild<QWidget *>(
        "AddDisplayDialog/Visualization_Typebox");
      QTabWidget * select_display_tab_widget =
      create_visualization_type_box->findChild<QTabWidget *>("Visualization_Typebox/TabWidget");

      QTreeWidget * add_by_name_tree = qobject_cast<QTreeWidget *>(
        select_display_tab_widget->currentWidget());

      auto items = add_by_name_tree->findItems(page_object->getDisplayName(), Qt::MatchRecursive);

      if (items.empty()) {
        std::cout << "[  ERROR   ] The display with name '" <<
          page_object->getDisplayName().toStdString() << "' does not exist!\n";
        // *INDENT-OFF* (uncrustify 0.72 erroneously wants to remove this return)
        return;
        // *INDENT-ON*
      }

      add_by_name_tree->scrollToItem(items.at(0));
      QRect display_element_bounding_box = add_by_name_tree->visualItemRect(items.at(0));

      QTest::mouseClick(
        add_by_name_tree->viewport(),
        Qt::MouseButton::LeftButton,
        Qt::KeyboardModifiers(),
        display_element_bounding_box.center());

      auto add_display_dialog_buttons = add_display_dialog_window->findChild<QDialogButtonBox *>(
        "AddDisplayDialog/ButtonBox");

      QWidget * ok_button = add_display_dialog_buttons->button(QDialogButtonBox::Ok);
      QTest::mouseClick(ok_button, Qt::MouseButton::LeftButton);
    }
  );
}

void DisplayHandler::addDisplayToIdsVector()
{
  all_display_ids_vector_->push_back(absolute_displays_number_);
}

int DisplayHandler::absolute_displays_number_ = 0;
std::shared_ptr<std::vector<int>> DisplayHandler::all_display_ids_vector_;
