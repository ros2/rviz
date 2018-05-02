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
#include "rviz_visual_testing_framework/page_objects/base_page_object.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <QTest>  // NOLINT

#include "rviz_visual_testing_framework/test_helpers.hpp"

BasePageObject::BasePageObject(int display_category, QString display_name)
: display_id_(0),
  display_category_(display_category),
  display_name_(display_name),
  executor_(nullptr),
  default_first_display_index_(2),
  all_display_ids_vector_(nullptr)
{
}

void BasePageObject::initialize(
  int display_id,
  std::shared_ptr<Executor> executor,
  std::shared_ptr<std::vector<int>> all_displays_ids)
{
  display_id_ = display_id;
  executor_ = executor;
  all_display_ids_vector_ = all_displays_ids;
}

void BasePageObject::setString(
  const QString & main_property_name,
  const QString & value_to_set,
  int sub_property_index,
  const QString & sub_property_name)
{
  executor_->queueAction(
    [this, value_to_set, main_property_name, sub_property_index, sub_property_name] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();
      int main_property_row_index =
      findPropertyRowIndexByName(main_property_name, relative_display_index);

      if (main_property_row_index < 0) {
        failForAbsentProperty(main_property_name);
      }
      doubleClickOnTreeItem(
        getPropertyToChangeIndex(main_property_row_index, relative_display_index));

      auto index_to_change = sub_property_index < 0 ?
      getValueToChangeIndex(main_property_row_index, relative_display_index) :
      getSubPropertyIndex(
        sub_property_name, main_property_row_index, sub_property_index, relative_display_index);

      clickOnTreeItem(index_to_change);
      QTest::keyClicks(helpers::getDisplaysTreeView()->focusWidget(), value_to_set);
      QTest::keyClick(helpers::getDisplaysTreeView()->focusWidget(), Qt::Key_Enter);
    }
  );
}

void BasePageObject::setComboBox(const QString & property_name, const QString & value_to_set)
{
  executor_->queueAction([this, value_to_set, property_name] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();

      int property_row_index = findPropertyRowIndexByName(property_name, relative_display_index);
      auto value_to_change_index =
      getMainPropertyIndex(property_name, property_row_index, relative_display_index);

      helpers::getDisplaysTreeView()->setCurrentIndex(value_to_change_index);

      QComboBox * property_combo_box =
      qobject_cast<QComboBox *>(helpers::getDisplaysTreeView()->focusWidget());

      auto value_index = property_combo_box->findText(value_to_set);

      if (value_index == -1) {
        GTEST_FAIL() << "[  ERROR   ]  The value '" << value_to_set.toStdString() << "' does "
        "not exist!";
      }
      property_combo_box->setCurrentIndex(value_index);
    }
  );
}

void BasePageObject::setBool(
  const QString & main_property_name,
  bool value_to_set,
  int sub_property_index,
  const QString & sub_property_name)
{
  executor_->queueAction(
    [this, value_to_set, main_property_name, sub_property_index, sub_property_name] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();
      int property_row_index =
      findPropertyRowIndexByName(main_property_name, relative_display_index);

      auto value_to_change_index = sub_property_index < 0 ?
      getMainPropertyIndex(main_property_name, property_row_index, relative_display_index) :
      getSubPropertyIndex(
        sub_property_name, property_row_index, sub_property_index, relative_display_index);

      auto checked_status = value_to_set ? Qt::Checked : Qt::Unchecked;
      helpers::getDisplaysTreeView()->model()->setData(
        value_to_change_index, checked_status, Qt::CheckStateRole);
    }
  );
}

void BasePageObject::setInt(const QString & property_name, int value_to_set)
{
  setString(property_name, QString::number(value_to_set));
}

void BasePageObject::setFloat(
  const QString & main_property_name,
  float value_to_set,
  int sub_property_index,
  const QString & sub_property_name)
{
  setString(main_property_name, format(value_to_set), sub_property_index, sub_property_name);
}

void BasePageObject::setColorCode(const QString & property_name, int red, int green, int blue)
{
  QString color_code = QString::fromStdString(
    std::to_string(red) + "; " + std::to_string(green) + "; " + std::to_string(blue));

  setString(property_name, color_code);
}

void BasePageObject::setVector(const QString & property_name, float x, float y, float z)
{
  QString formatted_vector = format(x) + "; " + format(y) + "; " + format(z);

  setString(property_name, formatted_vector);
}

QModelIndex BasePageObject::getMainPropertyIndex(
  const QString & property_name, int property_row_index, QModelIndex display_index)
{
  if (property_row_index < 0) {
    failForAbsentProperty(property_name);
    return {};
  }
  return getValueToChangeIndex(property_row_index, display_index);
}

QModelIndex BasePageObject::getSubPropertyIndex(
  QString property_name,
  int main_property_index,
  int sub_property_index,
  QModelIndex display_index)
{
  auto display_tree_view = helpers::getDisplaysTreeView();
  auto relative_property_index = display_tree_view
    ->model()
    ->index(main_property_index, 0, display_index);

  QString actual_property_name = display_tree_view->model()->data(
    display_tree_view->model()->index(sub_property_index, 0, relative_property_index)).toString();

  if (actual_property_name != property_name) {
    sub_property_index = findPropertyRowIndexByName(property_name, relative_property_index);

    if (sub_property_index < 0) {
      failForAbsentProperty(property_name);
      return {};
    }

    std::cout << "[  INFO   ]  The Sub-property '" << property_name.toStdString() << "' is "
      "not where expected. The first row with this name will be modified instead.\n";
  }

  clickOnTreeItem(getPropertyToChangeIndex(sub_property_index, relative_property_index));
  return getValueToChangeIndex(sub_property_index, relative_property_index);
}

QModelIndex BasePageObject::getPropertyToChangeIndex(
  int property_row_index, const QModelIndex & parent_index) const
{
  return helpers::getDisplaysTreeView()
         ->model()
         ->index(property_row_index, 0, parent_index);
}

QModelIndex BasePageObject::getValueToChangeIndex(
  int property_row_index, const QModelIndex & parent_index) const
{
  return helpers::getDisplaysTreeView()
         ->model()
         ->index(property_row_index, 1, parent_index);
}

QModelIndex BasePageObject::getRelativeIndexAndExpandDisplay()
{
  int display_index = helpers::findIndex(display_id_, *all_display_ids_vector_);
  auto relative_display_index =
    helpers::getDisplaysTreeView()->model()->index(default_first_display_index_ + display_index, 0);
  setExpanded(relative_display_index, true);
  return relative_display_index;
}

void BasePageObject::failForAbsentProperty(const QString & property_name)
{
  std::cout << "\n[  ERROR   ]  The Property '" << property_name.toStdString() << "' does not "
    "exist.";

  GTEST_FAIL();
}

void BasePageObject::clickOnTreeItem(QModelIndex item_index) const
{
  auto viewport = helpers::getDisplaysTreeView()->viewport();
  auto item_rect = helpers::getDisplaysTreeView()->visualRect(item_index);
  QTest::mouseClick(viewport, Qt::MouseButton::LeftButton, Qt::NoModifier, item_rect.center());
}

void BasePageObject::doubleClickOnTreeItem(QModelIndex item_index) const
{
  auto viewport = helpers::getDisplaysTreeView()->viewport();
  auto item_rect = helpers::getDisplaysTreeView()->visualRect(item_index);
  // TODO(botteroa-si): for some reason, QTest::mouseDClick() doesn't perform as expected (i.e.
  // simulating a double click). Neither is this result achieved with two consecutive
  // QTest::mouseClick(). With two consecutive mouseDClick() it works. Update this if/when the
  // issue with mouseDClick() is fixed.
  QTest::mouseDClick(viewport, Qt::MouseButton::LeftButton, Qt::NoModifier, item_rect.center());
  QTest::mouseDClick(viewport, Qt::MouseButton::LeftButton, Qt::NoModifier, item_rect.center());
}

int BasePageObject::getDisplayId() const
{
  return display_id_;
}

int BasePageObject::getDisplayCategory() const
{
  return display_category_;
}

QString BasePageObject::getDisplayName() const
{
  return display_name_;
}

void BasePageObject::collapse()
{
  executor_->queueAction([this] {
      int display_index =
      helpers::findIndex(display_id_, *all_display_ids_vector_);
      auto relative_display_index = helpers::getDisplaysTreeView()->model()->index(
        default_first_display_index_ + display_index, 0);
      setExpanded(relative_display_index, false);
    }
  );
}

void BasePageObject::setExpanded(QModelIndex display_index, bool expanded)
{
  if (expanded == helpers::getDisplaysTreeView()->isExpanded(display_index)) {
    return;
  }

  doubleClickOnTreeItem(display_index);
}

int BasePageObject::findPropertyRowIndexByName(
  const QString & property_name, QModelIndex relative_display_index)
{
  auto display_tree_model = helpers::getDisplaysTreeView()->model();
  auto rows_number = display_tree_model->rowCount(relative_display_index);

  for (int i = 0; i < rows_number; ++i) {
    QString current_row_property = display_tree_model->data(
      display_tree_model->index(i, 0, relative_display_index)).toString();
    if (current_row_property == property_name) {
      return i;
    }
  }

  return -1;
}

void BasePageObject::waitForFirstMessage()
{
  executor_->wait(1500);
}

QString format(float number)
{
  return QLocale().toString(number);
}
