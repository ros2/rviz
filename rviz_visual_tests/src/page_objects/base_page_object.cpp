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
#include "rviz_visual_tests/page_objects/base_page_object.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <QTest>  // NOLINT

#include "rviz_visual_tests/internal/test_helpers.hpp"

BasePageObject::BasePageObject(
  int display_id,
  int display_category,
  int display_name_index,
  std::shared_ptr<Executor> executor,
  std::shared_ptr<std::vector<int>> display_ids_vector)
: display_category_(display_category),
  display_name_index_(display_name_index),
  executor_(executor),
  default_first_display_index_(2),
  all_display_ids_vector_(display_ids_vector)
{
  display_id_ = display_id;
}

void BasePageObject::setString(
  QString property_name, QString value_to_set, int property_row_index)
{
  executor_->queueAction([this, value_to_set, property_row_index, property_name] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();

      failIfPropertyIsAbsent(property_name, property_row_index, relative_display_index);

      clickOnTreeItem(getPropertyToChangeIndex(property_row_index, relative_display_index));
      clickOnTreeItem(getValueToChangeIndex(property_row_index, relative_display_index));
      QTest::keyClicks(helpers::getDisplaysTreeView()->focusWidget(), value_to_set);
      QTest::keyClick(helpers::getDisplaysTreeView()->focusWidget(), Qt::Key_Enter);
    }
  );
}

void BasePageObject::setComboBox(
  QString property_name, QString value_to_set, int property_row_index)
{
  executor_->queueAction([this, value_to_set, property_row_index, property_name] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();

      auto value_to_change_index =
      getMainPropertyIndex(property_name, property_row_index, relative_display_index);

      helpers::getDisplaysTreeView()->setCurrentIndex(value_to_change_index);

      QComboBox * property_combo_box =
      qobject_cast<QComboBox *>(helpers::getDisplaysTreeView()->focusWidget());

      auto style_index = property_combo_box->findText(value_to_set);

      if (style_index == -1) {
        GTEST_FAIL() << "\n[  ERROR   ]  The given property value does not exist!\n";
      }
      property_combo_box->setCurrentIndex(style_index);
    }
  );
}

void BasePageObject::setBool(
  QString property_name, bool value_to_set, int property_row_index, int sub_property_index)
{
  executor_->queueAction(
    [this, value_to_set, property_row_index, property_name, sub_property_index] {
      auto relative_display_index = getRelativeIndexAndExpandDisplay();

      auto value_to_change_index = sub_property_index < 0 ?
      getMainPropertyIndex(property_name, property_row_index, relative_display_index) :
      getSubPropertyIndex(
        property_name, property_row_index, sub_property_index, relative_display_index);

      auto checked_status = value_to_set ? Qt::Checked : Qt::Unchecked;
      helpers::getDisplaysTreeView()->model()->setData(
        value_to_change_index, checked_status, Qt::CheckStateRole);
    }
  );
}

QModelIndex BasePageObject::getMainPropertyIndex(
  QString property_name, int property_row_index, QModelIndex display_index)
{
  failIfPropertyIsAbsent(property_name, property_row_index, display_index);
  return getValueToChangeIndex(property_row_index, display_index);
}

QModelIndex BasePageObject::getSubPropertyIndex(
  QString property_name,
  int main_property_index,
  int sub_property_index,
  QModelIndex display_index)
{
  auto relative_property_index = helpers::getDisplaysTreeView()
    ->model()
    ->index(main_property_index, 0, display_index);
  failIfPropertyIsAbsent(property_name, sub_property_index, relative_property_index);

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

bool BasePageObject::checkPropertyName(
  QString expected_property_name, int property_row_index, QModelIndex relative_display_index)
{
  auto display_tree_model = helpers::getDisplaysTreeView()->model();
  QString actual_property_name =
    display_tree_model->data(
    display_tree_model->index(property_row_index, 0, relative_display_index)).toString();

  if (expected_property_name != actual_property_name) {
    std::cout << "\n[  ERROR   ]  The seleced property is not where supposed to: expected "
      "property = " << expected_property_name.toStdString() << ", actual property = " <<
      actual_property_name.toStdString() << ".\n";
    return false;
  }

  return true;
}

void BasePageObject::failIfPropertyIsAbsent(
  QString property_name, int property_row_index, QModelIndex parent_index)
{
  if (!checkPropertyName(property_name, property_row_index, parent_index)) {
    GTEST_FAIL();
  }
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

int BasePageObject::getDisplayNameIndex() const
{
  return display_name_index_;
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

QString format(float number)
{
  return QLocale(QLocale::Language::English).toString(number);
}
