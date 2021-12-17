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

#include <memory>
#include <string>
#include <utility>
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
  const QString & property_to_change,
  const QString & value_to_set,
  std::vector<QString> super_properties)
{
  executor_->queueAction(
    [this, value_to_set, property_to_change, super_properties{std::move(super_properties)}] {
      auto index_to_change =
      getValueToChangeFromAllProperties(property_to_change, super_properties);

      clickOnTreeItem(index_to_change);
      QTest::keyClicks(helpers::getDisplaysTreeView()->focusWidget(), value_to_set);
      QTest::keyClick(helpers::getDisplaysTreeView()->focusWidget(), Qt::Key_Enter);
    }
  );
}

void BasePageObject::setComboBox(
  const QString & property_to_change,
  const QString & value_to_set,
  std::vector<QString> super_properties)
{
  executor_->queueAction(
    [this, value_to_set, property_to_change,
    super_properties{std::move(super_properties)}] {
      auto index_to_change =
      getValueToChangeFromAllProperties(property_to_change, super_properties);

      helpers::getDisplaysTreeView()->setCurrentIndex(index_to_change);

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
  const QString & property_to_change,
  bool value_to_set,
  std::vector<QString> super_properties)
{
  executor_->queueAction(
    [this, value_to_set, property_to_change, super_properties{std::move(super_properties)}] {
      auto index_to_change =
      getValueToChangeFromAllProperties(property_to_change, super_properties);

      auto checked_status = value_to_set ? Qt::Checked : Qt::Unchecked;
      helpers::getDisplaysTreeView()->model()->setData(
        index_to_change, checked_status, Qt::CheckStateRole);
    }
  );
}

void BasePageObject::setInt(
  const QString & property_to_change, int value_to_set, std::vector<QString> super_properties)
{
  setString(property_to_change, QString::number(value_to_set), super_properties);
}

void BasePageObject::setFloat(
  const QString & property_to_change,
  float value_to_set,
  std::initializer_list<QString> super_properties)
{
  setString(property_to_change, format(value_to_set), super_properties);
}

void BasePageObject::setColorCode(
  const QString & property_to_change,
  int red, int green, int blue,
  std::vector<QString> super_properties)
{
  QString color_code = QString::fromStdString(
    std::to_string(red) + "; " + std::to_string(green) + "; " + std::to_string(blue));

  setString(property_to_change, color_code, super_properties);
}

void BasePageObject::setVector(
  const QString & property_to_change,
  float x, float y, float z,
  std::vector<QString> super_properties)
{
  QString formatted_vector = format(x) + "; " + format(y) + "; " + format(z);

  setString(property_to_change, formatted_vector, super_properties);
}

QModelIndex BasePageObject::getValueToChangeFromAllProperties(
  const QString & property_to_change, std::vector<QString> super_properties)
{
  auto parent_index = super_properties.empty() ?
    getRelativeIndexAndExpandDisplay() :
    findSubPropertyParentIndex(super_properties, getRelativeIndexAndExpandDisplay());

  int property_row_index = findPropertyRowIndexByName(property_to_change, parent_index);

  if (property_row_index < 0) {
    failForAbsentProperty(property_to_change);
  }

  auto index_to_change = getValueToChangeIndex(property_row_index, parent_index);

  // We first need to click on the property name and then on the property to change it.
  // This is somehow necessary on Linux.
  clickOnTreeItem(getPropertyToChangeIndex(property_row_index, parent_index));
  return index_to_change;
}

QModelIndex BasePageObject::findSubPropertyParentIndex(
  const std::vector<QString> & super_properties, QModelIndex parent_index)
{
  for (const auto & property : super_properties) {
    int property_row_index =
      findPropertyRowIndexByName(property, parent_index);

    if (property_row_index < 0) {
      failForAbsentProperty(property);
    }

    parent_index = getPropertyToChangeIndex(property_row_index, parent_index);
    setExpanded(parent_index, true);
  }
  return parent_index;
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
  // For some reason, QTest::mouseDClick() doesn't perform as expected (i.e.
  // simulating a double click). Neither is this result achieved with two consecutive
  // QTest::mouseClick(). With two consecutive mouseDClick() it works.
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
  executor_->queueAction(
    [this] {
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
