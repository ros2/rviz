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

#ifndef RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_
#define RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_

#include <gtest/gtest.h>

#include <string>
#include <memory>
#include <vector>

#include <QString>  // NOLINT
#include <QtWidgets>  // NOLINT

#include "rviz_visual_testing_framework/internal/executor.hpp"

class BasePageObject : public QObject
{
public:
  BasePageObject(
    int display_id,
    int display_category,
    QString display_name_,
    std::shared_ptr<Executor> executor,
    std::shared_ptr<std::vector<int>> all_displays_ids);

  int getDisplayId() const;
  int getDisplayCategory() const;
  QString getDisplayName() const;
  void collapse();

protected:
  void setString(QString property_name, QString value_to_set, int property_row_index);
  void setComboBox(QString property_name, QString value_to_set, int property_row_index);
  void setBool(
    QString property_name, bool value_to_set, int property_row_index, int sub_property_index = -1);

  int display_id_;
  int display_category_;
  QString display_name_;
  std::shared_ptr<Executor> executor_;

private:
  bool checkPropertyName(
    QString expected_property_name, int property_row_index, QModelIndex display_index);
  void failIfPropertyIsAbsent(
    QString property_name, int property_row_index, QModelIndex parent_index);
  void clickOnTreeItem(QModelIndex item_index) const;
  void doubleClickOnTreeItem(QModelIndex item_index) const;
  QModelIndex getRelativeIndexAndExpandDisplay();
  QModelIndex getValueToChangeIndex(
    int property_row_index, const QModelIndex & parent_index) const;
  QModelIndex getPropertyToChangeIndex(
    int property_row_index, const QModelIndex & parent_index) const;
  QModelIndex getMainPropertyIndex(
    QString property_name, int property_row_index, QModelIndex display_index);
  QModelIndex getSubPropertyIndex(
    QString property_name,
    int main_property_index,
    int sub_property_index,
    QModelIndex display_index);
  void setExpanded(QModelIndex display_index, bool expanded);

  int default_first_display_index_;
  std::shared_ptr<std::vector<int>> all_display_ids_vector_;
};

QString format(float number);

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_
