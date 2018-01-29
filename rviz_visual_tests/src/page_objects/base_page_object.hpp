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

#ifndef PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_
#define PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <QString>  // NOLINT
#include <QtWidgets>  // NOLINT

class BasePageObject : public QObject
{
public:
  BasePageObject(int display_id, int display_category, int display_name_index);

  int getDisplayId() const;
  int getDisplayCategory() const;
  int getDisplayNameIndex() const;

protected:
  void setString(
    QString property_name, QString value_to_set, int property_row_index);
  void setComboBox(
    QString property_name, QString value_to_set, int property_row_index);
  void setBool(
    QString property_name, bool value_to_set, int property_row_index);

  int display_id_;
  int display_category_;
  int display_name_index_;

private:
  bool checkPropertyName(
    QString expected_property_name, int property_row_index, QModelIndex display_index);
  void failIfPropertyIsAbsent(
    QString property_name, int property_row_index, QModelIndex relative_display_index);
  void clickOnProperty(QModelIndex property_index);
  QModelIndex getRelativeIndexAndExpand() const;
  QModelIndex getValueToChangeIndex(
    int property_row_index, const QModelIndex & relative_display_index) const;
  QModelIndex getPropertyToChangeIndex(
    int property_row_index, const QModelIndex & relative_display_index) const;

  int default_first_display_index_;
};

QString format(float number);

#endif  // PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_
