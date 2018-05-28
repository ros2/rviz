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
  /**
   * Constructor of a BasePageObject.
   * N.B: When deriving from this class, you need to specify a zero argument constructor setting
   * the parameters in this constructor explicitly so that the "addDisplay" method in
   * VisualTestVixture can work correctly.
   * @param display_category The display category of this display. This is the number of the
   * folder in the "Add Display" dialog (0 for rviz_default_plugins)
   * @param display_name_ The name of the display in the "Add Display" dialog
   */
  BasePageObject(int display_category, QString display_name_);

  void initialize(
    int display_id,
    std::shared_ptr<Executor> executor,
    std::shared_ptr<std::vector<int>> all_displays_ids);

  int getDisplayId() const;
  int getDisplayCategory() const;
  QString getDisplayName() const;
  void collapse();

protected:
  int findPropertyRowIndexByName(
    const QString & property_name, QModelIndex relative_display_index);

  /**
   * Set a String into a property in the displays panel
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param value_to_set String to set for this property
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setString(
    const QString & property_to_change,
    const QString & value_to_set,
    std::vector<QString> super_properties = {});

  /**
   * Set an item of a ComboBox property in the display panel
   * @param main_property_name Name of the Property in the left row of the displays panel
   * @param value_to_set String representation of the value to set for this property
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setComboBox(
    const QString & property_to_change,
    const QString & value_to_set,
    std::vector<QString> super_properties = {});

  /**
   * Set a boolean property (represented by a checkbox in the display panel)
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param value_to_set Boolean value to set for this property
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setBool(
    const QString & property_to_change,
    bool value_to_set,
    std::vector<QString> super_properties = {});

  /**
   * Set an integer property in the display panel
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param value_to_set Integer to set for this property
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setInt(
    const QString & property_to_change,
    int value_to_set,
    std::vector<QString> super_properties = {});

  /**
   * Set a float property in the display panel
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param value_to_set Float to set for this property. Locales will be handled by the visual test
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setFloat(
    const QString & property_to_change,
    float value_to_set,
    std::initializer_list<QString> super_properties = {});

  /**
   * Set a color property in the display panel
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param red, blue, green Color values in rgb (between 0 and 255) to set for this property
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setColorCode(
    const QString & property_to_change,
    int red, int green, int blue,
    std::vector<QString> super_properties = {});

  /**
   * Set a vector property in the display panel
   * @param property_to_change Name of the Property in the left row of the displays panel
   * @param x, y, z coordinates of the Vector to set into this property as float values
   * @param super_properties Specifies all parent properties of property_to_change starting
   * with the root property below the display's property. Empty if the property is a top-level
   * property.
   */
  void setVector(
    const QString & property_to_change,
    float x, float y, float z,
    std::vector<QString> super_properties = {});

  void waitForFirstMessage();

  int display_id_;
  int display_category_;
  QString display_name_;
  std::shared_ptr<Executor> executor_;

private:
  QModelIndex getValueToChangeFromAllProperties(
    const QString & property_to_change, std::vector<QString> super_properties);
  QModelIndex findSubPropertyParentIndex(
    const std::vector<QString> & super_properties, QModelIndex parent_index);
  QModelIndex getRelativeIndexAndExpandDisplay();
  QModelIndex getValueToChangeIndex(
    int property_row_index, const QModelIndex & parent_index) const;
  QModelIndex getPropertyToChangeIndex(
    int property_row_index, const QModelIndex & parent_index) const;

  void failForAbsentProperty(const QString & property_name);
  void clickOnTreeItem(QModelIndex item_index) const;
  void doubleClickOnTreeItem(QModelIndex item_index) const;

  void setExpanded(QModelIndex display_index, bool expanded);

  int default_first_display_index_;
  std::shared_ptr<std::vector<int>> all_display_ids_vector_;
};

QString format(float number);

#endif  // RVIZ_VISUAL_TESTING_FRAMEWORK__PAGE_OBJECTS__BASE_PAGE_OBJECT_HPP_
