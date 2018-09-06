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

#ifndef RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_
#define RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QPushButton>  // NOLINT
#include <QVBoxLayout>  // NOLINT

#include "rviz_common/panel.hpp"
#include "rviz_common/properties/grouped_checkbox_property_group.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"

namespace rviz_common
{
namespace properties
{
class Property;
class StringProperty;
class PropertyTreeWidget;
}

class DisplayContext;

/// A panel to choose the transformation plugin.
class TransformationPanel : public Panel
{
  Q_OBJECT

public:
  explicit TransformationPanel(QWidget * parent = 0);
  ~TransformationPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onSaveClicked();
  void onItemClicked(const QModelIndex & index);

private:
  void updateButtonState();
  bool isCurrentTransformerProperty(properties::GroupedCheckboxProperty * property);
  bool checkedPropertyIsNotCurrentTransformer();

  properties::PropertyTreeWidget * initializeTreeWidget();
  QHBoxLayout * initializeBottomButtonRow();
  void createProperty(const PluginInfo & transformer_info);
  properties::Property * getOrCreatePackageProperty(const QString & package);

  properties::Property * root_property_;
  properties::PropertyTreeModel * tree_model_;
  properties::PropertyTreeWidget * tree_widget_;

  QPushButton * save_button_;

  std::shared_ptr<properties::GroupedCheckboxPropertyGroup> checkbox_property_group_;

  transformation::TransformationManager * transformation_manager_;

  std::map<properties::Property *, PluginInfo> transformer_property_infos_;

  std::map<QString, properties::Property *> package_properties_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_
