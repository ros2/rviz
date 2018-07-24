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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
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

#include "transformation_panel.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QButtonGroup>  // NOLINT
#include <QPushButton>  // NOLINT
#include <QString>  // NOLINT
#include <QVBoxLayout>  // NOLINT
#include <QtWidgets>  // NOLINT

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/properties/radio_button_property.hpp"
#include "rviz_common/properties/radio_button_property_group.hpp"
#include "./transformation/transformation_manager.hpp"

namespace rviz_common
{

TransformationPanel::TransformationPanel(QWidget * parent)
: Panel(parent), button_group_(std::make_shared<properties::RadioButtonPropertyGroup>())
{
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(initializeTreeWidget());
  layout->addLayout(initializeBottomButtonRow());
  layout->addStretch(1);
  setLayout(layout);
}

properties::PropertyTreeWidget * TransformationPanel::initializeTreeWidget()
{
  root_property_ = new properties::Property();
  tree_model_ = new properties::PropertyTreeModel(root_property_);
  tree_widget_ = new properties::PropertyTreeWidget();
  tree_widget_->setSelectionMode(QTreeView::NoSelection);
  tree_widget_->setModel(tree_model_);
  connect(
    tree_widget_,
    SIGNAL(clicked(const QModelIndex&)),
    this,
    SLOT(onItemClicked(const QModelIndex&)));

  return tree_widget_;
}

QHBoxLayout * TransformationPanel::initializeBottomButtonRow()
{
  save_button_ = new QPushButton("Save");
  reset_button_ = new QPushButton("Reset");

  connect(save_button_, SIGNAL(clicked()), this, SLOT(onSaveClicked()));
  connect(reset_button_, SIGNAL(clicked()), this, SLOT(onResetClicked()));

  auto button_layout = new QHBoxLayout();
  button_layout->addWidget(save_button_);
  button_layout->addWidget(reset_button_);
  return button_layout;
}

void TransformationPanel::onInitialize()
{
  QStringList available_transformer_names =
    getDisplayContext()->getTransformationManager()->getAvailableTransformerNames();

  for (const auto & transformer_name : available_transformer_names) {
    auto splitted_plugin = transformer_name.split("/");
    auto package_name = splitted_plugin[0];
    auto plugin_name = splitted_plugin[1];

    initializeProperties(package_name, plugin_name);
  }

  updateButtonState();
}

void TransformationPanel::initializeProperties(
  const QString & package_name, const QString & plugin_name)
{
  properties::Property * package_property;

  auto package_property_entry = package_properties_.find(package_name);
  if (package_property_entry != package_properties_.end()) {
    package_property = package_property_entry->second;
  } else {
    package_property = new properties::Property(package_name, QString(), QString(), root_property_);
    package_property->setReadOnly(true);
    package_property->expand();
    package_properties_.insert(
      std::pair<QString, properties::Property *>(package_name, package_property));
  }

  auto radio_button_property = new properties::RadioButtonProperty(
    button_group_, plugin_name, false, QString(), package_property);

  if (isCurrentPlugin(radio_button_property)) {
    radio_button_property->setValue(true);
  }
}

void TransformationPanel::onSaveClicked()
{
  auto property = button_group_->getChecked();
  if (property) {
    getDisplayContext()->getTransformationManager()->setTransformer(
      getClassIdFromProperty(property));
    updateButtonState();
  }
}

void TransformationPanel::onResetClicked()
{
  auto plugin = getDisplayContext()->getTransformationManager()->getCurrentTransformerName();
  auto splitted_plugin = plugin.split("/");
  auto package_name = splitted_plugin[0];
  auto plugin_name = splitted_plugin[1];

  auto package_property = package_properties_.find(package_name);
  if (package_property != package_properties_.end()) {
    package_property->second->subProp(plugin_name)->setValue(true);
  }
  updateButtonState();
}

void TransformationPanel::onItemClicked(const QModelIndex & index)
{
  auto property = dynamic_cast<properties::RadioButtonProperty *>(tree_model_->getProp(index));
  if (property) {
    property->setValue(true);
  }
  updateButtonState();
}

void TransformationPanel::updateButtonState()
{
  auto button = button_group_->getChecked();
  if (button && isCurrentPlugin(button)) {
    save_button_->setEnabled(false);
    reset_button_->setEnabled(false);
  } else {
    save_button_->setEnabled(true);
    reset_button_->setEnabled(true);
  }
}

bool TransformationPanel::isCurrentPlugin(properties::RadioButtonProperty * property)
{
  return getClassIdFromProperty(property) ==
         getDisplayContext()->getTransformationManager()->getCurrentTransformerName();
}

QString TransformationPanel::getClassIdFromProperty(properties::RadioButtonProperty * property)
{
  return property->getParent()->getName() + "/" + property->getName();
}

}  // namespace rviz_common
