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

#include "transformation_panel.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <QPushButton>  // NOLINT
#include <QString>  // NOLINT
#include <QVBoxLayout>  // NOLINT
#include <QtWidgets>  // NOLINT

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/grouped_checkbox_property.hpp"
#include "rviz_common/properties/grouped_checkbox_property_group.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/transformation/transformation_manager.hpp"

namespace rviz_common
{

TransformationPanel::TransformationPanel(QWidget * parent)
: Panel(parent),
  checkbox_property_group_(std::make_shared<properties::GroupedCheckboxPropertyGroup>()),
  transformation_manager_(nullptr)
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
  tree_widget_->setFocusPolicy(Qt::NoFocus);
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

  connect(save_button_, SIGNAL(clicked()), this, SLOT(onSaveClicked()));

  auto button_layout = new QHBoxLayout();
  button_layout->addWidget(save_button_);
  return button_layout;
}

void TransformationPanel::onInitialize()
{
  transformation_manager_ = getDisplayContext()->getTransformationManager();

  auto available_transformers = transformation_manager_->getAvailableTransformers();
  for (const auto & transformer_info : available_transformers) {
    createProperty(transformer_info);
  }

  updateButtonState();
}

void TransformationPanel::createProperty(const PluginInfo & transformer_info)
{
  properties::Property * package_property = getOrCreatePackageProperty(transformer_info.package);

  auto transformer_property = new properties::GroupedCheckboxProperty(
    checkbox_property_group_, transformer_info.name, false, QString(), package_property);
  transformer_property_infos_.insert(std::make_pair(transformer_property, transformer_info));

  if (isCurrentTransformerProperty(transformer_property)) {
    transformer_property->checkPropertyInGroup();
  }
}

properties::Property * TransformationPanel::getOrCreatePackageProperty(const QString & package)
{
  auto package_property_entry = package_properties_.find(package);
  if (package_property_entry != package_properties_.end()) {
    return package_property_entry->second;
  } else {
    auto package_property = new properties::Property(package, QString(), QString(), root_property_);

    package_property->setReadOnly(true);
    package_property->expand();

    package_properties_.insert(std::make_pair(package, package_property));

    return package_property;
  }
}

void TransformationPanel::onSaveClicked()
{
  auto property = checkbox_property_group_->getChecked();
  if (property) {
    transformation_manager_->setTransformer(transformer_property_infos_[property]);
    updateButtonState();
  }
}

void TransformationPanel::onItemClicked(const QModelIndex & index)
{
  auto property = dynamic_cast<properties::GroupedCheckboxProperty *>(tree_model_->getProp(index));
  if (property) {
    property->checkPropertyInGroup();
  }
  updateButtonState();
}

void TransformationPanel::updateButtonState()
{
  save_button_->setEnabled(checkedPropertyIsNotCurrentTransformer());
}

bool TransformationPanel::checkedPropertyIsNotCurrentTransformer()
{
  auto checked_property = checkbox_property_group_->getChecked();
  return !(checked_property && isCurrentTransformerProperty(checked_property));
}

bool TransformationPanel::isCurrentTransformerProperty(
  properties::GroupedCheckboxProperty * property)
{
  auto transformer_info = transformation_manager_->getCurrentTransformerInfo();
  return transformer_property_infos_[property] == transformer_info;
}

}  // namespace rviz_common
