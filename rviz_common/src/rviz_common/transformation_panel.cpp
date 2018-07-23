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

#include <string>
#include <vector>

#include <QPushButton>
#include <QGroupBox>
#include <QRadioButton>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"

namespace rviz_common
{

TransformationPanel::TransformationPanel(QWidget * parent)
  : Panel(parent)
{
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(initializeRadioButtonGroup());
  layout->addLayout(initializeBottomButtonRow());
  layout->addStretch(1);
  setLayout(layout);
}

QGroupBox * TransformationPanel::initializeRadioButtonGroup()
{
  auto group_box = new QGroupBox("Available plugins");
  radio_layout_ = new QVBoxLayout();
  radio_layout_->addStretch(1);
  group_box->setLayout(radio_layout_);
  return group_box;
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
  current_selection_ = "tf2";
// TODO   getDisplayContext()->getFrameManager()->getCurrentTransformationPlugin();

  std::vector<std::string> transformation_plugins {"tf2", "TARDIS"};
// TODO   getDisplayContext()->getFrameManager()->getTransformationPlugins();

  for(const auto & plugin : transformation_plugins) {
    auto button = new QRadioButton(QString::fromStdString(plugin));
    connect(button, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));

    if (plugin == current_selection_) {
      button->setChecked(true);
    }

    radio_buttons_.push_back(button);
    radio_layout_->addWidget(button);
  }

  updateButtonState();
}

void TransformationPanel::onSaveClicked()
{
  auto checked_button = getCheckedRadioButton();
  if (checked_button) {
    auto text = checked_button->text().toStdString();

    // TODO getDisplayContext()->getTransformationManager()->setPlugin(text)
    current_selection_ = text;
    updateButtonState();
  }
}

void TransformationPanel::onResetClicked()
{
  for (auto button : radio_buttons_) {
    if (button->text().toStdString() == current_selection_) {
      button->setChecked(true);
    }
  }
}

void TransformationPanel::onToggled(bool checked)
{
  if (checked) {
    updateButtonState();
  }
}

void TransformationPanel::updateButtonState()
{
  auto button = getCheckedRadioButton();
  if (button && button->text().toStdString() == current_selection_) {
    save_button_->setEnabled(false);
    reset_button_->setEnabled(false);
  } else {
    save_button_->setEnabled(true);
    reset_button_->setEnabled(true);
  }
}

QRadioButton * TransformationPanel::getCheckedRadioButton()
{
  for (auto button : radio_buttons_) {
    if(button->isChecked()) {
      return button;
    }
  }
  return nullptr;
}

}  // namespace rviz_common
