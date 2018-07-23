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

#ifndef RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_
#define RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_

#include <vector>

#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QtWidgets/QGroupBox>

#include "rviz_common/panel.hpp"

namespace rviz_common
{
namespace properties
{
class PropertyTreeWidget;
}

class DisplayContext;

/** A place to choose the transformation plugin
 */
class TransformationPanel : public Panel
{
  Q_OBJECT

public:
  explicit TransformationPanel(QWidget * parent = 0);
  ~TransformationPanel() override = default;

  void onInitialize() override;

private Q_SLOTS:
  void onSaveClicked();
  void onResetClicked();
  void onToggled(bool checked);

private:
  QHBoxLayout * initializeBottomButtonRow();
  QGroupBox * initializeRadioButtonGroup();

  QRadioButton * getCheckedRadioButton();
  void updateButtonState();

  std::string current_selection_;  // TODO Temporary, replace with TransformationManager

  QVBoxLayout *  radio_layout_;
  std::vector<QRadioButton *> radio_buttons_;

  QPushButton * save_button_;
  QPushButton * reset_button_;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION_PANEL_HPP_
