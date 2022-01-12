/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#include "displays_panel.hpp"

#include <string>

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here
#include <QHBoxLayout>  // NOLINT: cpplint is unable to handle the include order here
#include <QInputDialog>  // NOLINT: cpplint is unable to handle the include order here
#include <QProgressDialog> // NOLINT: cpplint is unable to handle the include order here
#include <QPushButton>  // NOLINT: cpplint is unable to handle the include order here
#include <QTimer>  // NOLINT: cpplint is unable to handle the include order here
#include <QVBoxLayout>  // NOLINT: cpplint is unable to handle the include order here

#include "display_factory.hpp"
#include "rviz_common/display.hpp"
#include "add_display_dialog.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/properties/property_tree_with_help.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace rviz_common
{

DisplaysPanel::DisplaysPanel(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
  VisualizationManager * manager,
  QWidget * parent)
: Panel(parent), vis_manager_(manager), rviz_ros_node_(rviz_ros_node)
{
  setObjectName("Displays/DisplayPanel");
  tree_with_help_ = new properties::PropertyTreeWithHelp;
  tree_with_help_->setObjectName("DisplayPanel/TreeWithHelp");
  property_grid_ = tree_with_help_->getTree();

  QPushButton * add_button = new QPushButton("Add");
  add_button->setObjectName("DisplayPanel/AddDisplayButton");
  add_button->setShortcut(QKeySequence(QString("Ctrl+N")));
  add_button->setToolTip("Add a new display, Ctrl+N");
  duplicate_button_ = new QPushButton("Duplicate");
  duplicate_button_->setObjectName("DisplayPanel/DuplicateDisplayButton");
  duplicate_button_->setShortcut(QKeySequence(QString("Ctrl+D")));
  duplicate_button_->setToolTip("Duplicate a display, Ctrl+D");
  duplicate_button_->setEnabled(false);
  remove_button_ = new QPushButton("Remove");
  remove_button_->setObjectName("DisplayPanel/RemoveDisplayButton");
  remove_button_->setShortcut(QKeySequence(QString("Ctrl+X")));
  remove_button_->setToolTip("Remove displays, Ctrl+X");
  remove_button_->setEnabled(false);
  rename_button_ = new QPushButton("Rename");
  rename_button_->setObjectName("DisplayPanel/RenameDisplayButton");
  rename_button_->setShortcut(QKeySequence(QString("Ctrl+R")));
  rename_button_->setToolTip("Rename a display, Ctrl+R");
  rename_button_->setEnabled(false);

  auto button_layout = new QHBoxLayout;
  button_layout->addWidget(add_button);
  button_layout->addWidget(duplicate_button_);
  button_layout->addWidget(remove_button_);
  button_layout->addWidget(rename_button_);
  button_layout->setContentsMargins(2, 0, 2, 2);

  auto layout = new QVBoxLayout;
  layout->setContentsMargins(0, 0, 0, 2);
  layout->addWidget(tree_with_help_);
  layout->addLayout(button_layout);

  setLayout(layout);

  connect(add_button, SIGNAL(clicked(bool)), this, SLOT(onNewDisplay()));
  connect(duplicate_button_, SIGNAL(clicked(bool)), this, SLOT(onDuplicateDisplay()));
  connect(remove_button_, SIGNAL(clicked(bool)), this, SLOT(onDeleteDisplay()));
  connect(rename_button_, SIGNAL(clicked(bool)), this, SLOT(onRenameDisplay()));
  connect(property_grid_, SIGNAL(selectionHasChanged()), this, SLOT(onSelectionChanged()));
}

void DisplaysPanel::onInitialize()
{
  property_grid_->setModel(vis_manager_->getDisplayTreeModel());
}

void DisplaysPanel::onNewDisplay()
{
  QString lookup_name;
  QString display_name;
  QString topic;
  QString datatype;

  QStringList empty;

  QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
  AddDisplayDialog dialog(
    vis_manager_->getDisplayFactory(),
    empty,
    empty,
    &lookup_name,
    rviz_ros_node_,
    &display_name,
    &topic,
    &datatype);
  QApplication::restoreOverrideCursor();

  if (dialog.exec() == QDialog::Accepted) {
    Display * disp = vis_manager_->createDisplay(lookup_name, display_name, true);
    if (!topic.isEmpty() && !datatype.isEmpty()) {
      disp->setTopic(topic, datatype);
    }
  }
  activateWindow();  // Force keyboard focus back on main window.
}

void DisplaysPanel::onDuplicateDisplay()
{
  QList<Display *> displays_to_duplicate = property_grid_->getSelectedObjects<Display>();

  QList<Display *> duplicated_displays;
  QProgressDialog progress_dlg("Duplicating displays...", "Cancel", 0, displays_to_duplicate.size(),
    this);
  progress_dlg.setWindowModality(Qt::WindowModal);
  progress_dlg.show();

  // duplicate all selected displays
  int i = 0;
  for (const auto & display_to_duplicate : displays_to_duplicate) {
    // initialize display
    QString lookup_name = display_to_duplicate->getClassId();
    QString display_name = display_to_duplicate->getName();
    Display * disp = vis_manager_->createDisplay(lookup_name, display_name, true);
    // duplicate config
    Config config;
    display_to_duplicate->save(config);
    disp->load(config);
    duplicated_displays.push_back(disp);
    progress_dlg.setValue(i + 1);
    i++;
    // push cancel to stop duplicate
    if (progress_dlg.wasCanceled()) {
      break;
    }
  }
  // make sure the newly duplicated displays are selected.
  if (!duplicated_displays.isEmpty()) {
    QModelIndex first = property_grid_->getModel()->indexOf(duplicated_displays.front());
    QModelIndex last = property_grid_->getModel()->indexOf(duplicated_displays.back());
    QItemSelection selection(first, last);
    property_grid_->selectionModel()->select(selection, QItemSelectionModel::ClearAndSelect);
  }
  activateWindow();  // Force keyboard focus back on main window.
}

void DisplaysPanel::onDeleteDisplay()
{
  QList<Display *> displays_to_delete = property_grid_->getSelectedObjects<Display>();

  QModelIndex new_selected;

  for (int i = 0; i < displays_to_delete.size(); i++) {
    if (i == 0) {
      QModelIndex first = property_grid_->getModel()->indexOf(displays_to_delete[i]);
      // This is safe because the first few rows cannot be deleted (they aren't "displays").
      new_selected = first.sibling(first.row() - 1, first.column());
    }
    // Displays can emit signals from other threads with self pointers.  We're
    // freeing the display now, so ensure no one is listening to those signals.
    displays_to_delete[i]->disconnect();
    // Delete display later in case there are pending signals to it.
    displays_to_delete[i]->deleteLater();
  }

  QItemSelection selection(new_selected, new_selected);
  property_grid_->selectionModel()->select(selection, QItemSelectionModel::ClearAndSelect);

  vis_manager_->notifyConfigChanged();
}

void DisplaysPanel::onSelectionChanged()
{
  QList<Display *> displays = property_grid_->getSelectedObjects<Display>();

  int num_displays_selected = displays.size();

  duplicate_button_->setEnabled(num_displays_selected > 0);
  remove_button_->setEnabled(num_displays_selected > 0);
  rename_button_->setEnabled(num_displays_selected == 1);
}

void DisplaysPanel::onRenameDisplay()
{
  QList<Display *> displays = property_grid_->getSelectedObjects<Display>();
  if (displays.isEmpty()) {
    return;
  }
  Display * display_to_rename = displays[0];

  if (!display_to_rename) {
    return;
  }

  QString old_name = display_to_rename->getName();
  QString new_name = QInputDialog::getText(
    this, "Rename Display", "New Name?", QLineEdit::Normal, old_name);

  if (new_name.isEmpty() || new_name == old_name) {
    return;
  }

  display_to_rename->setName(new_name);
}

void DisplaysPanel::save(Config config) const
{
  Panel::save(config);
  tree_with_help_->save(config);
}

void DisplaysPanel::load(const Config & config)
{
  Panel::load(config);
  tree_with_help_->load(config);
}

}  // namespace rviz_common
