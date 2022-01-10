/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include "new_object_dialog.hpp"

#include <algorithm>
#include <map>

#include <QDialogButtonBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QGroupBox>  // NOLINT: cpplint is unable to handle the include order here
#include <QLabel>  // NOLINT: cpplint is unable to handle the include order here
#include <QLineEdit>  // NOLINT: cpplint is unable to handle the include order here
#include <QPushButton>  // NOLINT: cpplint is unable to handle the include order here
#include <QTextBrowser>  // NOLINT: cpplint is unable to handle the include order here
#include <QTreeWidget>  // NOLINT: cpplint is unable to handle the include order here
#include <QVBoxLayout>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/load_resource.hpp"

namespace rviz_common
{

NewObjectDialog::NewObjectDialog(
  Factory * factory,
  const QString & object_type,
  const QStringList & disallowed_display_names,
  const QStringList & disallowed_class_lookup_names,
  QString * lookup_name_output,
  QString * display_name_output,
  QWidget * parent)
: QDialog(parent),
  factory_(factory),
  disallowed_display_names_(disallowed_display_names),
  disallowed_class_lookup_names_(disallowed_class_lookup_names),
  lookup_name_output_(lookup_name_output),
  display_name_output_(display_name_output)
{
  //***** Layout

  // Display Type group
  auto type_box = new QGroupBox(object_type + " Type");

  auto tree = new QTreeWidget;
  tree->setHeaderHidden(true);
  fillTree(tree);

  auto description_label = new QLabel("Description:");
  description_ = new QTextBrowser;
  description_->setMaximumHeight(100);
  description_->setOpenExternalLinks(true);

  auto type_layout = new QVBoxLayout;
  type_layout->addWidget(tree);
  type_layout->addWidget(description_label);
  type_layout->addWidget(description_);

  type_box->setLayout(type_layout);

  // Display Name group
  QGroupBox * name_box = nullptr;
  if (display_name_output_) {
    name_box = new QGroupBox(object_type + " Name");
    name_editor_ = new QLineEdit;
    auto name_layout = new QVBoxLayout;
    name_layout->addWidget(name_editor_);
    name_box->setLayout(name_layout);
  }

  // Buttons
  button_box_ = new QDialogButtonBox(
    QDialogButtonBox::Ok | QDialogButtonBox::Cancel, Qt::Horizontal);

  auto main_layout = new QVBoxLayout;
  main_layout->addWidget(type_box);
  if (display_name_output_) {
    main_layout->addWidget(name_box);
  }
  main_layout->addWidget(button_box_);
  setLayout(main_layout);

  //***** Connections
  // *INDENT-OFF* - uncrustify cannot deal with commas here
  connect(tree, SIGNAL(currentItemChanged(QTreeWidgetItem *, QTreeWidgetItem *)),
    this, SLOT(onDisplaySelected(QTreeWidgetItem *)));
  connect(tree, SIGNAL(itemActivated(QTreeWidgetItem *, int)),
    this, SLOT(accept()));
  connect(button_box_, SIGNAL(accepted()), this, SLOT(accept()));
  connect(button_box_, SIGNAL(rejected()), this, SLOT(reject()));
  // *INDENT-ON*

  if (display_name_output_) {
    connect(name_editor_, SIGNAL(textEdited(const QString&)), this, SLOT(onNameChanged()));
  }
}

QSize NewObjectDialog::sizeHint() const
{
  return QSize(500, 660);
}

void NewObjectDialog::fillTree(QTreeWidget * tree)
{
  QIcon default_package_icon = loadPixmap("package://rviz_common/icons/default_package_icon.png");

  auto plugins = factory_->getDeclaredPlugins();
  std::sort(plugins.begin(), plugins.end());

  // Map from package names to the corresponding top-level tree widget items.
  std::map<QString, QTreeWidgetItem *> package_items;

  for (const auto & plugin : plugins) {
    QTreeWidgetItem * package_item;

    std::map<QString, QTreeWidgetItem *>::iterator mi;
    mi = package_items.find(plugin.package);
    if (mi == package_items.end()) {
      package_item = new QTreeWidgetItem(tree);
      package_item->setText(0, plugin.package);
      package_item->setIcon(0, default_package_icon);

      package_item->setExpanded(true);
      package_items[plugin.package] = package_item;
    } else {
      package_item = (*mi).second;
    }
    auto class_item = new QTreeWidgetItem(package_item);

    class_item->setIcon(0, plugin.icon);

    class_item->setText(0, plugin.name);
    class_item->setWhatsThis(0, plugin.description);
    // Store the lookup name for each class in the UserRole of the item.
    class_item->setData(0, Qt::UserRole, plugin.id);
    class_item->setDisabled(disallowed_class_lookup_names_.contains(plugin.id));
  }
}

void NewObjectDialog::onDisplaySelected(QTreeWidgetItem * selected_item)
{
  QString html = "<html><body>" + selected_item->whatsThis(0) + "</body></html>";
  description_->setHtml(html);

  // We stored the lookup name for the class in the UserRole of the items.
  QVariant user_data = selected_item->data(0, Qt::UserRole);
  bool selection_is_valid = user_data.isValid();
  if (selection_is_valid) {
    lookup_name_ = user_data.toString();
    if (display_name_output_) {
      QString display_name = selected_item->text(0);

      int counter = 1;
      QString name;
      do {
        name = display_name;
        if (counter > 1) {
          name += QString::number(counter);
        }
        ++counter;
      } while (disallowed_display_names_.contains(name));

      name_editor_->setText(name);
    }
  } else {
    lookup_name_ = "";
    if (display_name_output_) {
      name_editor_->setText("");
    }
  }
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

bool NewObjectDialog::isValid()
{
  if (lookup_name_.size() == 0) {
    setError("Select a Display type.");
    return false;
  }
  if (display_name_output_) {
    QString display_name = name_editor_->text();
    if (display_name.size() == 0) {
      setError("Enter a name for the display.");
      return false;
    }
    if (disallowed_display_names_.contains(display_name)) {
      setError("Name in use.  Display names must be unique.");
      return false;
    }
  }
  setError("");
  return true;
}

void NewObjectDialog::setError(const QString & error_text)
{
  button_box_->button(QDialogButtonBox::Ok)->setToolTip(error_text);
}

void NewObjectDialog::onNameChanged()
{
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

void NewObjectDialog::accept()
{
  if (isValid()) {
    *lookup_name_output_ = lookup_name_;
    if (display_name_output_) {
      *display_name_output_ = name_editor_->text();
    }
    QDialog::accept();
  }
}

}  // namespace rviz_common
