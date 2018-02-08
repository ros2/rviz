/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2017, Open Source Robotics Foundation, Inc.
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

#ifndef RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_
#define RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_

#include <QAbstractItemModel>

#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class Property;

class RVIZ_COMMON_PUBLIC PropertyTreeModel : public QAbstractItemModel
{
  Q_OBJECT

public:
  /// Constructor.
  /**
   * \param root_property The root of the property tree.
   *   PropertyTreeModel takes ownership of root_property and
   *   deletes it in its destructor.
   * \param parent A QObject to set as the parent.
   */
  explicit PropertyTreeModel(Property * root_property, QObject * parent = 0);

  /// Destructor.
  /**
   * Deletes the root property (and thus the entire property tree).
   */
  virtual ~PropertyTreeModel();

  /// Set the drag and drop class.
  void
  setDragDropClass(const QString & drag_drop_class);

  // Read-only model functions:

  /// Return the data at the index.
  QVariant
  data(const QModelIndex & index, int role) const override;

  /// Return the header data.
  QVariant
  headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  /// Return the index for a given row and column.
  QModelIndex
  index(int row, int column, const QModelIndex & parent = QModelIndex()) const override;

  /// Return the parent at the given index.
  QModelIndex
  parent(const QModelIndex & index) const override;

  /// Same as parent() but taking a Property pointer instead of an index.
  QModelIndex
  parentIndex(const Property * child) const;

  /// Return the number of rows under the given parent index.
  int
  rowCount(const QModelIndex & parent = QModelIndex()) const override;

  /// Return the number of columns under the given parent index, which is always 2 for this model.
  int
  columnCount(const QModelIndex & parent = QModelIndex()) const override;

  // Editable model functions:

  /// Return the flags at the given index.
  Qt::ItemFlags
  flags(const QModelIndex & index) const override;

  /// Set the data at the given index.
  bool
  setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole) override;

  /// Return supported drop actions.
  Qt::DropActions
  supportedDropActions() const override;

  /// Return a (non-standard) mime-encoded version of the given indexes.
  /**
   * Returns the model indexes encoded using pointer values, which
   * means they will only work within the application this is compiled into.
   */
  QMimeData *
  mimeData(const QModelIndexList & indexes) const override;  // Override from QAbstractItemModel.

  /// Take a (non-standard) mime-encoded version of an index list and drops it at the destination.
  /**
   * The model indexes are encoded using pointer values (by mimeData()), which
   * means they will only work within the application this is compiled into.
   */
  bool
  dropMimeData(
    const QMimeData * data,
    Qt::DropAction action,
    int destination_row,
    int destination_column,
    const QModelIndex & destination_parent) override;  // Override from QAbstractItemModel.

  /// Return a list with just "application/x-rviz-" plus drag_drop_class_.
  QStringList
  mimeTypes() const override;

  /// Return the root property.
  Property *
  getRoot() const;

  /// Return the Property at the given index.
  QModelIndex
  indexOf(Property * property) const;

  /// Emit a signal that indicates the Property's data has changed.
  void
  emitDataChanged(Property * property);

  /// Indicated that inserting has begun.
  void
  beginInsert(Property * parent_property, int row_within_parent, int count = 1);

  /// Indicate that inserting has ended.
  void
  endInsert();

  /// Indicate removing has begun.
  void
  beginRemove(Property * parent_property, int row_within_parent, int count = 1);

  /// Indicate removing has ended.
  void
  endRemove();

  /// Return the Property at the given index, or the root property if the index is invalid.
  Property *
  getProp(const QModelIndex & index) const;

  /// Emit the propertyHiddenChanged() signal for the given Property.
  void
  emitPropertyHiddenChanged(const Property * property);

  /// Expand (show the children of) the given Property.
  void
  expandProperty(Property * property);

  /// Collapse (hide the children of) the given Property.
  void
  collapseProperty(Property * property);

  /// Print, with printf(), the property names of current persistent indices.
  void
  printPersistentIndices();

Q_SIGNALS:
  /// Emitted when a property within the model is hidden or shown.
  void
  propertyHiddenChanged(const Property * property);

  /// Emitted when a Property which should be saved changes.
  void
  configChanged();

  /// Emitted when a Property wants to expand (display its children).
  void
  expand(const QModelIndex & index);

  /// Emitted when a Property wants to collapse (hide its children).
  void
  collapse(const QModelIndex & index);

private:
  Property * root_property_;
  /// Identifier to add to mimeTypes() entry to keep drag/drops from crossing types.
  QString drag_drop_class_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__PROPERTY_TREE_MODEL_HPP_
