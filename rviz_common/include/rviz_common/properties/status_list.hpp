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

#ifndef RVIZ_COMMON__PROPERTIES__STATUS_LIST_HPP_
#define RVIZ_COMMON__PROPERTIES__STATUS_LIST_HPP_

#include <QHash>
#include <QString>

#include "./status_property.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_common
{
namespace properties
{

class RVIZ_COMMON_PUBLIC StatusList : public StatusProperty
{
  Q_OBJECT

public:
  explicit StatusList(const QString & name = QString("Status"), Property * parent = 0);

  virtual void setLevel(Level level);

  /// Add and set a status to the list by name.
  void setStatus(Level level, const QString & name, const QString & text);

  /// Delete a status by name.
  void deleteStatus(const QString & name);

  /// Clear all statuses from the list.
  void clear();

  /// Update the level of the list based on the contained statuses.
  void updateLevel();

  /// Set the prefix of the name for added statuses.
  /**
   * Setting the name to "Foo" will give a displayed name like
   * "Foo: Ok" or "Foo: Error".
   */
  virtual void setName(const QString & name);

private:
  /// Update the label text based on the name_prefix_ and the current status level.
  void updateLabel();

  QHash<QString, StatusProperty *> status_children_;
  QString name_prefix_;
};

}  // namespace properties
}  // namespace rviz_common

#endif  // RVIZ_COMMON__PROPERTIES__STATUS_LIST_HPP_
