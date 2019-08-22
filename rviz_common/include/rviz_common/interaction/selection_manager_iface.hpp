/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_
#define RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QObject>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_handler.hpp"
#include "rviz_common/visibility_control.hpp"

namespace rviz_rendering
{
class RenderWindow;
}

namespace rviz_common
{
namespace properties
{
class PropertyTreeModel;
}

class DisplayContext;

namespace interaction
{

class RVIZ_COMMON_PUBLIC SelectionManagerIface : public QObject
{
  Q_OBJECT

public:
  enum SelectType
  {
    Add,
    Remove,
    Replace
  };

  virtual void initialize() = 0;

  /// Control the highlight box being displayed while selecting.
  virtual void
  highlight(rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2) = 0;

  virtual void removeHighlight() = 0;

  /// Select all objects in bounding box.
  virtual void select(
    rviz_rendering::RenderWindow * window, int x1, int y1, int x2, int y2, SelectType type) = 0;

  /// Get all objects in a bounding box.
  /**
   * \return handles of all objects in the given bounding box
   */
  virtual void pick(
    rviz_rendering::RenderWindow * window,
    int x1,
    int y1,
    int x2,
    int y2,
    M_Picked & results) = 0;

  virtual void update() = 0;

  virtual const M_Picked & getSelection() const = 0;

  /// Tell the view controller to look at the selection.
  virtual void focusOnSelection() = 0;

  /// Change the size of the off-screen selection buffer texture.
  virtual void setTextureSize(unsigned size) = 0;

  virtual rviz_common::properties::PropertyTreeModel * getPropertyModel() = 0;
};

}  // namespace interaction
}  // namespace rviz_common

#endif  // RVIZ_COMMON__INTERACTION__SELECTION_MANAGER_IFACE_HPP_
