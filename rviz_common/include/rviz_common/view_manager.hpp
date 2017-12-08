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

#ifndef RVIZ_COMMON__VIEW_MANAGER_HPP_
#define RVIZ_COMMON__VIEW_MANAGER_HPP_

#include <algorithm>
#include <memory>

#include <QList>  // NOLINT: cpplint is unable to handle the include order here
#include <QObject>  // NOLINT: cpplint is unable to handle the include order here
#include <QStringList>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/properties/property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/visibility_control.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_common
{

namespace properties
{

class PropertyTreeModel;

}  // namespace properties

class ViewControllerContainer;

class RVIZ_COMMON_PUBLIC ViewManager : public QObject
{
  Q_OBJECT

public:
  explicit ViewManager(DisplayContext * context);
  ~ViewManager();

  void initialize();

  void update(float wall_dt, float ros_dt);

  /// Return the current ViewController in use for the main RenderWindow.
  ViewController * getCurrent() const;

  /// Create a view controller by name.
  ViewController * create(const QString & type);

  /// Return a list of the class id's for available view controllers.
  QStringList getDeclaredClassIdsFromFactory();

  /// Get the number of views.
  int getNumViews() const;

  /// Get a ViewController by index.
  ViewController * getViewAt(int index) const;

  /// Add a view controller.
  void add(ViewController * view, int index = -1);

  /// Remove the given ViewController from the list and return it.
  /**
   * If it is not in the list, nullptr is returned and nothing changes.
   */
  ViewController * take(ViewController * view);

  /// Remove the ViewController at the given index from the list and return it.
  /**
   * If the index is not valid, nullptr is returned and nothing changes.
   */
  ViewController * takeAt(int index);

  /// Get property tree model.
  rviz_common::properties::PropertyTreeModel * getPropertyModel();

  /// Load configuration from a Config object.
  void load(const Config & config);

  /// Save configuration to a Config object.
  void save(Config config) const;

  /// Make a copy of view_to_copy and install that as the new current ViewController.
  void setCurrentFrom(ViewController * view_to_copy);

  /// Return a copy of source.
  /**
   * The copy is made by saving source to a Config and instantiating and
   * loading a new one from that.
   */
  ViewController * copy(ViewController * source);

  /// Get the factory for loading view controller plugins.
  // PluginlibFactory<ViewController> * getFactory() const {return factory_; }

  /// Set the render panel whose view will be controlled by ViewControllers of this ViewManager.
  void setRenderPanel(RenderPanel * render_panel);

  /// Return the render panel widget whos views are managed by this ViewManager.
  RenderPanel * getRenderPanel() const;

public Q_SLOTS:
  /// Make a copy of the current ViewController and add it to the end of the list of saved views.
  void copyCurrentToList();

  /// Create a new view controller of the given type, then use it to replace the current view.
  void setCurrentViewControllerType(const QString & new_class_id);

Q_SIGNALS:
  /// Emitted when the configuration of the view controller changes.
  void configChanged();

  /// Emitted just after the current view controller changes.
  void currentChanged();

private Q_SLOTS:
  /// Called on destruction.
  void onCurrentDestroyed(QObject * obj);

private:
  /// Set new_current as current.
  /**
   * This calls mimic() or transitionFrom() on the new controller,
   * deletes the previous controller (if one existed), and tells the
   * RenderPanel about the new controller.
   *
   * \param mimic_view If true, call new_current->mimic(previous), if false
   *   call new_current->transitionFrom(previous).
   */
  void setCurrent(ViewController * new_current, bool mimic_view);

  struct ViewManagerImpl;

  std::unique_ptr<ViewManagerImpl> impl_;
};

/// Wrapper property for view controllers.
/**
 * This container property for ViewControllers is need to get the drag/drop
 * right for the funky way Current-View is always the first entry.
 */
class RVIZ_COMMON_PUBLIC ViewControllerContainer : public rviz_common::properties::Property
{
  Q_OBJECT

public:
  /// Get the view flags.
  Qt::ItemFlags getViewFlags(int column) const;

  /// Add a child ViewController.
  /**
   * This notifies the model about the addition.
   *
   * This is overridden from Property to keep saved ViewControllers from being
   * added at index 0, where the Current view belongs.
   *
   * \param child The child to add.
   * \param index [optional] The index at which to add the child.
   *   If less than 0 or greater than the number of child properties, the
   *   child will be added at the end.
   */
  virtual void addChild(Property * child, int index = -1);

  /// Add a child propert to the front of the list of child properties.
  void addChildToFront(Property * child);
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__VIEW_MANAGER_HPP_
