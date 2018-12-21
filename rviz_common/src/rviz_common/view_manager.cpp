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

#include "rviz_common/view_manager.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <sstream>

#include "rviz_common/factory/pluginlib_factory.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/render_panel.hpp"

#include "./failed_view_controller.hpp"

namespace rviz_common
{

using rviz_common::properties::PropertyTreeModel;

struct ViewManager::ViewManagerImpl
{
  explicit ViewManagerImpl(DisplayContext * context_arg)
  : context(context_arg),
    root_property(new ViewControllerContainer),
    property_model(new PropertyTreeModel(root_property)),
    factory(new PluginlibFactory<ViewController>("rviz_common", "rviz_common::ViewController")),
    current(nullptr),
    render_panel(nullptr)
  {}

  ~ViewManagerImpl()
  {}

  DisplayContext * context;
  ViewControllerContainer * root_property;
  std::unique_ptr<rviz_common::properties::PropertyTreeModel> property_model;
  std::unique_ptr<PluginlibFactory<ViewController>> factory;
  ViewController * current;
  RenderPanel * render_panel;
};

ViewManager::ViewManager(DisplayContext * context)
: impl_(new ViewManagerImpl(context))
{
  impl_->property_model->setDragDropClass("view-controller");
  connect(impl_->property_model.get(), SIGNAL(configChanged()), this, SIGNAL(configChanged()));
}

ViewManager::~ViewManager()
{}

void ViewManager::initialize()
{
  setCurrent(create("rviz_default_plugins/Orbit"), false);
}

void ViewManager::update(float wall_dt, float ros_dt)
{
  if (getCurrent()) {
    getCurrent()->update(wall_dt, ros_dt);
  }
}

ViewController * ViewManager::create(const QString & class_id)
{
  QString error;
  ViewController * view = this->impl_->factory->make(class_id, &error);
  if (!view) {
    view = new rviz_common::FailedViewController(class_id, error);
  }
  view->initialize(this->impl_->context);

  return view;
}

QStringList ViewManager::getDeclaredClassIdsFromFactory()
{
  QStringList class_ids;
  for (const auto & plugin : impl_->factory->getDeclaredPlugins()) {
    class_ids.append(plugin.id);
  }
  return class_ids;
}

ViewController * ViewManager::getCurrent() const
{
  return impl_->current;
}

void ViewManager::setCurrentFrom(ViewController * source_view)
{
  if (source_view == nullptr) {
    return;
  }

  ViewController * previous = getCurrent();
  if (source_view != previous) {
    ViewController * new_current = copy(source_view);

    setCurrent(new_current, false);
    Q_EMIT configChanged();
  }
}

void ViewManager::onCurrentDestroyed(QObject * obj)
{
  if (obj == impl_->current) {
    impl_->current = nullptr;
  }
}

void ViewManager::setCurrent(ViewController * new_current, bool mimic_view)
{
  ViewController * previous = getCurrent();
  if (previous) {
    if (mimic_view) {
      new_current->mimic(previous);
    } else {
      new_current->transitionFrom(previous);
    }
    disconnect(previous, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  }
  new_current->setName("Current View");
  connect(new_current, SIGNAL(destroyed(QObject*)), this, SLOT(onCurrentDestroyed(QObject*)));
  impl_->current = new_current;
  impl_->root_property->addChildToFront(new_current);
  delete previous;

  if (impl_->render_panel) {
    // This setViewController() can indirectly call
    // ViewManager::update(), so make sure getCurrent() will return the
    // new one by this point.
    impl_->render_panel->setViewController(new_current);
  }
  Q_EMIT currentChanged();
}

void ViewManager::setCurrentViewControllerType(const QString & new_class_id)
{
  setCurrent(create(new_class_id), true);
}

void ViewManager::copyCurrentToList()
{
  ViewController * current = getCurrent();
  if (current) {
    ViewController * new_copy = copy(current);
    new_copy->setName(impl_->factory->getPluginInfo(new_copy->getClassId()).name);
    impl_->root_property->addChild(new_copy);
  }
}

ViewController * ViewManager::getViewAt(int index) const
{
  if (index < 0) {
    index = 0;
  }
  // TODO(greimela) Figure out why qobject_cast does not work here
  return dynamic_cast<ViewController *>(impl_->root_property->childAt(index + 1));
}

int ViewManager::getNumViews() const
{
  int count = impl_->root_property->numChildren();
  if (count <= 0) {
    return 0;
  } else {
    return count - 1;
  }
}

void ViewManager::add(ViewController * view, int index)
{
  if (index < 0) {
    index = impl_->root_property->numChildren();
  } else {
    index++;
  }
  impl_->property_model->getRoot()->addChild(view, index);
}

ViewController * ViewManager::take(ViewController * view)
{
  for (int i = 0; i < getNumViews(); i++) {
    if (getViewAt(i) == view) {
      return qobject_cast<ViewController *>(impl_->root_property->takeChildAt(i + 1));
    }
  }
  return nullptr;
}

ViewController * ViewManager::takeAt(int index)
{
  if (index < 0) {
    return nullptr;
  }
  return qobject_cast<ViewController *>(impl_->root_property->takeChildAt(index + 1));
}

PropertyTreeModel * ViewManager::getPropertyModel()
{
  return impl_->property_model.get();
}

void ViewManager::load(const Config & config)
{
  Config current_config = config.mapGetChild("Current");
  QString class_id;
  if (current_config.mapGetString("Class", &class_id)) {
    ViewController * new_current = create(class_id);
    new_current->load(current_config);
    setCurrent(new_current, false);
  }

  Config saved_views_config = config.mapGetChild("Saved");
  impl_->root_property->removeChildren(1);
  int num_saved = saved_views_config.listLength();
  for (int i = 0; i < num_saved; i++) {
    Config view_config = saved_views_config.listChildAt(i);

    if (view_config.mapGetString("Class", &class_id)) {
      ViewController * view = create(class_id);
      view->load(view_config);
      add(view);
    }
  }
}

void ViewManager::save(Config config) const
{
  getCurrent()->save(config.mapMakeChild("Current"));

  Config saved_views_config = config.mapMakeChild("Saved");
  for (int i = 0; i < getNumViews(); i++) {
    if (getViewAt(i)) {
      getViewAt(i)->save(saved_views_config.listAppendNew());
    }
  }
}

ViewController * ViewManager::copy(ViewController * source)
{
  Config config;
  source->save(config);

  ViewController * copy_of_source = create(source->getClassId());
  copy_of_source->load(config);

  return copy_of_source;
}

void ViewManager::setRenderPanel(RenderPanel * render_panel)
{
  impl_->render_panel = render_panel;
}

RenderPanel * ViewManager::getRenderPanel() const
{
  return impl_->render_panel;
}

Qt::ItemFlags ViewControllerContainer::getViewFlags(int column) const
{
  return Property::getViewFlags(column) | Qt::ItemIsDropEnabled;
}

void ViewControllerContainer::addChild(Property * child, int index)
{
  if (index == 0) {
    index = 1;
  }
  Property::addChild(child, index);
}

void ViewControllerContainer::addChildToFront(Property * child)
{
  Property::addChild(child, 0);
}

}  // namespace rviz_common
