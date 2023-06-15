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

#include "rviz_common/tool.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/property.hpp"

namespace rviz_common
{

using rviz_common::properties::Property;

Tool::Tool()
: scene_manager_(nullptr),
  context_(nullptr),
  shortcut_key_('\0'),
  access_all_keys_(false),
  property_container_(new Property())
{
}

Tool::~Tool()
{
  delete property_container_;
}

void Tool::initialize(DisplayContext * context)
{
  context_ = context;
  scene_manager_ = context_->getSceneManager();

  // Let subclasses do initialization if they want.
  onInitialize();
}

rviz_common::properties::Property * Tool::getPropertyContainer() const
{
  return property_container_;
}

char Tool::getShortcutKey() const
{
  return shortcut_key_;
}

bool Tool::accessAllKeys() const
{
  return access_all_keys_;
}

void Tool::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
}

int Tool::processMouseEvent(ViewportMouseEvent & event)
{
  (void) event;
  return 0;
}

int Tool::processKeyEvent(QKeyEvent * event, RenderPanel * panel)
{
  (void) event;
  (void) panel;
  return 0;
}

QString Tool::getName() const
{
  return name_;
}

QString Tool::getDescription() const
{
  return description_;
}

QString Tool::getClassId() const
{
  return class_id_;
}

void Tool::setClassId(const QString & class_id)
{
  class_id_ = class_id;
}

const QIcon & Tool::getIcon()
{
  return icon_;
}

const QCursor & Tool::getCursor()
{
  return cursor_;
}

void Tool::setIcon(const QIcon & icon)
{
  icon_ = icon;
  cursor_ = makeIconCursor(icon.pixmap(16), "tool_cursor:" + name_);
}

void Tool::setCursor(const QCursor & cursor)
{
  cursor_ = cursor;
}

void Tool::setName(const QString & name)
{
  name_ = name;
  property_container_->setName(name_);

  Q_EMIT nameChanged(name_);
}

void Tool::setDescription(const QString & description)
{
  description_ = description;
  property_container_->setDescription(description_);
}

void Tool::load(const Config & config)
{
  property_container_->load(config);
}

void Tool::save(Config config) const
{
  property_container_->save(config);
  config.mapSetValue("Class", getClassId());
}

void Tool::setStatus(const QString & message)
{
  if (context_) {
    context_->setStatus(message);
  }
}

}  // namespace rviz_common
