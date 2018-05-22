/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include "rviz_common/panel.hpp"
#include "rviz_common/display_context.hpp"

namespace rviz_common
{

Panel::Panel(QWidget * parent)
: QWidget(parent), context_(nullptr)
{
}

Panel::~Panel() = default;

void Panel::initialize(DisplayContext * context)
{
  context_ = context;
  onInitialize();
}

void Panel::onInitialize() {}

QString Panel::getName() const
{
  return name_;
}

void Panel::setName(const QString & name)
{
  name_ = name;
}

QString Panel::getDescription() const
{
  return description_;
}

void Panel::setDescription(const QString & description)
{
  description_ = description;
}

QString Panel::getClassId() const
{
  return class_id_;
}

void Panel::setClassId(const QString & class_id)
{
  class_id_ = class_id;
}

void Panel::save(Config config) const
{
  config.mapSetValue("Class", getClassId() );
  config.mapSetValue("Name", getName() );
}

void Panel::load(const Config & config)
{
  QString name;
  if (config.mapGetString("Name", &name)) {
    setName(name);
  }
}

DisplayContext * Panel::getDisplayContext() const
{
  return context_;
}

}  // namespace rviz_common
