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

#ifndef RVIZ_COMMON__FACTORY__FACTORY_HPP_
#define RVIZ_COMMON__FACTORY__FACTORY_HPP_

#include <utility>
#include <vector>

#include <QIcon>  // NOLINT
#include <QString>  // NOLINT
#include <QStringList>  // NOLINT

namespace rviz_common
{

/// Struct to bundle the information available for a plugin
struct PluginInfo
{
  QString id;
  QString name;
  QString package;
  QString description;
  QIcon icon;

  friend bool operator==(const PluginInfo & lhs, const PluginInfo & rhs)
  {
    return lhs.id == rhs.id;
  }

  friend bool operator<(const PluginInfo & lhs, const PluginInfo & rhs)
  {
    return lhs.id < rhs.id;
  }
};

/// Abstract base class representing a plugin load-able class factory.
/**
 * The class represents the ability to get a list of class IDs and the ability
 * to get name, description, and package strings for each.
 * Actually instantiating objects must be done by subclasses specialized for
 * specific types.
 */
class Factory
{
public:
  virtual ~Factory() {}

  virtual std::vector<PluginInfo> getDeclaredPlugins() = 0;
  virtual PluginInfo getPluginInfo(const QString & class_id) const = 0;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__FACTORY__FACTORY_HPP_
