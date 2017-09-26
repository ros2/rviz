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

// TODO(wjwwood): revist this file when pluginlib solution found

#ifndef SRC__RVIZ_COMMON__DISPLAY_FACTORY_HPP_
#define SRC__RVIZ_COMMON__DISPLAY_FACTORY_HPP_

#include "./display.hpp"
#include "./pluginlib_factory.hpp"

#include <QMap>  // NOLINT: cpplint is unable to handle the include order here
#include <QSet>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here

namespace rviz_common
{

class DisplayFactory : public PluginlibFactory<Display>
{
public:
  DisplayFactory();

  /** @brief Get all supported message types for the given class id. */
  virtual QSet<QString> getMessageTypes(const QString & class_id);

protected:
  /** @brief Overridden from PluginlibFactory<Display> to set the icon of the Display. */
  virtual Display * makeRaw(const QString & class_id, QString * error_return = NULL);

  QMap<QString, QSet<QString>> message_type_cache_;
};

}  // namespace rviz_common

#endif  // SRC__RVIZ_COMMON__DISPLAY_FACTORY_HPP_
