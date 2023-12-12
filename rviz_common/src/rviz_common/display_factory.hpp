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

#ifndef RVIZ_COMMON__DISPLAY_FACTORY_HPP_
#define RVIZ_COMMON__DISPLAY_FACTORY_HPP_

#include <tinyxml2.h>

#include <string>

#include <QMap>  // NOLINT: cpplint cannot handle include order here
#include <QSet>  // NOLINT: cpplint cannot handle include order here
#include <QString>  // NOLINT: cpplint cannot handle include order here

#include "rviz_common/factory/pluginlib_factory.hpp"
#include "rviz_common/display.hpp"

namespace rviz_common
{

class DisplayFactory : public PluginlibFactory<Display>
{
public:
  DisplayFactory();

  /// Get all supported message types for the given class id.
  virtual QSet<QString> getMessageTypes(const QString & class_id);

protected:
  /// Overridden from PluginlibFactory<Display> to set the icon of the Display.
  Display * makeRaw(const QString & class_id, QString * error_return = nullptr) override;

  QMap<QString, QSet<QString>> message_type_cache_;

private:
  bool hasRootNode(tinyxml2::XMLElement * root_element, const std::string & xml_file);
  bool hasLibraryRoot(tinyxml2::XMLElement * root_element, const std::string & xml_file);
  void fillCacheForAllClassElements(tinyxml2::XMLElement * library);
  QSet<QString> parseMessageTypes(
    tinyxml2::XMLElement * class_element, const std::string & current_class_id) const;
  std::string lookupClassId(
    const tinyxml2::XMLElement * class_element, const std::string & derived_class) const;
  std::string lookupDerivedClass(const tinyxml2::XMLElement * class_element) const;
};

}  // namespace rviz_common

#endif  // RVIZ_COMMON__DISPLAY_FACTORY_HPP_
