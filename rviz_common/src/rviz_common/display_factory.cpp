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

#include "display_factory.hpp"

#include <string>

#include <tinyxml2.h>  // NOLINT: cpplint is unable to handle the include order here

#include "rviz_common/display_group.hpp"
#include "rviz_common/logging.hpp"

namespace rviz_common
{

static Display * newDisplayGroup()
{
  return new DisplayGroup();
}

DisplayFactory::DisplayFactory()
: PluginlibFactory<Display>("rviz_common", "rviz_common::Display")
{
  addBuiltInClass("rviz_common", "Group", "A container for Displays", &newDisplayGroup);
}

Display * DisplayFactory::makeRaw(const QString & class_id, QString * error_return)
{
  Display * display = PluginlibFactory<Display>::makeRaw(class_id, error_return);
  if (display) {
    display->setIcon(getPluginInfo(class_id).icon);
  }
  return display;
}

QSet<QString> DisplayFactory::getMessageTypes(const QString & class_id)
{
  // lookup in cache
  if (message_type_cache_.find(class_id) != message_type_cache_.end()) {
    return message_type_cache_[class_id];
  }

  // Always initialize cache as empty so if we don't find it, next time
  // we won't look for it anymore either.
  message_type_cache_[class_id] = QSet<QString>();

  // parse xml plugin description to find out message types of all displays in it.
  QString xml_file = getPluginManifestPath(class_id);

  if (!xml_file.isEmpty()) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Parsing " << xml_file.toStdString());
    tinyxml2::XMLDocument document;
    document.LoadFile(xml_file.toUtf8().constData());
    tinyxml2::XMLElement * config = document.RootElement();
    if (!hasRootNode(config, xml_file.toStdString()) ||
      !hasLibraryRoot(config, xml_file.toStdString()))
    {
      return QSet<QString>();
    }
    // Step into the filter list if necessary
    if (config->Value() == std::string("class_libraries")) {
      config = config->FirstChildElement("library");
    }

    tinyxml2::XMLElement * library = config;
    while (library) {
      fillCacheForAllClassElements(library);
      library = library->NextSiblingElement("library");
    }
  }

  // search cache again.
  if (message_type_cache_.find(class_id) != message_type_cache_.end()) {
    return message_type_cache_[class_id];
  }

  return QSet<QString>();
}

bool DisplayFactory::hasRootNode(tinyxml2::XMLElement * root_element, const std::string & xml_file)
{
  if (root_element == nullptr) {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Skipping XML Document \"" << xml_file << "\" which had no Root Element.  "
        "This likely means the XML is malformed or missing.");
    return false;
  }
  return true;
}

bool
DisplayFactory::hasLibraryRoot(tinyxml2::XMLElement * root_element, const std::string & xml_file)
{
  if (root_element->Value() != std::string("library") &&
    root_element->Value() != std::string("class_libraries"))
  {
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "The XML document \"" << xml_file <<
        "\" given to add must have either \"library\" or "
        "\"class_libraries\" as the root tag");
    return false;
  }
  return true;
}

void DisplayFactory::fillCacheForAllClassElements(tinyxml2::XMLElement * library)
{
  tinyxml2::XMLElement * class_element = library->FirstChildElement("class");
  while (class_element) {
    const std::string derived_class = lookupDerivedClass(class_element);
    const std::string current_class_id = lookupClassId(class_element, derived_class);
    QSet<QString> message_types = parseMessageTypes(class_element, current_class_id);

    message_type_cache_[QString::fromStdString(current_class_id)] = message_types;

    class_element = class_element->NextSiblingElement("class");
  }
}

QSet<QString> DisplayFactory::parseMessageTypes(
  tinyxml2::XMLElement * class_element, const std::string & current_class_id) const
{
  QSet<QString> message_types;

  const tinyxml2::XMLElement * message_type = class_element->FirstChildElement("message_type");
  while (message_type) {
    if (message_type->GetText()) {
      const char * message_type_str = message_type->GetText();
      RVIZ_COMMON_LOG_DEBUG_STREAM(
        current_class_id << " supports message type " << message_type_str);
      message_types.insert(QString(message_type_str));
    }
    message_type = message_type->NextSiblingElement("message_type");
  }
  return message_types;
}

std::string DisplayFactory::lookupDerivedClass(const tinyxml2::XMLElement * class_element) const
{
  if (class_element->Attribute("type")) {
    return class_element->Attribute("type");
  }
  return "";
}

std::string DisplayFactory::lookupClassId(
  const tinyxml2::XMLElement * class_element, const std::string & derived_class) const
{
  if (class_element->Attribute("name")) {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "XML file specifies lookup name (i.e. magic name) = " << class_element->Attribute("name"));
    return class_element->Attribute("name");
  } else {
    RVIZ_COMMON_LOG_DEBUG_STREAM(
      "XML file has no lookup name (i.e. magic name) for class " << derived_class <<
        ", assuming class_id == real class name.");
    return derived_class;
  }
}

}  // namespace rviz_common
