// Copyright (c) 2020, TNG Technology Consulting GmbH.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <string>
#include <unordered_set>

#include "image_transport/camera_common.hpp"
#include "image_transport/subscriber_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcutils/logging_macros.h"
#include "rviz_default_plugins/displays/image/get_transport_from_topic.hpp"

namespace rviz_default_plugins
{
namespace displays
{

namespace
{
const std::unordered_set<std::string> & getKnownNonRawTransports()
{
  static const std::unordered_set<std::string> known_transports = []() {
      std::unordered_set<std::string> result;
      try {
        pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
          "image_transport", "image_transport::SubscriberPlugin");
        for (const std::string & plugin_class : sub_loader.getDeclaredClasses()) {
          const std::string message_type = image_transport::get_message_type_from_manifest(
            sub_loader.getPluginManifestPath(plugin_class), plugin_class);
          if (!message_type.empty()) {
            const std::string transport_name = image_transport::get_transport_name_from_manifest(
              sub_loader.getPluginManifestPath(plugin_class), plugin_class);
            if (transport_name != "raw") {
              result.insert(transport_name);
            }
          }
        }
      } catch (const std::exception & e) {
        RCUTILS_LOG_WARN_NAMED(
          "rviz_default_plugins",
          "Failed to load image_transport subscriber plugins: %s", e.what());
      } catch (...) {
        RCUTILS_LOG_WARN_NAMED(
          "rviz_default_plugins",
          "Failed to load image_transport subscriber plugins: unknown error");
      }
      return result;
    }();
  return known_transports;
}
}  // namespace

bool isRawTransport(const std::string & topic)
{
  const std::string last_subtopic = topic.substr(topic.find_last_of('/') + 1);
  const auto & transports = getKnownNonRawTransports();
  return transports.find(last_subtopic) == transports.end();
}

std::string getTransportFromTopic(const std::string & topic)
{
  if (isRawTransport(topic)) {
    return "raw";
  }
  return topic.substr(topic.find_last_of('/') + 1);
}

std::string getBaseTopicFromTopic(const std::string & topic)
{
  if (isRawTransport(topic)) {
    return topic;
  }
  return topic.substr(0, topic.find_last_of('/'));
}

}  //  end namespace displays
}  //  end namespace rviz_default_plugins
