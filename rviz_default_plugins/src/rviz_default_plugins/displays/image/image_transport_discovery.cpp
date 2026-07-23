// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

#include "rviz_default_plugins/displays/image/image_transport_discovery.hpp"

#include <string>
#include <vector>

#include "image_transport/camera_common.hpp"
#include "image_transport/subscriber_plugin.hpp"
#include "pluginlib/class_loader.hpp"

namespace rviz_default_plugins
{
namespace displays
{

std::vector<ImageTransportPluginInfo> discoverImageTransportSubscriberPlugins()
{
  std::vector<ImageTransportPluginInfo> plugins;

  pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader(
    "image_transport", "image_transport::SubscriberPlugin");
  for (const std::string & plugin_class : sub_loader.getDeclaredClasses()) {
    const std::string manifest_path = sub_loader.getPluginManifestPath(plugin_class);
    ImageTransportPluginInfo info;
    info.transport_name =
      image_transport::get_transport_name_from_manifest(manifest_path, plugin_class);
    info.message_type =
      image_transport::get_message_type_from_manifest(manifest_path, plugin_class);
    plugins.push_back(info);
  }
  return plugins;
}

}  // namespace displays
}  // namespace rviz_default_plugins
