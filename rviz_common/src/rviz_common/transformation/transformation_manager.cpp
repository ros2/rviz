/*
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "transformation_manager.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rviz_common
{
namespace transformation
{

TransformationManager::TransformationManager(std::string current_plugin)
: available_plugins_({}), current_plugin_(current_plugin)
{
  // TODO(bottteroa-si): use pluginlob to fill correctly available plugins list
  setAvailablePlugins({"dummy_transformer", "tf_transformer"});
}

void TransformationManager::setAvailablePlugins(std::vector<std::string> available_plugins)
{
  available_plugins_ = available_plugins;
}

std::string TransformationManager::getCurrentPlugin()
{
  return current_plugin_;
}

std::vector<std::string> TransformationManager::getAvailablePlugins()
{
  return available_plugins_;
}

void TransformationManager::setPlugin(std::string plugin_name)
{
  current_plugin_ = plugin_name;
  // TODO(botteroa-si): use pluginlib to build an object of the correct type
//  auto plugin = // pluginlib code
//  Q_EMIT currentPluginChanged(plugin);
}

}  // namespace transformation
}  // namespace rviz_common
