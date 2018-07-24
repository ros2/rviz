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

#include "rviz_common/factory/pluginlib_factory.hpp"

namespace rviz_common
{
namespace transformation
{

TransformationManager::TransformationManager(
  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
: rviz_ros_node_(rviz_ros_node)
{
  factory_ = std::make_unique<PluginlibFactory<FrameTransformer>>(
    "rviz_common", "rviz_common::transformation::FrameTransformer");

  // TODO(greimela) Robust loading
  // 1) Load from config
  // 2) Load TF if available
  // 3) Load dummy
  setTransformer("rviz_default_plugins/TFFrameTransformer");
}

QStringList TransformationManager::getAvailableTransformerNames()
{
  return factory_->getDeclaredClassIds();
}

std::shared_ptr<FrameTransformer> TransformationManager::getCurrentTransformer()
{
  return current_transformer_;
}

QString TransformationManager::getCurrentTransformerName()
{
  return current_transformer_name_;
}

void TransformationManager::setTransformer(const QString & transformer_name)
{
  current_transformer_ = std::unique_ptr<FrameTransformer>(factory_->make(transformer_name));
  current_transformer_name_ = transformer_name;

  current_transformer_->initialize(rviz_ros_node_);

  Q_EMIT transformerChanged(current_transformer_);
}

}  // namespace transformation
}  // namespace rviz_common
