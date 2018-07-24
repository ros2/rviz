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

#ifndef RVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_
#define RVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <QObject>  // NOLINT
#include <QString>  // NOLINT

#include "rviz_common/factory/pluginlib_factory.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

namespace rviz_common
{
namespace transformation
{

class TransformationManager : public QObject
{
  Q_OBJECT

public:
  explicit TransformationManager(ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node);

  QStringList getAvailableTransformerNames();
  std::shared_ptr<FrameTransformer> getCurrentTransformer();
  QString getCurrentTransformerName();
  void setTransformer(const QString &  transformer_name);

Q_SIGNALS:
  void transformerChanged(std::shared_ptr<FrameTransformer> new_transformer);

private:
  std::unique_ptr<PluginlibFactory<FrameTransformer>> factory_;

  std::shared_ptr<FrameTransformer> current_transformer_;
  QString current_transformer_name_;

  ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace transformation
}  // namespace rviz_common

#endif  // RVIZ_COMMON__TRANSFORMATION__TRANSFORMATION_MANAGER_HPP_
