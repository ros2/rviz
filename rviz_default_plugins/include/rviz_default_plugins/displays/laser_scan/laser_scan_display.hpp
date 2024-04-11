/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__LASER_SCAN__LASER_SCAN_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__LASER_SCAN__LASER_SCAN_DISPLAY_HPP_

#include <memory>

#include <QObject>  //NOLINT

#include "sensor_msgs/msg/laser_scan.hpp"

#include "laser_geometry/laser_geometry.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/transformation/frame_transformer.hpp"

#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_wrapper.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <rclcpp/duration.hpp>

namespace rviz_common
{
class QueueSizeProperty;
namespace properties
{
class IntProperty;
}  // namespace properties
}  // namespace rviz_common
namespace rviz_default_plugins
{
namespace displays
{
/** @brief Visualizes a laser scan, received as a sensor_msgs::LaserScan. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC LaserScanDisplay : public
  rviz_common::MessageFilterDisplay<sensor_msgs::msg::LaserScan>
{
  Q_OBJECT

public:
  LaserScanDisplay();
  ~LaserScanDisplay() override = default;

  void reset() override;
  void update(float wall_dt, float ros_dt) override;

  void onDisable() override;

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) override;

  /** create a status warning when tolerance is larger than 1s */
  void checkTolerance(rclcpp::Duration tolerance);

  std::unique_ptr<PointCloudCommon> point_cloud_common_;
  std::unique_ptr<laser_geometry::LaserProjection> projector_;
  rclcpp::Duration filter_tolerance_;

private:
  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__LASER_SCAN__LASER_SCAN_DISPLAY_HPP_
