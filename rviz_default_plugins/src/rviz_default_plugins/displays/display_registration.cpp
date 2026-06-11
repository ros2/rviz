// Copyright (c) 2024, Open Source Robotics Foundation, Inc.
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

// Consolidated pluginlib exports for all default display plugins.
// Keeping PLUGINLIB_EXPORT_CLASS in a single TU means pluginlib/class_list_macros.hpp
// (~347 headers) is parsed exactly once instead of once per display .cpp file.
// class_loader requires complete types at macro expansion, so all display headers
// are included here.  Include guards ensure each header is only parsed once even
// though they transitively share many common dependencies.

#include "rviz_default_plugins/displays/accel/accel_display.hpp"
#include "rviz_default_plugins/displays/axes/axes_display.hpp"
#include "rviz_default_plugins/displays/camera/camera_display.hpp"
#include "rviz_default_plugins/displays/camera_info/camera_info_display.hpp"
#include "rviz_default_plugins/displays/depth_cloud/depth_cloud_display.hpp"
#include "rviz_default_plugins/displays/effort/effort_display.hpp"
#include "rviz_default_plugins/displays/fluid_pressure/fluid_pressure_display.hpp"
#include "rviz_default_plugins/displays/grid/grid_display.hpp"
#include "rviz_default_plugins/displays/grid_cells/grid_cells_display.hpp"
#include "rviz_default_plugins/displays/illuminance/illuminance_display.hpp"
#include "rviz_default_plugins/displays/image/image_display.hpp"
#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_display.hpp"
#include "rviz_default_plugins/displays/laser_scan/laser_scan_display.hpp"
#include "rviz_default_plugins/displays/map/map_display.hpp"
#include "rviz_default_plugins/displays/marker/marker_display.hpp"
#include "rviz_default_plugins/displays/marker_array/marker_array_display.hpp"
#include "rviz_default_plugins/displays/odometry/odometry_display.hpp"
#include "rviz_default_plugins/displays/path/path_display.hpp"
#include "rviz_default_plugins/displays/point/point_stamped_display.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud2_display.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_display.hpp"
#include "rviz_default_plugins/displays/polygon/polygon_display.hpp"
#include "rviz_default_plugins/displays/pose/pose_display.hpp"
#include "rviz_default_plugins/displays/pose_array/pose_array_display.hpp"
#include "rviz_default_plugins/displays/pose_covariance/pose_with_covariance_display.hpp"
#include "rviz_default_plugins/displays/range/range_display.hpp"
#include "rviz_default_plugins/displays/relative_humidity/relative_humidity_display.hpp"
#include "rviz_default_plugins/displays/robot_model/robot_model_display.hpp"
#include "rviz_default_plugins/displays/temperature/temperature_display.hpp"
#include "rviz_default_plugins/displays/tf/tf_display.hpp"
#include "rviz_default_plugins/displays/twist/twist_display.hpp"
#include "rviz_default_plugins/displays/wrench/wrench_display.hpp"

#include <pluginlib/class_list_macros.hpp>  // NOLINT(build/include_order)

// clang-format off
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::AccelStampedDisplay,          rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::AxesDisplay,               rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::CameraDisplay,             rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::CameraInfoDisplay,         rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::DepthCloudDisplay,         rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::EffortDisplay,             rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::FluidPressureDisplay,      rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::GridDisplay,               rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::GridCellsDisplay,          rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::IlluminanceDisplay,        rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::ImageDisplay,              rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::InteractiveMarkerDisplay,  rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::LaserScanDisplay,          rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MapDisplay,                rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MarkerDisplay,             rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::MarkerArrayDisplay,        rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::OdometryDisplay,           rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PathDisplay,               rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointCloud2Display,        rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointCloudDisplay,         rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PointStampedDisplay,       rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PolygonDisplay,            rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseDisplay,               rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseArrayDisplay,          rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::PoseWithCovarianceDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RangeDisplay,              rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RelativeHumidityDisplay,   rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RobotModelDisplay,         rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TemperatureDisplay,        rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TFDisplay,                 rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::TwistStampedDisplay,       rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::WrenchDisplay,             rviz_common::Display)
// clang-format on
