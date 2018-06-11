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

#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer_factory.hpp"

#include "rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/intensity_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/rgb8_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/rgbf32_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/axis_color_pc_transformer.hpp"
#include "rviz_default_plugins/displays/pointcloud/transformers/flat_color_pc_transformer.hpp"

namespace rviz_default_plugins
{

PointCloudTransformerFactory::PointCloudTransformerFactory()
: rviz_common::PluginlibFactory<PointCloudTransformer>(
    "rviz_default_plugins", "rviz_default_plugins::PointCloudTransformer")
{
  addBuiltInClass(
    "rviz_default_plugins",
    "XYZ",
    "Transforms the point cloud data into XYZ coordinates to position each point.",
    []() {return new XYZPCTransformer();});

  addBuiltInClass(
    "rviz_default_plugins",
    "Intensity",
    "Transforms the color of each point based on its \"intensity\" value.",
    []() {return new IntensityPCTransformer();});

  addBuiltInClass(
    "rviz_default_plugins",
    "RGB8",
    "Sets the color of each point based on RGB8 data.",
    []() {return new RGB8PCTransformer();});

  addBuiltInClass(
    "rviz_default_plugins",
    "RGBF32",
    "Sets the color of each point based on RGBF32 data.",
    []() {return new RGBF32PCTransformer();});

  addBuiltInClass(
    "rviz_default_plugins",
    "AxisColor",
    "Sets the color of each point based on its position along one of the X, Y, or Z axes.",
    []() {return new AxisColorPCTransformer();});

  addBuiltInClass(
    "rviz_default_plugins",
    "FlatColor",
    "Sets the color of each point to be a single flat color.",
    []() {return new FlatColorPCTransformer();});
}

}  // namespace rviz_default_plugins
