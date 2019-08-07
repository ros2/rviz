/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
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


#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID_CELLS__GRID_CELLS_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID_CELLS__GRID_CELLS_DISPLAY_HPP_

#include <memory>

#include "nav_msgs/msg/grid_cells.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

#include "rviz_common/display.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class PointCloud;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
class FloatProperty;
}  // properties
}  // rviz_common

namespace rviz_default_plugins
{
namespace displays
{

// TODO(Martin-Idel-SI): This display previously used tf message filter. Use again once available.
/**
 * \class GridCellsDisplay
 * \brief Displays a nav_msgs::GridCells message
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC GridCellsDisplay : public
  rviz_common::MessageFilterDisplay<nav_msgs::msg::GridCells>
{
  Q_OBJECT

public:
  // TODO(Martin-Idel-SI): Constructor for testing. Remove once ros nodes can be mocked and
  // initialize() can be called
  explicit GridCellsDisplay(rviz_common::DisplayContext * display_context);

  GridCellsDisplay();

  ~GridCellsDisplay() override;

  void onInitialize() override;

  void processMessage(nav_msgs::msg::GridCells::ConstSharedPtr msg) override;

  void setupCloud();

private Q_SLOTS:
  void updateAlpha();
  void updateColor();

private:
  bool messageIsValid(nav_msgs::msg::GridCells::ConstSharedPtr msg);
  void convertMessageToCloud(nav_msgs::msg::GridCells::ConstSharedPtr msg);
  bool setTransform(std_msgs::msg::Header const & header);

  std::shared_ptr<rviz_rendering::PointCloud> cloud_;

  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * alpha_property_;

  uint64_t last_frame_count_;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__GRID_CELLS__GRID_CELLS_DISPLAY_HPP_
