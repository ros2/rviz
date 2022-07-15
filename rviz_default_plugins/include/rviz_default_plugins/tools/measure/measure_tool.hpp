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
/*
 * measure_tool.h
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#ifndef RVIZ_DEFAULT_PLUGINS__TOOLS__MEASURE__MEASURE_TOOL_HPP_
#define RVIZ_DEFAULT_PLUGINS__TOOLS__MEASURE__MEASURE_TOOL_HPP_

#include <memory>

#include <QCursor>  // NOLINT cpplint cannot handle include order

#include <OgreVector.h>

#include "rviz_common/tool.hpp"

#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_rendering
{
class Line;
}

namespace rviz_common
{
namespace properties
{
class ColorProperty;
}
}

namespace rviz_default_plugins
{
namespace tools
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC MeasureTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  MeasureTool();

  void onInitialize() override;

  void activate() override;

  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

public Q_SLOTS:
  void updateLineColor();

private:
  void setStatusMessage();
  void processLeftButton(const Ogre::Vector3 & pos);
  void processRightButton();

  rviz_common::properties::ColorProperty * color_property_;

  std::shared_ptr<rviz_rendering::Line> line_;
  Ogre::Vector3 start_;
  Ogre::Vector3 end_;
  bool is_line_started_;
  float length_;

  QCursor std_cursor_;
  QCursor hit_cursor_;
};

}  // namespace tools
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__TOOLS__MEASURE__MEASURE_TOOL_HPP_
