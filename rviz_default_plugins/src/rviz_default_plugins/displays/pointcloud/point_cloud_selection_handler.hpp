/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_SELECTION_HANDLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_SELECTION_HANDLER_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <memory>
# include <string>

# include "sensor_msgs/msg/point_cloud.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"

# include "rviz_common/selection/selection_manager.hpp"
# include "rviz_common/properties/color_property.hpp"
# include "rviz_rendering/point_cloud.hpp"
# include "rviz_common/selection/forwards.hpp"
#endif

namespace rviz_default_plugins
{

struct CloudInfo;

struct IndexAndMessage
{
  IndexAndMessage(int _index, const void * _message)
  : index(_index),
    message( (uint64_t) _message)
  {}

  int index;
  uint64_t message;
};

class PointCloudSelectionHandler : public rviz_common::selection::SelectionHandler
{
public:
  PointCloudSelectionHandler(
    float box_size,
    CloudInfo * cloud_info,
    rviz_common::DisplayContext * context);
  virtual ~PointCloudSelectionHandler();

  virtual void createProperties(
    const rviz_common::selection::Picked & obj,
    rviz_common::properties::Property * parent_property);
  virtual void destroyProperties(
    const rviz_common::selection::Picked & obj,
    rviz_common::properties::Property * parent_property);

  virtual bool needsAdditionalRenderPass(uint32_t pass)
  {
    if (pass < 2) {
      return true;
    }

    return false;
  }

  virtual void preRenderPass(uint32_t pass);
  virtual void postRenderPass(uint32_t pass);

  virtual void onSelect(const rviz_common::selection::Picked & obj);
  virtual void onDeselect(const rviz_common::selection::Picked & obj);

  virtual void getAABBs(
    const rviz_common::selection::Picked & obj,
    rviz_common::selection::V_AABB & aabbs);

  void setBoxSize(float size) {box_size_ = size;}

  bool hasSelections() {return !boxes_.empty();}

private:
  CloudInfo * cloud_info_;
  QHash<IndexAndMessage, rviz_common::properties::Property *> property_hash_;
  float box_size_;
};

}  // namespace rviz_default_plugins


#endif  // RVIZ_DEFAULT_PLUGINS__POINT_CLOUD_SELECTION_HANDLER_HPP_
