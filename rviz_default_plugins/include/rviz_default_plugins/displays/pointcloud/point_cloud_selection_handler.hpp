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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SELECTION_HANDLER_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SELECTION_HANDLER_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <memory>
# include <set>
# include <string>

# include "sensor_msgs/msg/point_cloud.hpp"
# include "sensor_msgs/msg/point_cloud2.hpp"

# include "rviz_common/interaction/forwards.hpp"
# include "rviz_common/interaction/selection_manager.hpp"
# include "rviz_common/properties/color_property.hpp"
# include "rviz_rendering/objects/point_cloud.hpp"
# include "rviz_default_plugins/visibility_control.hpp"
#endif

namespace rviz_default_plugins
{

struct CloudInfo;

struct IndexAndMessage
{
  IndexAndMessage(uint64_t _index, const void * _message)
  : index(_index),
    message( (uint64_t) _message)
  {}

  uint64_t index;
  uint64_t message;
};

class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudSelectionHandler : public
  rviz_common::interaction::SelectionHandler
{
public:
  PointCloudSelectionHandler(
    float box_size,
    CloudInfo * cloud_info,
    rviz_common::DisplayContext * context);
  ~PointCloudSelectionHandler() override;

  void createProperties(
    const rviz_common::interaction::Picked & obj,
    rviz_common::properties::Property * parent_property) override;
  void destroyProperties(
    const rviz_common::interaction::Picked & obj,
    rviz_common::properties::Property * parent_property) override;

  bool needsAdditionalRenderPass(uint32_t pass) override
  {
    return pass < 2;
  }

  void preRenderPass(uint32_t pass) override;
  void postRenderPass(uint32_t pass) override;

  void onSelect(const rviz_common::interaction::Picked & obj) override;
  void onDeselect(const rviz_common::interaction::Picked & obj) override;

  rviz_common::interaction::V_AABB getAABBs(const rviz_common::interaction::Picked & obj) override;

  void setBoxSize(float size) {box_size_ = size;}

  bool hasSelections() {return !boxes_.empty();}

private:
  typedef std::set<uint64_t> S_int;

  uint64_t handleToIndex(uint64_t handle) const
  {
    return (handle & 0xffffffff) - 1;
  }

  S_int getIndicesOfSelectedPoints(const rviz_common::interaction::Picked & obj)
  {
    S_int indices;
    for (auto handle : obj.extra_handles) {
      indices.insert(handleToIndex(handle));
    }
    return indices;
  }

  rviz_common::properties::Property * createParentPropertyForPoint(
    rviz_common::properties::Property * parent_property,
    uint64_t index,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message);

  void
  addPositionProperty(rviz_common::properties::Property * parent, uint64_t index) const;

  void addAdditionalProperties(
    rviz_common::properties::Property * parent,
    uint64_t index,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const;

  uint32_t convertValueToColor(
    uint64_t index,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
    const sensor_msgs::msg::PointField & f) const;

  void addColorProperty(
    rviz_common::properties::Property * parent,
    size_t field,
    const std::string & name,
    uint32_t val) const;

  void addAlphaProperty(rviz_common::properties::Property * parent, uint32_t val) const;

  void addIntensityProperty(
    rviz_common::properties::Property * parent,
    size_t field,
    const std::string & name,
    float val) const;

  CloudInfo * cloud_info_;
  QHash<IndexAndMessage, rviz_common::properties::Property *> property_hash_;
  float box_size_;

  template<typename T, typename ... Args>
  friend typename std::shared_ptr<T>
  rviz_common::interaction::createSelectionHandler(Args ... arguments);
};

}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SELECTION_HANDLER_HPP_
