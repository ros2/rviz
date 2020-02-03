/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2018, TNG Technology Consulting GmbH.
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

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_

#include <memory>
#include <string>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"

namespace rviz_default_plugins
{

namespace displays
{

/// This is the parent class for several scalar type message displays
/// that use a point cloud to display their data
/**
 * \class PointCloudScalarDisplay
 */

template<typename MessageType>
class RVIZ_DEFAULT_PLUGINS_PUBLIC PointCloudScalarDisplay
  : public rviz_common::MessageFilterDisplay<MessageType>
{
public:
  PointCloudScalarDisplay()
  : point_cloud_common_(std::make_shared<PointCloudCommon>(this))
  {}

  ~PointCloudScalarDisplay() override = default;

  std::shared_ptr<sensor_msgs::msg::PointCloud2> createPointCloud2Message(
    const std_msgs::msg::Header & header, double scalar_value, const std::string & channelName)
  {
    auto point_cloud_message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    unsigned int field_size_total = 0;

    point_cloud_message->header = header;

    field_size_total = addFieldsAndReturnSize(point_cloud_message, channelName);
    point_cloud_message->data.resize(field_size_total);

    copyCoordinates(point_cloud_message);
    copyScalarValue(point_cloud_message, scalar_value);

    point_cloud_message->height = 1;
    point_cloud_message->width = 1;
    point_cloud_message->is_bigendian = false;
    point_cloud_message->point_step = field_size_total;
    point_cloud_message->row_step = 1;

    return point_cloud_message;
  }

protected:
  virtual void setInitialValues() = 0;
  virtual void hideUnneededProperties() = 0;

  std::shared_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common_;

private:
  void onInitialize() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onInitialize();
    point_cloud_common_->initialize(
      this->context_, this->scene_node_);
    setInitialValues();
  }

  void update(float wall_dt, float ros_dt) override
  {
    point_cloud_common_->update(wall_dt, ros_dt);
    hideUnneededProperties();
  }

  void onEnable() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onEnable();
  }

  void onDisable() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onDisable();
    point_cloud_common_->onDisable();
  }

  void reset() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::reset();
    point_cloud_common_->reset();
  }

  int addFieldsAndReturnSize(
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message,
    const std::string & channelName)
  {
    unsigned int field_size_increment = 0;

    field_size_increment =
      addField32andReturnOffset(point_cloud_message, field_size_increment, "x");
    field_size_increment =
      addField32andReturnOffset(point_cloud_message, field_size_increment, "y");
    field_size_increment =
      addField32andReturnOffset(point_cloud_message, field_size_increment, "z");
    field_size_increment =
      addField64andReturnOffset(point_cloud_message, field_size_increment, channelName);
    return field_size_increment;
  }

  int addField32andReturnOffset(
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message,
    unsigned int offset, std::string field_name)
  {
    sensor_msgs::msg::PointField field;
    field.name = field_name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    point_cloud_message->fields.push_back(field);
    offset += field_size_32_;

    return offset;
  }

  int addField64andReturnOffset(
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message,
    unsigned int offset, std::string field_name)
  {
    sensor_msgs::msg::PointField field;
    field.name = field_name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT64;
    field.count = 1;
    point_cloud_message->fields.push_back(field);
    offset += field_size_64_;

    return offset;
  }

  void copyCoordinates(std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message)
  {
    float coordinate_value = 0.0;

    for (int i = 0; i < 3; i++) {
      memcpy(
        &point_cloud_message->data[point_cloud_message->fields[i].offset],
        &coordinate_value, field_size_32_);
    }
  }

  void copyScalarValue(
    std::shared_ptr<sensor_msgs::msg::PointCloud2> point_cloud_message, double scalar_value)
  {
    memcpy(
      &point_cloud_message->data[point_cloud_message->fields[3].offset],
      &scalar_value, field_size_64_);
  }

  const unsigned int field_size_32_ = 4;
  const unsigned int field_size_64_ = 8;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_SCALAR_DISPLAY_HPP_
