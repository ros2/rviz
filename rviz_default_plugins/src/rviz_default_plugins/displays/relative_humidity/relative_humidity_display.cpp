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

#include "rviz_default_plugins/displays/relative_humidity/relative_humidity_display.hpp"

#include <memory>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_transformer.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_rendering/objects/point_cloud.hpp"

#include "rviz_common/properties/queue_size_property.hpp"
#include "rviz_common/validate_floats.hpp"

namespace rviz_default_plugins
{

    namespace displays
    {

        RelativeHumidityDisplay::RelativeHumidityDisplay()
                : queue_size_property_(new rviz_common::QueueSizeProperty(this, 10)),
                  point_cloud_common_(new PointCloudCommon(this))
        {}

        RelativeHumidityDisplay::~RelativeHumidityDisplay()
        {}

        void RelativeHumidityDisplay::onInitialize()
        {
          RTDClass::onInitialize();
          point_cloud_common_->initialize(context_, scene_node_);

          // Set correct initial values
          subProp("Channel Name")->setValue("relative_humidity");
          subProp("Autocompute Intensity Bounds")->setValue(false);
          subProp("Min Intensity")->setValue(0.0); // 0% relative humidity
          subProp("Max Intensity")->setValue(1.0); // 100% relative humidity
        }

        void RelativeHumidityDisplay::processMessage(const sensor_msgs::msg::RelativeHumidity::ConstSharedPtr msg)
        {
          // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
          // will get their points put off in lala land, but it means they still do get processed/rendered
          // which can be a big performance hit
          auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>();

          // Create fields
          sensor_msgs::msg::PointField x;
          x.name = "x";
          x.offset = 0;
          x.datatype = sensor_msgs::msg::PointField::FLOAT32;
          x.count = 1;
          sensor_msgs::msg::PointField y;
          y.name = "y";
          y.offset = 4;
          y.datatype = sensor_msgs::msg::PointField::FLOAT32;
          y.count = 1;
          sensor_msgs::msg::PointField z;
          z.name = "z";
          z.offset = 8;
          z.datatype = sensor_msgs::msg::PointField::FLOAT32;
          z.count = 1;
          sensor_msgs::msg::PointField relative_humidity;
          relative_humidity.name = "relative_humidity";
          relative_humidity.offset = 12;
          relative_humidity.datatype = sensor_msgs::msg::PointField::FLOAT64;
          relative_humidity.count = 1;

          // Create pointcloud from message
          filtered->header = msg->header;
          filtered->fields.push_back(x);
          filtered->fields.push_back(y);
          filtered->fields.push_back(z);
          filtered->fields.push_back(relative_humidity);
          filtered->data.resize(20);
          const float zero_float = 0.0;  // RelativeHumidity is always on its tf frame
          memcpy(&filtered->data[x.offset], &zero_float, 4);
          memcpy(&filtered->data[y.offset], &zero_float, 4);
          memcpy(&filtered->data[z.offset], &zero_float, 4);
          memcpy(&filtered->data[relative_humidity.offset], &msg->relative_humidity, 8);
          filtered->height = 1;
          filtered->width = 1;
          filtered->is_bigendian = false;
          filtered->point_step = 20;
          filtered->row_step = 1;

          // Give to point_cloud_common to draw
          point_cloud_common_->addMessage(filtered);
        }


        void RelativeHumidityDisplay::update(float wall_dt, float ros_dt)
        {
          point_cloud_common_->update(wall_dt, ros_dt);

          // Hide unneeded properties
          subProp("Position Transformer")->hide();
          subProp("Color Transformer")->hide();
          subProp("Channel Name")->hide();
          subProp("Autocompute Intensity Bounds")->hide();
        }

        void RelativeHumidityDisplay::reset()
        {
          RTDClass::reset();
          point_cloud_common_->reset();
        }

        void RelativeHumidityDisplay::onDisable()
        {
          RosTopicDisplay::onDisable();
          point_cloud_common_->onDisable();
        }

    }  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::RelativeHumidityDisplay, rviz_common::Display)